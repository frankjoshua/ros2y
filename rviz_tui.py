#!/usr/bin/env python3
"""rviz in the terminal: map + scan + plan + robot pose, keyboard teleop, click-to-goal / waypoints.

Run inside the dev container:  python3 rviz_tui.py        (TUI)
Self-check (no UI):            python3 rviz_tui.py --check

Keys: w/s fwd/back  a/d turn  space stop  [ ] speed
      m waypoint mode (click adds, enter sends FollowWaypoints, c clears)
      click = send /goal_pose (when not in waypoint mode)
      f follow robot  h/j/k/l pan  + / - zoom  q quit
"""
import math
import sys
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

try:
    from nav2_msgs.action import FollowWaypoints, NavigateToPose
except ImportError:  # nav2_msgs not installed; goal/waypoint sending disabled
    FollowWaypoints = NavigateToPose = None


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class RosBridge(Node):
    """Holds latest messages; publishes teleop twist; sends goals/waypoints."""

    def __init__(self):
        super().__init__("rviz_tui")
        self.msgs = {}      # topic -> latest msg
        self.stamps = {}    # topic -> wall time received
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        subs = [
            ("/map", OccupancyGrid, latched),
            ("/scan", LaserScan, qos_profile_sensor_data),
            ("/pose", None, 10),
            ("/plan_smoothed", Path, 10),
            ("/transformed_global_plan", Path, 10),
            ("/odom", Odometry, qos_profile_sensor_data),
        ]
        from geometry_msgs.msg import PoseWithCovarianceStamped
        for topic, typ, qos in subs:
            typ = typ or PoseWithCovarianceStamped
            self.create_subscription(typ, topic, self._store(topic), qos)

        # /pose is only published on slam updates; live pose comes from composing
        # map->odom (slam_toolbox) with odom->base_link off /tf instead.
        from tf2_msgs.msg import TFMessage
        self.tf = {}  # (frame, child) -> (x, y, yaw)
        self.create_subscription(TFMessage, "/tf", self._tf_cb, qos_profile_sensor_data)

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        # /goal_pose has no subscribers on this robot; goals go through the NavigateToPose action
        self.nav_client = (
            ActionClient(self, NavigateToPose, "navigate_to_pose") if NavigateToPose else None
        )
        self.twist = Twist()
        self.teleop_active = False
        self.last_key_time = 0.0
        self.create_timer(0.1, self._teleop_tick)
        self.wp_client = (
            ActionClient(self, FollowWaypoints, "follow_waypoints") if FollowWaypoints else None
        )
        self.wp_status = ""

    def _tf_cb(self, msg):
        for t in msg.transforms:
            tr = t.transform
            self.tf[(t.header.frame_id, t.child_frame_id)] = (
                tr.translation.x, tr.translation.y, yaw_of(tr.rotation))
        self.stamps["/tf"] = time.time()

    def _store(self, topic):
        def cb(msg):
            self.msgs[topic] = msg
            self.stamps[topic] = time.time()
        return cb

    # -- teleop: publish only while engaged so the twist mux isn't starved with zeros
    def _teleop_tick(self):
        if not self.teleop_active:
            return
        # ponytail: 2s deadman — no keypress for 2s stops the robot; tune if key repeat feels short
        if time.time() - self.last_key_time > 2.0:
            self.stop()
            return
        self.cmd_pub.publish(self.twist)

    def drive(self, dlin, dang, lin_step, ang_step):
        self.twist.linear.x = round(self.twist.linear.x + dlin * lin_step, 3)
        self.twist.angular.z = round(self.twist.angular.z + dang * ang_step, 3)
        self.teleop_active = True
        self.last_key_time = time.time()

    def stop(self):
        self.twist = Twist()
        if self.teleop_active:
            self.cmd_pub.publish(self.twist)  # one explicit zero, then release the mux
        self.teleop_active = False

    def pose_xyyaw(self):
        ob = self.tf.get(("odom", "base_link")) or self.tf.get(("odom", "base_footprint"))
        if ob:
            mx, my, myaw = self.tf.get(("map", "odom"), (0.0, 0.0, 0.0))
            c, s = math.cos(myaw), math.sin(myaw)
            return (mx + c * ob[0] - s * ob[1], my + s * ob[0] + c * ob[1], myaw + ob[2])
        m = self.msgs.get("/pose")  # fallback: last slam update
        if not m:
            return None
        p = m.pose.pose
        return p.position.x, p.position.y, yaw_of(p.orientation)

    def _stamped(self, wx, wy, yaw):
        g = PoseStamped()
        g.header.frame_id = "map"
        g.header.stamp = self.get_clock().now().to_msg()
        g.pose.position.x = wx
        g.pose.position.y = wy
        g.pose.orientation.z = math.sin(yaw / 2)
        g.pose.orientation.w = math.cos(yaw / 2)
        return g

    def send_goal(self, wx, wy):
        if not self.nav_client or not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.wp_status = "no navigate_to_pose server"
            return
        me = self.pose_xyyaw()
        yaw = math.atan2(wy - me[1], wx - me[0]) if me else 0.0
        goal = NavigateToPose.Goal()
        goal.pose = self._stamped(wx, wy, yaw)
        self.wp_status = "navigating"
        fut = self.nav_client.send_goal_async(goal)

        def on_goal(gh_fut):
            gh = gh_fut.result()
            if not gh.accepted:
                self.wp_status = "goal rejected"
                return
            gh.get_result_async().add_done_callback(
                lambda rf: setattr(self, "wp_status", "goal reached")
            )
        fut.add_done_callback(on_goal)

    def send_waypoints(self, pts):
        if not self.wp_client:
            self.wp_status = "nav2_msgs missing"
            return
        if not self.wp_client.wait_for_server(timeout_sec=1.0):
            self.wp_status = "no follow_waypoints server"
            return
        goal = FollowWaypoints.Goal()
        prev = self.pose_xyyaw() or (0, 0, 0)
        for wx, wy in pts:
            yaw = math.atan2(wy - prev[1], wx - prev[0])
            goal.poses.append(self._stamped(wx, wy, yaw))
            prev = (wx, wy, yaw)
        self.wp_status = f"following {len(pts)} wp"
        fut = self.wp_client.send_goal_async(goal)

        def on_result(gh_fut):
            gh = gh_fut.result()
            if not gh.accepted:
                self.wp_status = "rejected"
                return
            gh.get_result_async().add_done_callback(
                lambda rf: setattr(self, "wp_status", "waypoints done")
            )
        fut.add_done_callback(on_result)


# ---------------------------------------------------------------- self-check
def check():
    rclpy.init()
    node = RosBridge()
    end = time.time() + 6
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    got = sorted(node.msgs)
    print("received:", got or "nothing")
    assert "/map" in got and "/scan" in got, "missing core topics"
    assert node.pose_xyyaw() is not None, "no pose (tf or /pose)"
    print("pose:", tuple(round(v, 2) for v in node.pose_xyyaw()))
    m = node.msgs["/map"]
    assert len(m.data) == m.info.width * m.info.height
    print(f"map {m.info.width}x{m.info.height} @ {m.info.resolution}m")
    print("self-check OK")
    rclpy.shutdown()


if "--check" in sys.argv:
    check()
    sys.exit(0)

# ---------------------------------------------------------------- TUI
from rich.segment import Segment
from rich.style import Style
from textual.app import App, ComposeResult
from textual.strip import Strip
from textual.widget import Widget
from textual.widgets import Static

C_UNKNOWN = (40, 42, 48)
C_FREE = (16, 17, 20)
C_WALL = (200, 205, 215)
C_SCAN = (235, 80, 80)
C_PLAN = (80, 140, 235)
C_ROBOT = (60, 220, 220)
C_GOAL = (80, 220, 100)
C_WP = (235, 200, 60)


class MapView(Widget):
    can_focus = False

    def __init__(self, ros, app_state):
        super().__init__()
        self.ros = ros
        self.st = app_state
        self.follow = True
        self.center = (0.0, 0.0)   # world meters
        self.ppm = 20.0            # pixels per meter (1 px = 1 half-block)
        self._buf = None
        self._style_cache = {}

    # world -> pixel (pixel origin = top-left of buffer)
    def _w2p(self, wx, wy, w, h):
        px = (wx - self.center[0]) * self.ppm + w / 2
        py = h / 2 - (wy - self.center[1]) * self.ppm
        return int(px), int(py)

    def _compose_buffer(self):
        w = self.size.width
        h = self.size.height * 2
        if w < 4 or h < 4:
            return None
        ros = self.ros
        if self.follow:
            me = ros.pose_xyyaw()
            if me:
                self.center = (me[0], me[1])
        buf = np.empty((h, w, 3), np.uint8)
        buf[:] = C_UNKNOWN

        grid = ros.msgs.get("/map")
        if grid is not None:
            info = grid.info
            occ = np.frombuffer(bytes(grid.data), np.int8).reshape(info.height, info.width)
            # world coords of each buffer pixel -> map cell index (vectorized gather)
            xs = (np.arange(w) - w / 2) / self.ppm + self.center[0]
            ys = self.center[1] - (np.arange(h) - h / 2) / self.ppm
            ix = ((xs - info.origin.position.x) / info.resolution).astype(int)
            iy = ((ys - info.origin.position.y) / info.resolution).astype(int)
            valid = (ix >= 0) & (ix < info.width)
            validy = (iy >= 0) & (iy < info.height)
            cells = occ[np.clip(iy, 0, info.height - 1)[:, None],
                        np.clip(ix, 0, info.width - 1)[None, :]]
            cells[~validy[:, None] | ~valid[None, :]] = -1
            buf[cells == -1] = C_UNKNOWN
            buf[(cells >= 0) & (cells < 50)] = C_FREE
            buf[cells >= 50] = C_WALL

        me = ros.pose_xyyaw()

        def put(wx, wy, color, r=0):
            px, py = self._w2p(wx, wy, w, h)
            buf[max(0, py - r):py + r + 1, max(0, px - r):px + r + 1] = color

        plan = ros.msgs.get("/plan_smoothed") or ros.msgs.get("/transformed_global_plan")
        if plan:
            for p in plan.poses:
                put(p.pose.position.x, p.pose.position.y, C_PLAN)

        scan = ros.msgs.get("/scan")
        if scan and me:
            rx, ry, ryaw = me
            rng = np.asarray(scan.ranges, np.float32)
            ang = scan.angle_min + np.arange(len(rng)) * scan.angle_increment + ryaw
            ok = np.isfinite(rng) & (rng > scan.range_min) & (rng < scan.range_max)
            # ponytail: laser assumed at base origin, no mount offset; add x/y/yaw offsets if scan looks rotated
            pxs = ((rx + rng[ok] * np.cos(ang[ok]) - self.center[0]) * self.ppm + w / 2).astype(int)
            pys = (h / 2 - (ry + rng[ok] * np.sin(ang[ok]) - self.center[1]) * self.ppm).astype(int)
            keep = (pxs >= 0) & (pxs < w) & (pys >= 0) & (pys < h)
            buf[pys[keep], pxs[keep]] = C_SCAN

        for i, (wx, wy) in enumerate(self.st["waypoints"]):
            put(wx, wy, C_WP, r=1)
        if self.st["goal"]:
            put(*self.st["goal"], C_GOAL, r=1)

        if me:
            rx, ry, ryaw = me
            put(rx, ry, C_ROBOT, r=1)
            for d in np.linspace(0, 0.45, 8):  # heading whisker
                put(rx + d * math.cos(ryaw), ry + d * math.sin(ryaw), C_ROBOT)
        return buf

    def render_lines(self, crop):
        self._buf = self._compose_buffer()
        return super().render_lines(crop)

    def render_line(self, y):
        buf = self._buf
        if buf is None or 2 * y + 1 >= buf.shape[0]:
            return Strip([])
        top, bot = buf[2 * y], buf[2 * y + 1]
        segs = []
        cache = self._style_cache
        run_start = 0
        keys = (top.astype(np.uint32) << 12) ^ bot.astype(np.uint32)  # cheap pair id per column
        keys = keys[:, 0] * 7 + keys[:, 1] * 13 + keys[:, 2]
        for x in range(1, len(keys) + 1):
            if x == len(keys) or keys[x] != keys[run_start]:
                t, b = tuple(top[run_start]), tuple(bot[run_start])
                style = cache.get((t, b))
                if style is None:
                    style = cache[(t, b)] = Style(
                        color=f"rgb({t[0]},{t[1]},{t[2]})", bgcolor=f"rgb({b[0]},{b[1]},{b[2]})"
                    )
                segs.append(Segment("▀" * (x - run_start), style))
                run_start = x
        return Strip(segs)

    def on_mouse_down(self, event):
        w, h = self.size.width, self.size.height * 2
        wx = self.center[0] + (event.x - w / 2) / self.ppm
        wy = self.center[1] - (event.y * 2 - h / 2) / self.ppm
        if self.st["wp_mode"]:
            self.st["waypoints"].append((wx, wy))
        else:
            self.st["goal"] = (wx, wy)
            self.ros.send_goal(wx, wy)

    def pan(self, dx, dy):
        self.follow = False
        self.center = (self.center[0] + dx / self.ppm * 8, self.center[1] + dy / self.ppm * 8)

    def zoom(self, factor):
        self.ppm = min(200.0, max(2.0, self.ppm * factor))


class RvizTui(App):
    CSS = """
    Screen { layout: horizontal; }
    MapView { width: 1fr; height: 100%; }
    #side { width: 34; height: 100%; padding: 1; background: $surface; }
    """
    BINDINGS = [("q", "quit", "quit")]

    def __init__(self, ros):
        super().__init__()
        self.ros = ros
        self.state = {"wp_mode": False, "waypoints": [], "goal": None}
        self.lin_step, self.ang_step = 0.1, 0.2

    def compose(self) -> ComposeResult:
        self.map_view = MapView(self.ros, self.state)
        self.side = Static(id="side")
        yield self.map_view
        yield self.side

    def on_mount(self):
        self.set_interval(0.2, self.map_view.refresh)
        self.set_interval(0.5, self._sidebar)

    def _sidebar(self):
        ros = self.ros
        me = ros.pose_xyyaw()
        odom = ros.msgs.get("/odom")
        v = odom.twist.twist if odom else None
        age = lambda t: f"{time.time() - ros.stamps[t]:4.1f}s" if t in ros.stamps else "  --"
        lines = [
            "[b]rviz-tui[/b]",
            "",
            f"pose  {me[0]:6.2f} {me[1]:6.2f} {math.degrees(me[2]):5.0f}°" if me else "pose  --",
            f"vel   {v.linear.x:5.2f} m/s  {v.angular.z:5.2f} r/s" if v else "vel   --",
            "",
            f"[b]teleop[/b] {'[red]ACTIVE[/red]' if ros.teleop_active else 'off'}",
            f"cmd   {ros.twist.linear.x:5.2f} / {ros.twist.angular.z:5.2f}",
            f"step  {self.lin_step:.2f} m/s  [ ] adjust",
            "",
            f"[b]mode[/b]  {'[yellow]WAYPOINT[/yellow]' if self.state['wp_mode'] else 'goal (click)'}",
            f"wps   {len(self.state['waypoints'])}   {ros.wp_status}",
            "",
            "[b]topics[/b]",
            *(f" {t:<16}{age(t)}" for t in ("/map", "/scan", "/pose", "/plan_smoothed", "/odom")),
            "",
            "w/s a/d drive · space stop",
            "m wp-mode · enter send · c clear",
            "f follow · hjkl pan · +- zoom",
        ]
        self.side.update("\n".join(lines))

    def on_key(self, event):
        k = event.key
        ros, mv = self.ros, self.map_view
        drives = {"w": (1, 0), "s": (-1, 0), "a": (0, 1), "d": (0, -1),
                  "up": (1, 0), "down": (-1, 0), "left": (0, 1), "right": (0, -1)}
        if k in drives:
            ros.drive(*drives[k], self.lin_step, self.ang_step)
        elif k in ("space", "x"):
            ros.stop()
        elif k == "left_square_bracket":
            self.lin_step = max(0.05, self.lin_step - 0.05)
        elif k == "right_square_bracket":
            self.lin_step += 0.05
        elif k == "m":
            self.state["wp_mode"] = not self.state["wp_mode"]
        elif k == "enter" and self.state["waypoints"]:
            ros.send_waypoints(self.state["waypoints"])
        elif k == "c":
            self.state["waypoints"].clear()
        elif k == "f":
            mv.follow = True
        elif k in "hjkl":
            dx, dy = {"h": (-1, 0), "l": (1, 0), "j": (0, -1), "k": (0, 1)}[k]
            mv.pan(dx, dy)
        elif k in ("plus", "equals_sign", "equals"):
            mv.zoom(1.5)
        elif k in ("minus", "hyphen"):
            mv.zoom(1 / 1.5)


def main():
    rclpy.init()
    ros = RosBridge()
    spinner = threading.Thread(target=rclpy.spin, args=(ros,), daemon=True)
    spinner.start()
    app = RvizTui(ros)
    try:
        app.run()
    finally:
        ros.stop()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
