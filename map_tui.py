#!/usr/bin/env python3
"""Live occupancy-grid TUI: renders /map with half-block unicode, lidar overlay, robot arrow.

Keys: q quit | +/- zoom | arrows pan | f toggle follow-robot | c re-center/auto-fit
Check: python3 map_tui.py --snapshot   (one ASCII frame to stdout, no curses)
"""
import curses
import locale
import math
import sys
import threading
import time

import numpy as np
import rclpy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

UNK, FREE, WALL, SCAN = 0, 1, 2, 3
ARROWS = '→↗↑↖←↙↓↘'  # by yaw octant, ccw from +x


class MapTui(Node):
    def __init__(self):
        super().__init__('map_tui')
        self.grid = None       # (info, HxW int8)
        self.scan = None
        self.base_frame = None
        map_qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, map_qos)
        self.create_subscription(LaserScan, '/scan', self._on_scan, qos_profile_sensor_data)
        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf, self)

    def _on_map(self, msg):
        arr = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.grid = (msg.info, arr)

    def _on_scan(self, msg):
        self.scan = msg

    def _pose_of(self, frame):
        t = self.tf_buf.lookup_transform('map', frame, rclpy.time.Time())
        q = t.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return t.transform.translation.x, t.transform.translation.y, yaw

    def robot_pose(self):
        for frame in [self.base_frame] if self.base_frame else ['base_link', 'base_footprint']:
            try:
                pose = self._pose_of(frame)
                self.base_frame = frame
                return pose
            except Exception:
                continue
        return None

    def scan_points(self):
        s = self.scan
        if s is None:
            return np.empty(0), np.empty(0)
        try:
            ox, oy, yaw = self._pose_of(s.header.frame_id)
        except Exception:
            return np.empty(0), np.empty(0)
        r = np.asarray(s.ranges, dtype=np.float32)
        a = s.angle_min + np.arange(len(r)) * s.angle_increment + yaw
        ok = np.isfinite(r) & (r >= s.range_min) & (r <= s.range_max)
        return ox + r[ok] * np.cos(a[ok]), oy + r[ok] * np.sin(a[ok])


def classify(arr):
    cls = np.full(arr.shape, UNK, np.uint8)
    cls[(arr >= 0) & (arr < 50)] = FREE
    cls[arr >= 50] = WALL
    return cls


def reduce_grid(cls, s):
    """Max-pool by s so walls survive downsampling. Returns screen-oriented (row 0 = top)."""
    h, w = cls.shape
    ph, pw = -(-h // s), -(-w // s)
    padded = np.zeros((ph * s, pw * s), np.uint8)
    padded[:h, :w] = cls
    return padded.reshape(ph, s, pw, s).max(axis=(1, 3))[::-1]


def compose(node, pw, ph, scale, center, follow):
    """Build a ph x pw class canvas + robot char position. center is in map cells (x, y)."""
    info, arr = node.grid
    res = info.resolution
    reduced = reduce_grid(classify(arr), scale)
    hr, wr = reduced.shape

    pose = node.robot_pose()
    if follow and pose:
        center = ((pose[0] - info.origin.position.x) / res,
                  (pose[1] - info.origin.position.y) / res)

    # view top-left in reduced (screen-oriented) pixel coords
    x0 = int(center[0] / scale - pw / 2)
    y0 = int((hr - 1 - center[1] / scale) - ph / 2)

    canvas = np.full((ph, pw), UNK, np.uint8)
    sx, sy = max(0, -x0), max(0, -y0)
    mx, my = max(0, x0), max(0, y0)
    w, h = min(pw - sx, wr - mx), min(ph - sy, hr - my)
    if w > 0 and h > 0:
        canvas[sy:sy + h, sx:sx + w] = reduced[my:my + h, mx:mx + w]

    def to_px(wx, wy):
        rx = (wx - info.origin.position.x) / res / scale
        ry = hr - 1 - (wy - info.origin.position.y) / res / scale
        return rx - x0, ry - y0

    xs, ys = node.scan_points()
    if len(xs):
        px, py = to_px(xs, ys)
        px, py = px.astype(int), py.astype(int)
        ok = (px >= 0) & (px < pw) & (py >= 0) & (py < ph)
        canvas[py[ok], px[ok]] = SCAN

    robot = None
    if pose:
        px, py = to_px(pose[0], pose[1])
        if 0 <= int(px) < pw and 0 <= int(py) < ph:
            octant = int(round(pose[2] / (math.pi / 4))) % 8
            robot = (int(py), int(px), ARROWS[octant])
    return canvas, robot, pose, center


def auto_scale(node, pw, ph):
    info, arr = node.grid
    return max(1, -(-arr.shape[0] // ph), -(-arr.shape[1] // pw))


def snapshot():
    rclpy.init()
    node = MapTui()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    deadline = time.time() + 15
    while node.grid is None and time.time() < deadline:
        time.sleep(0.2)
    assert node.grid is not None, 'no /map received in 15s'
    info, arr = node.grid
    pw, ph = 110, 44
    scale = auto_scale(node, pw, ph)
    center = (arr.shape[1] / 2, arr.shape[0] / 2)
    canvas, robot, pose, _ = compose(node, pw, ph, scale, center, follow=False)
    chars = {UNK: ' ', FREE: '·', WALL: '█', SCAN: '*'}
    lines = [''.join(chars[c] for c in row) for row in canvas]
    if robot:
        r, c, ch = robot
        lines[r] = lines[r][:c] + ch + lines[r][c + 1:]
    print('\n'.join(lines))
    print(f"map {arr.shape[1]}x{arr.shape[0]} @ {info.resolution:.3f} m/cell, scale 1:{scale}, "
          f"robot {'(%.2f, %.2f)' % pose[:2] if pose else 'n/a'}, scan pts drawn: {(canvas == SCAN).sum()}")
    assert (canvas == WALL).any(), 'map received but no walls rendered'
    rclpy.shutdown()


def tui(stdscr, node):
    curses.curs_set(0)
    stdscr.nodelay(True)
    curses.start_color()
    curses.use_default_colors()
    c256 = curses.COLORS >= 256
    col = {UNK: 16 if c256 else curses.COLOR_BLACK,
           FREE: 237 if c256 else curses.COLOR_BLUE,
           WALL: 255 if c256 else curses.COLOR_WHITE,
           SCAN: 196 if c256 else curses.COLOR_RED}
    for t in range(4):
        for b in range(4):
            curses.init_pair(1 + t * 4 + b, col[t], col[b])
    curses.init_pair(17, 201 if c256 else curses.COLOR_MAGENTA, -1)

    scale, center, follow = None, None, True
    while True:
        k = stdscr.getch()
        if k in (ord('q'), 27):
            break
        elif k in (ord('+'), ord('=')) and scale:
            scale = max(1, scale - 1)
        elif k == ord('-') and scale:
            scale += 1
        elif k == ord('f'):
            follow = not follow
        elif k == ord('c'):
            scale, center, follow = None, None, False
        elif k in (curses.KEY_UP, curses.KEY_DOWN, curses.KEY_LEFT, curses.KEY_RIGHT) and center:
            follow = False
            dx = {curses.KEY_LEFT: -1, curses.KEY_RIGHT: 1}.get(k, 0)
            dy = {curses.KEY_DOWN: -1, curses.KEY_UP: 1}.get(k, 0)
            step = 8 * (scale or 1)
            center = (center[0] + dx * step, center[1] + dy * step)

        rows, cols = stdscr.getmaxyx()
        pw, ph = cols - 1, 2 * (rows - 1)
        if node.grid is None:
            stdscr.erase()
            stdscr.addstr(0, 0, 'waiting for /map ...')
            stdscr.refresh()
            time.sleep(0.2)
            continue

        info, arr = node.grid
        if scale is None:
            scale = auto_scale(node, pw, ph)
        if center is None:
            center = (arr.shape[1] / 2, arr.shape[0] / 2)

        canvas, robot, pose, center = compose(node, pw, ph, scale, center, follow)
        stdscr.erase()
        for r in range(rows - 1):
            top, bot = canvas[2 * r], canvas[2 * r + 1] if 2 * r + 1 < ph else canvas[2 * r] * 0
            line = []
            prev = -1
            for x in range(pw):
                pair = 1 + int(top[x]) * 4 + int(bot[x])
                if pair != prev:
                    line.append((x, pair))
                    prev = pair
            # draw runs of same color pair
            for i, (x, pair) in enumerate(line):
                end = line[i + 1][0] if i + 1 < len(line) else pw
                try:
                    stdscr.addstr(r, x, '▀' * (end - x), curses.color_pair(pair))
                except curses.error:
                    pass
        if robot:
            r, c, ch = robot
            try:
                stdscr.addstr(r // 2, c, ch, curses.color_pair(17) | curses.A_BOLD)
            except curses.error:
                pass
        status = (f" {arr.shape[1]}x{arr.shape[0]} @{info.resolution:.2f}m 1:{scale}"
                  f"{' FOLLOW' if follow else ''}"
                  f"  robot {'(%.2f, %.2f)' % pose[:2] if pose else '?'}"
                  f"  [q]uit [+/-]zoom [arrows]pan [f]ollow [c]enter ")
        try:
            stdscr.addstr(rows - 1, 0, status[:cols - 1], curses.A_REVERSE)
        except curses.error:
            pass
        stdscr.refresh()
        time.sleep(0.1)


def main():
    locale.setlocale(locale.LC_ALL, '')
    if '--snapshot' in sys.argv:
        snapshot()
        return
    rclpy.init()
    node = MapTui()
    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        curses.wrapper(tui, node)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
