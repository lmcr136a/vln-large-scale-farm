#!/usr/bin/env python3
import sys
import tty
import termios
import select
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


def get_key(timeout=0.1):
    if select.select([sys.stdin], [], [], timeout)[0]:
        ch1 = sys.stdin.read(1)
        if ch1 == '\x1b':
            ch2 = sys.stdin.read(1)
            if ch2 == '[':
                ch3 = sys.stdin.read(1)
                if ch3 == 'A': return 'UP'
                if ch3 == 'B': return 'DOWN'
                if ch3 == 'C': return 'RIGHT'
                if ch3 == 'D': return 'LEFT'
        return ch1
    return ''


class Teleop(Node):
    def __init__(self, topic='/cmd_vel_raw', lin=0.2, ang=1.0):
        super().__init__('terminal_cmd_vel_teleop')
        self.pub = self.create_publisher(Twist, topic, 10)
        self.lin = float(lin)
        self.ang = float(ang)
        self.get_logger().info(f"Publishing Twist to: {topic}")
        self.get_logger().info(f"Linear speed: {self.lin} m/s, Angular speed: {self.ang} rad/s")

    def publish(self, lin_x=0.0, ang_z=0.0):
        msg = Twist()
        msg.linear.x = float(lin_x)
        msg.angular.z = float(ang_z)
        self.pub.publish(msg)


def main():
    try:
        user = input("INPUT ROBOT VEL(?): ").strip()
        base_lin = float(user) if user else 0.2
    except Exception:
        base_lin = 0.2
    base_ang = 1.0

    # Publish to /cmd_vel
    topic = '/cmd_vel'
    publish_hz = 20.0
    deadman_timeout = 0.25  # seconds: stop if no key input recently

    rclpy.init()
    node = Teleop(topic=topic, lin=base_lin, ang=base_ang)

    print("\nControls (continuous):")
    print("  ↑ forward   ↓ backward")
    print("  ← rotate L  → rotate R")
    print("  space: stop (and hold stopped)")
    print("  q / Ctrl+C: quit\n")

    old_attrs = termios.tcgetattr(sys.stdin)
    tty.setraw(sys.stdin.fileno())

    # Desired command (kept and published continuously)
    cmd_lin = 0.0
    cmd_ang = 0.0
    last_input_t = time.monotonic()

    dt = 1.0 / publish_hz

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.0)

            key = get_key(timeout=0.01)
            now = time.monotonic()

            if key:
                last_input_t = now

                if key == 'UP':
                    cmd_lin, cmd_ang = node.lin, 0.0
                elif key == 'DOWN':
                    cmd_lin, cmd_ang = -node.lin, 0.0
                elif key == 'LEFT':
                    cmd_lin, cmd_ang = 0.0, node.ang
                elif key == 'RIGHT':
                    cmd_lin, cmd_ang = 0.0, -node.ang
                elif key == ' ':
                    cmd_lin, cmd_ang = 0.0, 0.0
                elif key in ('q', 'Q', '\x03'):
                    break

            # if no key activity, stop
            if (now - last_input_t) > deadman_timeout:
                cmd_lin, cmd_ang = 0.0, 0.0

            node.publish(cmd_lin, cmd_ang)
            time.sleep(dt)

    finally:
        try:
            node.publish(0.0, 0.0)
        except Exception:
            pass
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_attrs)
        node.destroy_node()
        rclpy.shutdown()
        print("\nStopped. Exiting.")


if __name__ == "__main__":
    main()
