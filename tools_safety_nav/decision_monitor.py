#!/usr/bin/env python3
from __future__ import annotations

import json
from datetime import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class DecisionMonitor(Node):
    def __init__(self) -> None:
        super().__init__("decision_monitor")
        self.create_subscription(String, "/agro_nav/decision", self._decision_cb, 20)
        self.get_logger().info("Decision monitor ready — listening on /agro_nav/decision")

    def _decision_cb(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Non-JSON decision payload: {msg.data[:120]}")
            return

        now = datetime.now().strftime("%H:%M:%S")
        seen = payload.get("primary_semantic_label") or payload.get("dominant_label") or "clear"
        if payload.get("primary_semantic_distance") is not None:
            seen = f"{seen}@{float(payload['primary_semantic_distance']):.1f}m"

        compliance = payload.get("primary_semantic_compliance") or payload.get("compliance") or "none"
        action = payload.get("action") or "GO"
        layer = payload.get("decision_layer") or "semantic"
        sem_status = payload.get("semantic_status") or "offline"
        sem_count = int(payload.get("semantic_detection_count") or 0)
        reaction_ms = payload.get("reaction_latency_ms")
        reaction_str = f"{float(reaction_ms):.0f}ms" if reaction_ms is not None else "--"
        sem_source = payload.get("sem_source") or "none"
        reason = payload.get("reason") or ""

        print(
            f"[{now}] seen={seen:<18} compliance={compliance:<14} "
            f"action={action:<12} layer={layer:<10} sem={sem_status}:{sem_count:<4} source={sem_source:<13} "
            f"reaction={reaction_str:<6} reason={reason}",
            flush=True,
        )


def main() -> None:
    rclpy.init()
    node = DecisionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
