#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026, DRAGON Laboratory, The University of Tokyo

"""End-to-end Gazebo flight check driven only by simulated CRSF frames."""

import math
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Joy
from spinal_msgs.msg import (
    CommunicationAckStatus,
    CommunicationLinkStatus,
    CommunicationStatus,
    FlightStatus,
)
from std_srvs.srv import SetBool, Trigger


class CrsfGazeboTest(Node):
    def __init__(self):
        super().__init__("crsf_gazebo_test")
        self._status = None
        self._position = None
        self._initial_position = None
        self._maximum_height = -math.inf
        self._communication_status = None
        self._diagnostic_names = set()
        self.create_subscription(
            FlightStatus, "fc/flight_status", self._status_callback, 10
        )
        self.create_subscription(
            CommunicationStatus,
            "fc/communication_status",
            self._communication_callback,
            10,
        )
        self.create_subscription(
            DiagnosticArray, "/diagnostics", self._diagnostics_callback, 10
        )
        self.create_subscription(Odometry, "ground_truth", self._odometry_callback, 10)
        self._joy_publisher = self.create_publisher(Joy, "rc/sim/joy", 10)
        self._service_clients = {
            name: self.create_client(Trigger, f"rc/sim/{name}")
            for name in ("reset", "arm", "takeoff", "land")
        }
        self._link_client = self.create_client(SetBool, "rc/sim/link")

    def _status_callback(self, message):
        self._status = message

    def _odometry_callback(self, message):
        position = message.pose.pose.position
        self._position = (position.x, position.y, position.z)
        if self._initial_position is None:
            self._initial_position = self._position
        self._maximum_height = max(self._maximum_height, position.z)

    def _communication_callback(self, message):
        self._communication_status = message

    def _diagnostics_callback(self, message):
        self._diagnostic_names.update(status.name for status in message.status)

    def _spin_until(self, predicate, timeout, description):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if predicate():
                self.get_logger().info(f"PASS: {description}")
                return
        status = self._status
        details = "no FC status"
        if status is not None:
            details = (
                f"arming={status.arming_state}, phase={status.flight_phase}, mode={status.control_mode}, "
                f"authority={status.authority}, rc={status.rc_connected}, command={status.last_command}, "
                f"result={status.last_command_result}, ready={status.ready_to_arm}"
            )
        raise RuntimeError(f"timeout waiting for {description}: {details}")

    def _spin_until_stable(self, predicate, stable_duration, timeout, description):
        deadline = time.monotonic() + timeout
        stable_since = None
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            now = time.monotonic()
            if predicate():
                if stable_since is None:
                    stable_since = now
                elif now - stable_since >= stable_duration:
                    self.get_logger().info(f"PASS: {description}")
                    return
            else:
                stable_since = None
        raise RuntimeError(f"timeout waiting for stable {description}")

    def _call(self, name):
        client = self._service_clients[name]
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"service rc/sim/{name} is unavailable")
        future = client.call_async(Trigger.Request())
        self._spin_until(lambda: future.done(), 5.0, f"rc/sim/{name} response")
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(
                f"rc/sim/{name} failed: {getattr(response, 'message', 'no response')}"
            )

    def _set_link(self, enabled):
        if not self._link_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError("service rc/sim/link is unavailable")
        request = SetBool.Request()
        request.data = enabled
        future = self._link_client.call_async(request)
        self._spin_until(lambda: future.done(), 5.0, f"rc/sim/link={enabled} response")
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError(f"rc/sim/link={enabled} failed")

    def _publish_joy_for(self, axes, duration):
        message = Joy()
        message.axes = [float(value) for value in axes]
        deadline = time.monotonic() + duration
        while rclpy.ok() and time.monotonic() < deadline:
            message.header.stamp = self.get_clock().now().to_msg()
            self._joy_publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.05)

    def run(self):
        self._spin_until(
            lambda: self._status is not None and self._position is not None,
            30.0,
            "Gazebo FC telemetry",
        )
        self._call("reset")
        self._spin_until(lambda: self._status.rc_connected, 5.0, "CRSF link connection")
        self._spin_until_stable(
            lambda: self._status.ready_to_arm, 1.5, 30.0, "FC ready-to-arm"
        )
        self._spin_until(
            lambda: self._communication_status is not None
            and self._communication_status.micro_usb.state
            == CommunicationLinkStatus.STATE_DISABLED
            and self._communication_status.ros.state
            == CommunicationLinkStatus.STATE_CONNECTED
            and self._communication_status.rc.state
            == CommunicationLinkStatus.STATE_CONNECTED
            and self._communication_status.rc.quality_percent == 100
            and self._communication_status.ack.state
            == CommunicationAckStatus.STATE_DISABLED,
            5.0,
            "communication status for Gazebo transport and CRSF",
        )
        self._spin_until(
            lambda: {
                "spinal_link/micro_usb",
                "spinal_link/ros",
                "spinal_link/wifi",
                "spinal_link/rc",
                "spinal_link/ack",
            }.issubset(self._diagnostic_names),
            5.0,
            "communication diagnostics",
        )

        self._call("arm")
        self._spin_until(
            lambda: self._status.arming_state == FlightStatus.ARMING_ARMED,
            8.0,
            "ARM through raw CRSF",
        )

        self._call("takeoff")
        self._spin_until(
            lambda: self._status.flight_phase == FlightStatus.PHASE_TAKING_OFF,
            8.0,
            "TAKEOFF accepted through raw CRSF",
        )
        self._spin_until(
            lambda: self._status.flight_phase == FlightStatus.PHASE_AIRBORNE,
            25.0,
            "FC takeoff completion",
        )

        before_motion = self._position
        self._publish_joy_for([0.0, 0.30, 0.0, 0.20], 2.0)
        self._publish_joy_for([0.0, 0.0, 0.0, 0.0], 0.5)
        planar_motion = math.hypot(
            self._position[0] - before_motion[0], self._position[1] - before_motion[1]
        )
        if planar_motion < 0.05:
            raise RuntimeError(
                f"CRSF stick input did not move the aircraft sufficiently: {planar_motion:.3f} m"
            )
        self.get_logger().info(
            f"PASS: CRSF forward/yaw stick motion ({planar_motion:.3f} m)"
        )

        self._call("land")
        self._spin_until(
            lambda: self._status.flight_phase == FlightStatus.PHASE_LANDING,
            8.0,
            "LAND accepted through raw CRSF",
        )
        self._spin_until(
            lambda: self._status.flight_phase == FlightStatus.PHASE_LANDED,
            30.0,
            "FC landing completion",
        )
        self._spin_until(
            lambda: self._status.arming_state == FlightStatus.ARMING_DISARMED,
            8.0,
            "FC automatic disarm after landing",
        )

        self._set_link(False)
        self._spin_until(
            lambda: self._status is not None
            and not self._status.rc_connected
            and self._communication_status is not None
            and self._communication_status.rc.state
            == CommunicationLinkStatus.STATE_DISCONNECTED,
            5.0,
            "CRSF link loss in communication status",
        )
        self._set_link(True)
        self._spin_until(
            lambda: self._status is not None
            and self._status.rc_connected
            and self._communication_status is not None
            and self._communication_status.rc.state
            == CommunicationLinkStatus.STATE_CONNECTED,
            5.0,
            "CRSF link recovery in communication status",
        )

        altitude_gain = self._maximum_height - self._initial_position[2]
        if altitude_gain < 0.30:
            raise RuntimeError(
                f"aircraft did not take off sufficiently: altitude gain={altitude_gain:.3f} m"
            )
        self.get_logger().info(f"PASS: Gazebo altitude gain ({altitude_gain:.3f} m)")


def main():
    rclpy.init()
    node = CrsfGazeboTest()
    exit_code = 0
    try:
        node.run()
        node.get_logger().info("CRSF Gazebo end-to-end test PASSED")
    except Exception as error:  # pylint: disable=broad-except
        node.get_logger().error(f"CRSF Gazebo end-to-end test FAILED: {error}")
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
