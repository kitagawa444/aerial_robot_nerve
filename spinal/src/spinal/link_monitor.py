#!/usr/bin/env python3

import argparse
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rqt_gui_py.plugin import Plugin

from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtGui import QColor
from python_qt_binding.QtWidgets import (
    QAbstractItemView,
    QHeaderView,
    QLabel,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from spinal_msgs.msg import CommunicationLinkStatus, CommunicationStatus

DISCOVERY_TIMEOUT_SEC = 2.0
MESSAGE_STALE_SEC = 2.0
UNKNOWN_AGE = 0xFFFFFFFF


def _ns_join(namespace, name):
    if not namespace:
        return "/" + name.lstrip("/")
    return f'{namespace.rstrip("/")}/{name.lstrip("/")}'


class LinkMonitor(Plugin):
    STATE_NAMES = {
        CommunicationLinkStatus.STATE_UNKNOWN: "UNKNOWN",
        CommunicationLinkStatus.STATE_DISCONNECTED: "DISCONNECTED",
        CommunicationLinkStatus.STATE_CONNECTING: "CONNECTING",
        CommunicationLinkStatus.STATE_CONNECTED: "CONNECTED",
        CommunicationLinkStatus.STATE_STALE: "STALE",
        CommunicationLinkStatus.STATE_DISABLED: "DISABLED",
        CommunicationLinkStatus.STATE_ERROR: "ERROR",
    }
    STATE_COLORS = {
        CommunicationLinkStatus.STATE_UNKNOWN: QColor("#9e9e9e"),
        CommunicationLinkStatus.STATE_DISCONNECTED: QColor("#ef5350"),
        CommunicationLinkStatus.STATE_CONNECTING: QColor("#ffca28"),
        CommunicationLinkStatus.STATE_CONNECTED: QColor("#66bb6a"),
        CommunicationLinkStatus.STATE_STALE: QColor("#ffa726"),
        CommunicationLinkStatus.STATE_DISABLED: QColor("#bdbdbd"),
        CommunicationLinkStatus.STATE_ERROR: QColor("#d32f2f"),
    }

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("SpinalLinkMonitor")

        parser = argparse.ArgumentParser()
        parser.add_argument("--robot-ns", default="")
        args, _unknown = parser.parse_known_args(context.argv())

        self._rclpy_initialized_here = False
        if not rclpy.ok():
            rclpy.init(args=None)
            self._rclpy_initialized_here = True
        self.node = rclpy.create_node(f"spinal_link_monitor_rqt_{id(self)}")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

        robot_ns = args.robot_ns or self._discover_robot_ns()
        topic = _ns_join(robot_ns, "fc/communication_status")
        self.subscription = self.node.create_subscription(
            CommunicationStatus, topic, self._status_callback, 10
        )

        self._widget = QWidget()
        self._widget.setWindowTitle("Spinal Link Monitor")
        layout = QVBoxLayout(self._widget)
        self.summary_label = QLabel(f"Waiting for {topic}")
        layout.addWidget(self.summary_label)
        self.table = QTableWidget(5, 7)
        self.table.setHorizontalHeaderLabels(
            ["Link", "State", "Last RX", "Rate", "Quality", "Errors", "Details"]
        )
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(6, QHeaderView.Stretch)
        layout.addWidget(self.table)
        context.add_widget(self._widget)

        self.last_message_time = 0.0
        self.last_status = None
        self.spin_timer = QTimer()
        self.spin_timer.timeout.connect(self._spin_once)
        self.spin_timer.start(20)
        self.stale_timer = QTimer()
        self.stale_timer.timeout.connect(self._update_staleness)
        self.stale_timer.start(250)

    def _discover_robot_ns(self):
        deadline = time.monotonic() + DISCOVERY_TIMEOUT_SEC
        while time.monotonic() <= deadline:
            try:
                candidates = [
                    name
                    for name, types in self.node.get_topic_names_and_types()
                    if (
                        name == "/fc/communication_status"
                        or name.endswith("/fc/communication_status")
                    )
                    and "spinal_msgs/msg/CommunicationStatus" in types
                ]
                if candidates:
                    return candidates[0].rsplit("/fc/communication_status", 1)[0]
            except Exception:
                pass
            self.executor.spin_once(timeout_sec=0.05)
        return ""

    def _spin_once(self):
        try:
            self.executor.spin_once(timeout_sec=0.0)
        except Exception as error:
            self.summary_label.setText(f"ROS error: {error}")

    def _status_callback(self, message):
        self.last_status = message
        self.last_message_time = time.monotonic()
        links = [message.micro_usb, message.ros, message.wifi, message.rc]
        for row, link in enumerate(links):
            self._set_link_row(row, link)
        self._set_ack_row(4, message)
        problem_count = sum(
            link.state
            not in (
                CommunicationLinkStatus.STATE_CONNECTED,
                CommunicationLinkStatus.STATE_DISABLED,
            )
            for link in links
        )
        if message.ack.state != CommunicationLinkStatus.STATE_CONNECTED:
            problem_count += 1
        self.summary_label.setText(
            "All links healthy"
            if problem_count == 0
            else f"{problem_count} link item(s) need attention"
        )

    def _set_item(self, row, column, text, color=None):
        item = QTableWidgetItem(text)
        if color is not None:
            item.setBackground(color)
        self.table.setItem(row, column, item)

    @staticmethod
    def _age_text(age_ms):
        if age_ms == UNKNOWN_AGE:
            return "—"
        if age_ms < 1000:
            return f"{age_ms} ms"
        return f"{age_ms / 1000.0:.1f} s"

    def _set_link_row(self, row, link):
        color = self.STATE_COLORS.get(link.state, self.STATE_COLORS[0])
        state = self.STATE_NAMES.get(link.state, f"UNKNOWN({link.state})")
        rate = f"{link.receive_rate_hz:.1f} / {link.transmit_rate_hz:.1f} Hz"
        quality = "—"
        if link.name.startswith("ELRS"):
            quality = f"{link.quality_percent}% / {link.rssi_dbm} dBm"
        self._set_item(row, 0, link.name)
        self._set_item(row, 1, state, color)
        self._set_item(row, 2, self._age_text(link.last_receive_age_ms))
        self._set_item(row, 3, rate)
        self._set_item(row, 4, quality)
        self._set_item(row, 5, str(link.error_count))
        detail = link.detail + ("; DATA STALE" if link.data_stale else "")
        self._set_item(row, 6, detail)

    def _set_ack_row(self, row, message):
        ack = message.ack
        color = self.STATE_COLORS.get(ack.state, self.STATE_COLORS[0])
        state = self.STATE_NAMES.get(ack.state, f"UNKNOWN({ack.state})")
        self._set_item(row, 0, "ACK delivery")
        self._set_item(row, 1, state, color)
        pending_age = (
            self._age_text(ack.oldest_pending_age_ms) if ack.pending_count > 0 else "—"
        )
        self._set_item(row, 2, pending_age)
        self._set_item(row, 3, f"RTT {ack.last_rtt_ms} ms")
        self._set_item(row, 4, f"{ack.pending_count} pending")
        errors = ack.timeout_count + ack.rejected_count
        self._set_item(row, 5, str(errors))
        self._set_item(
            row,
            6,
            f"{ack.detail}; retries={ack.retry_count}, "
            f"duplicates={ack.duplicate_count}, "
            f"last={ack.last_message_id}/{ack.last_request_id}",
        )

    def _update_staleness(self):
        if self.last_message_time == 0.0:
            return
        age = time.monotonic() - self.last_message_time
        if age <= MESSAGE_STALE_SEC:
            return
        self.summary_label.setText(
            f"CommunicationStatus is stale ({age:.1f} s); ROS path may be disconnected"
        )
        stale_color = self.STATE_COLORS[CommunicationLinkStatus.STATE_UNKNOWN]
        for row in range(self.table.rowCount()):
            self._set_item(row, 1, "UNKNOWN", stale_color)

    def shutdown_plugin(self):
        self.spin_timer.stop()
        self.stale_timer.stop()
        try:
            self.executor.remove_node(self.node)
            self.node.destroy_node()
        except Exception:
            pass
        if self._rclpy_initialized_here and rclpy.ok():
            rclpy.shutdown()

    def save_settings(self, _plugin_settings, _instance_settings):
        pass

    def restore_settings(self, _plugin_settings, _instance_settings):
        pass
