#!/usr/bin/env python3

import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rqt_gui_py.plugin import Plugin
from python_qt_binding.QtCore import Qt, QTimer
from python_qt_binding.QtWidgets import (
    QAbstractItemView,
    QCheckBox,
    QComboBox,
    QGridLayout,
    QHBoxLayout,
    QLabel,
    QMessageBox,
    QPushButton,
    QTabWidget,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from spinal_msgs.msg import (
    ApplicationCapabilities,
    ConfigFlashStatus,
    FlightParameterTable,
)
from spinal_msgs.srv import ManageConfigFlash, ManageFlightParameters
from std_srvs.srv import Trigger


def _ns_join(namespace, name):
    if not namespace:
        return "/" + name.lstrip("/")
    return f'{namespace.rstrip("/")}/{name.lstrip("/")}'


def _table():
    table = QTableWidget(0, 2)
    table.setHorizontalHeaderLabels(["parameter", "value"])
    table.setEditTriggers(QAbstractItemView.NoEditTriggers)
    table.setSelectionBehavior(QAbstractItemView.SelectRows)
    table.horizontalHeader().setStretchLastSection(True)
    return table


def _set_rows(table, rows):
    table.setRowCount(len(rows))
    for row, (key, value) in enumerate(rows):
        table.setItem(row, 0, QTableWidgetItem(str(key)))
        table.setItem(row, 1, QTableWidgetItem(str(value)))
    table.resizeColumnToContents(0)


def _flatten_message(message, prefix=""):
    rows = []
    for name in message.get_fields_and_field_types():
        value = getattr(message, name)
        path = f"{prefix}.{name}" if prefix else name
        if hasattr(value, "get_fields_and_field_types"):
            rows.extend(_flatten_message(value, path))
        elif isinstance(value, (list, tuple)):
            if value and hasattr(value[0], "get_fields_and_field_types"):
                for index, item in enumerate(value):
                    rows.extend(_flatten_message(item, f"{path}[{index}]"))
            else:
                rows.append((path, list(value)))
        else:
            rows.append((path, value))
    return rows


class ParameterDatabase(Plugin):
    CAPABILITY_NAMES = (
        (ApplicationCapabilities.CAP_IMU_ICM20948, "IMU ICM20948"),
        (ApplicationCapabilities.CAP_IMU_MPU9250, "IMU MPU9250"),
        (ApplicationCapabilities.CAP_BAROMETER, "Barometer"),
        (ApplicationCapabilities.CAP_GPS_UART3, "GPS on UART3"),
        (ApplicationCapabilities.CAP_CRSF_UART3, "CRSF on UART3"),
        (ApplicationCapabilities.CAP_DYNAMIXEL, "Dynamixel"),
        (ApplicationCapabilities.CAP_KONDO, "Kondo servo"),
        (ApplicationCapabilities.CAP_DSHOT, "DShot"),
        (ApplicationCapabilities.CAP_FLIGHT_CONTROL, "Flight control"),
        (ApplicationCapabilities.CAP_POSITION_CONTROL, "Position control"),
        (ApplicationCapabilities.CAP_CONFIG_FLASH_AB, "A/B Config Flash"),
        (ApplicationCapabilities.CAP_PWM, "PWM output"),
        (ApplicationCapabilities.CAP_ATTITUDE_ESTIMATION, "Attitude estimation"),
        (ApplicationCapabilities.CAP_HEIGHT_ESTIMATION, "Height estimation"),
        (ApplicationCapabilities.CAP_POSITION_ESTIMATION, "Position estimation"),
    )
    UART3_NAMES = {
        ConfigFlashStatus.UART3_DISABLED: "Disabled",
        ConfigFlashStatus.UART3_GPS: "GPS",
        ConfigFlashStatus.UART3_CRSF: "CRSF (receive only)",
    }
    IMU_NAMES = {
        ConfigFlashStatus.IMU_DISABLED: "Disabled",
        ConfigFlashStatus.IMU_MPU9250: "MPU9250",
        ConfigFlashStatus.IMU_ICM20948: "ICM20948",
    }
    SERVO_NAMES = {
        ConfigFlashStatus.SERVO_DISABLED: "Disabled",
        ConfigFlashStatus.SERVO_DYNAMIXEL: "Dynamixel",
        ConfigFlashStatus.SERVO_KONDO: "Kondo",
    }
    MOTOR_OUTPUT_NAMES = {
        ConfigFlashStatus.MOTOR_OUTPUT_DISABLED: "Disabled",
        ConfigFlashStatus.MOTOR_OUTPUT_PWM: "PWM",
        ConfigFlashStatus.MOTOR_OUTPUT_DSHOT: "DSHOT",
    }

    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("SpinalParameterDatabase")
        self._owns_rclpy = False
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy = True
        self.node = rclpy.create_node(f"spinal_parameter_database_rqt_{id(self)}")
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._capabilities = None
        self._config_status = None
        self._pending_futures = set()
        self._config_controls_dirty = False

        namespace = self._discover_namespace()
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.application_sub = self.node.create_subscription(
            ApplicationCapabilities,
            _ns_join(namespace, "fc/application_capabilities"),
            self._application_callback,
            qos,
        )
        self.config_sub = self.node.create_subscription(
            ConfigFlashStatus,
            _ns_join(namespace, "fc/config_flash/status"),
            self._config_callback,
            qos,
        )
        self.flight_sub = self.node.create_subscription(
            FlightParameterTable,
            _ns_join(namespace, "fc/flight_parameters/table"),
            self._flight_callback,
            qos,
        )
        self.config_client = self.node.create_client(
            ManageConfigFlash, _ns_join(namespace, "fc/config_flash")
        )
        self.flight_client = self.node.create_client(
            ManageFlightParameters, _ns_join(namespace, "fc/parameters")
        )
        self.reboot_client = self.node.create_client(
            Trigger, _ns_join(namespace, "fc/reboot")
        )

        self.widget = QWidget()
        self.widget.setWindowTitle("Spinal Flash & Parameters")
        root = QVBoxLayout(self.widget)
        self.connection_label = QLabel(
            f'namespace: {namespace or "/"} — waiting for FC'
        )
        root.addWidget(self.connection_label)
        tabs = QTabWidget()
        root.addWidget(tabs)
        self.application_table = _table()
        tabs.addTab(self.application_table, "Application Flash")
        tabs.addTab(self._build_config_tab(), "Config Flash (reboot)")
        tabs.addTab(self._build_flight_tab(), "Flight Parameters")
        context.add_widget(self.widget)

        self.timer = QTimer()
        self.timer.timeout.connect(self._spin_once)
        self.timer.start(10)

    def _build_config_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        warning = QLabel(
            "This configuration is stored in A/B Config Flash. A committed change "
            "takes effect only after a normal FC reboot."
        )
        warning.setWordWrap(True)
        layout.addWidget(warning)
        self.config_table = _table()
        layout.addWidget(self.config_table)
        controls = QGridLayout()
        controls.addWidget(QLabel("IMU driver"), 0, 0)
        self.imu_combo = QComboBox()
        controls.addWidget(self.imu_combo, 0, 1)
        controls.addWidget(QLabel("Barometer"), 1, 0)
        self.barometer_check = QCheckBox("Enabled")
        controls.addWidget(self.barometer_check, 1, 1)
        controls.addWidget(QLabel("UART3 driver"), 2, 0)
        self.uart3_combo = QComboBox()
        controls.addWidget(self.uart3_combo, 2, 1)
        controls.addWidget(QLabel("Servo driver"), 3, 0)
        self.servo_combo = QComboBox()
        controls.addWidget(self.servo_combo, 3, 1)
        controls.addWidget(QLabel("Attitude estimation"), 4, 0)
        self.attitude_estimation_check = QCheckBox("Enabled")
        controls.addWidget(self.attitude_estimation_check, 4, 1)
        controls.addWidget(QLabel("Height estimation"), 5, 0)
        self.height_estimation_check = QCheckBox("Enabled")
        controls.addWidget(self.height_estimation_check, 5, 1)
        controls.addWidget(QLabel("Position estimation"), 6, 0)
        self.position_estimation_check = QCheckBox("Enabled")
        controls.addWidget(self.position_estimation_check, 6, 1)
        controls.addWidget(QLabel("Flight control"), 7, 0)
        self.flight_control_check = QCheckBox("Enabled")
        controls.addWidget(self.flight_control_check, 7, 1)
        controls.addWidget(QLabel("Motor output"), 8, 0)
        self.motor_output_combo = QComboBox()
        controls.addWidget(self.motor_output_combo, 8, 1)
        for combo in (
            self.imu_combo,
            self.uart3_combo,
            self.servo_combo,
            self.motor_output_combo,
        ):
            combo.currentIndexChanged.connect(self._mark_config_controls_dirty)
        for checkbox in (
            self.barometer_check,
            self.attitude_estimation_check,
            self.height_estimation_check,
            self.position_estimation_check,
            self.flight_control_check,
        ):
            checkbox.stateChanged.connect(self._mark_config_controls_dirty)
        self.stage_button = QPushButton("Stage")
        self.commit_config_button = QPushButton("Commit Config Flash")
        self.reload_config_button = QPushButton("Discard staged / Reload")
        self.reboot_button = QPushButton("Reboot FC")
        self.apply_reboot_button = QPushButton("Apply Config Flash && Reboot")
        self.stage_button.clicked.connect(self._stage_config)
        self.commit_config_button.clicked.connect(
            lambda: self._call_config(ManageConfigFlash.Request.COMMIT)
        )
        self.reload_config_button.clicked.connect(
            lambda: self._call_config(ManageConfigFlash.Request.RELOAD)
        )
        self.reboot_button.clicked.connect(self._reboot)
        self.apply_reboot_button.clicked.connect(self._apply_and_reboot)
        controls.addWidget(self.stage_button, 9, 0)
        controls.addWidget(self.commit_config_button, 9, 1)
        controls.addWidget(self.reload_config_button, 10, 0)
        controls.addWidget(self.reboot_button, 10, 1)
        controls.addWidget(self.apply_reboot_button, 11, 0, 1, 2)
        layout.addLayout(controls)
        return widget

    def _build_flight_tab(self):
        widget = QWidget()
        layout = QVBoxLayout(widget)
        description = QLabel(
            "Runtime flight-control values are separate from reboot configuration. "
            "Commit and reload are rejected while armed."
        )
        description.setWordWrap(True)
        layout.addWidget(description)
        self.flight_table = _table()
        layout.addWidget(self.flight_table)
        buttons = QHBoxLayout()
        commit = QPushButton("Commit Flight Parameters")
        reload_button = QPushButton("Reload Flight Parameters")
        commit.clicked.connect(
            lambda: self._call_flight(ManageFlightParameters.Request.COMMIT)
        )
        reload_button.clicked.connect(
            lambda: self._call_flight(ManageFlightParameters.Request.RELOAD)
        )
        buttons.addWidget(commit)
        buttons.addWidget(reload_button)
        layout.addLayout(buttons)
        return widget

    def _discover_namespace(self):
        suffixes = (
            "/fc/application_capabilities",
            "/fc/config_flash/status",
            "/fc/flight_parameters/table",
        )
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            for topic, _types in self.node.get_topic_names_and_types():
                for suffix in suffixes:
                    if topic == suffix or topic.endswith(suffix):
                        return topic[: -len(suffix)]
            self.executor.spin_once(timeout_sec=0.05)
        return ""

    def _spin_once(self):
        try:
            self.executor.spin_once(timeout_sec=0.0)
        except Exception as error:
            self.connection_label.setText(f"ROS error: {error}")

    def _application_callback(self, msg):
        self._capabilities = msg
        board = (
            "STM32H7 v2"
            if msg.board == msg.BOARD_STM32H7_V2
            else "Simulation / unknown"
        )
        rows = [
            ("storage class", "Application Flash (read-only at runtime)"),
            ("board", board),
            ("capability mask", f"0x{msg.capability_mask:08x}"),
            ("max motor count", msg.max_motor_count),
            ("Config Flash schema", msg.config_schema_version),
            ("Flight Parameter schema", msg.flight_parameter_schema_version),
        ]
        rows.extend(
            (name, "compiled in" if msg.capability_mask & bit else "not available")
            for bit, name in self.CAPABILITY_NAMES
        )
        _set_rows(self.application_table, rows)
        self._rebuild_config_choices()
        self.connection_label.setText("FC parameter interface connected")

    def _config_callback(self, msg):
        self._config_status = msg
        slot = {-1: "none", 0: "A", 1: "B"}.get(msg.active_slot, "invalid")
        rows = [
            ("storage class", "Config Flash (reboot required)"),
            ("persistent storage", msg.persistent_storage),
            ("valid", msg.valid),
            ("dirty (not committed)", msg.dirty),
            ("reboot required", msg.reboot_required),
            ("active A/B slot", slot),
            (
                "running UART3 driver",
                self.UART3_NAMES.get(msg.active_uart3_driver, "invalid"),
            ),
            (
                "saved/pending UART3 driver",
                self.UART3_NAMES.get(msg.pending_uart3_driver, "invalid"),
            ),
            (
                "running IMU driver",
                self.IMU_NAMES.get(msg.active_imu_driver, "invalid"),
            ),
            (
                "saved/pending IMU driver",
                self.IMU_NAMES.get(msg.pending_imu_driver, "invalid"),
            ),
            ("running Barometer", msg.active_barometer_enabled),
            ("saved/pending Barometer", msg.pending_barometer_enabled),
            (
                "running Servo driver",
                self.SERVO_NAMES.get(msg.active_servo_driver, "invalid"),
            ),
            (
                "saved/pending Servo driver",
                self.SERVO_NAMES.get(msg.pending_servo_driver, "invalid"),
            ),
            ("running Attitude estimation", msg.active_attitude_estimation_enabled),
            (
                "saved/pending Attitude estimation",
                msg.pending_attitude_estimation_enabled,
            ),
            ("running Height estimation", msg.active_height_estimation_enabled),
            ("saved/pending Height estimation", msg.pending_height_estimation_enabled),
            ("running Position estimation", msg.active_position_estimation_enabled),
            (
                "saved/pending Position estimation",
                msg.pending_position_estimation_enabled,
            ),
            ("running Flight control", msg.active_flight_control_enabled),
            ("saved/pending Flight control", msg.pending_flight_control_enabled),
            (
                "running Motor output",
                self.MOTOR_OUTPUT_NAMES.get(msg.active_motor_output_driver, "invalid"),
            ),
            (
                "saved/pending Motor output",
                self.MOTOR_OUTPUT_NAMES.get(msg.pending_motor_output_driver, "invalid"),
            ),
            ("schema version", msg.schema_version),
            ("generation", msg.generation),
            ("CRC32", f"0x{msg.crc32:08x}"),
        ]
        _set_rows(self.config_table, rows)
        if not self._config_controls_dirty:
            self._set_combo(self.imu_combo, msg.pending_imu_driver)
            self._set_combo(self.uart3_combo, msg.pending_uart3_driver)
            self._set_combo(self.servo_combo, msg.pending_servo_driver)
            self._set_combo(self.motor_output_combo, msg.pending_motor_output_driver)
            for checkbox, value in (
                (self.barometer_check, msg.pending_barometer_enabled),
                (
                    self.attitude_estimation_check,
                    msg.pending_attitude_estimation_enabled,
                ),
                (self.height_estimation_check, msg.pending_height_estimation_enabled),
                (
                    self.position_estimation_check,
                    msg.pending_position_estimation_enabled,
                ),
                (self.flight_control_check, msg.pending_flight_control_enabled),
            ):
                checkbox.blockSignals(True)
                checkbox.setChecked(value)
                checkbox.blockSignals(False)
            self._config_controls_dirty = False
        self.reboot_button.setStyleSheet(
            "background: #d98c00;" if msg.reboot_required else ""
        )

    def _flight_callback(self, msg):
        rows = [("storage class", "Flight Parameters (runtime/staged)")]
        rows.extend(_flatten_message(msg))
        _set_rows(self.flight_table, rows)

    def _rebuild_config_choices(self):
        mask = self._capabilities.capability_mask if self._capabilities else 0
        self._replace_combo(
            self.imu_combo,
            [("Disabled", ConfigFlashStatus.IMU_DISABLED)]
            + (
                [("MPU9250", ConfigFlashStatus.IMU_MPU9250)]
                if mask & ApplicationCapabilities.CAP_IMU_MPU9250
                else []
            )
            + (
                [("ICM20948", ConfigFlashStatus.IMU_ICM20948)]
                if mask & ApplicationCapabilities.CAP_IMU_ICM20948
                else []
            ),
        )
        self._replace_combo(
            self.uart3_combo,
            [("Disabled", ConfigFlashStatus.UART3_DISABLED)]
            + (
                [("GPS", ConfigFlashStatus.UART3_GPS)]
                if mask & ApplicationCapabilities.CAP_GPS_UART3
                else []
            )
            + (
                [("CRSF (receive only)", ConfigFlashStatus.UART3_CRSF)]
                if mask & ApplicationCapabilities.CAP_CRSF_UART3
                else []
            ),
        )
        self._replace_combo(
            self.servo_combo,
            [("Disabled", ConfigFlashStatus.SERVO_DISABLED)]
            + (
                [("Dynamixel", ConfigFlashStatus.SERVO_DYNAMIXEL)]
                if mask & ApplicationCapabilities.CAP_DYNAMIXEL
                else []
            )
            + (
                [("Kondo", ConfigFlashStatus.SERVO_KONDO)]
                if mask & ApplicationCapabilities.CAP_KONDO
                else []
            ),
        )
        self._replace_combo(
            self.motor_output_combo,
            [("Disabled", ConfigFlashStatus.MOTOR_OUTPUT_DISABLED)]
            + (
                [("PWM", ConfigFlashStatus.MOTOR_OUTPUT_PWM)]
                if mask & ApplicationCapabilities.CAP_PWM
                else []
            )
            + (
                [("DSHOT", ConfigFlashStatus.MOTOR_OUTPUT_DSHOT)]
                if mask & ApplicationCapabilities.CAP_DSHOT
                else []
            ),
        )
        for checkbox, capability in (
            (self.barometer_check, ApplicationCapabilities.CAP_BAROMETER),
            (
                self.attitude_estimation_check,
                ApplicationCapabilities.CAP_ATTITUDE_ESTIMATION,
            ),
            (
                self.height_estimation_check,
                ApplicationCapabilities.CAP_HEIGHT_ESTIMATION,
            ),
            (
                self.position_estimation_check,
                ApplicationCapabilities.CAP_POSITION_ESTIMATION,
            ),
            (self.flight_control_check, ApplicationCapabilities.CAP_FLIGHT_CONTROL),
        ):
            checkbox.setEnabled(bool(mask & capability))

    def _replace_combo(self, combo, items):
        selected = combo.currentData()
        combo.blockSignals(True)
        combo.clear()
        for label, value in items:
            combo.addItem(label, value)
        index = combo.findData(selected)
        if index >= 0:
            combo.setCurrentIndex(index)
        combo.blockSignals(False)

    @staticmethod
    def _set_combo(combo, value):
        combo.blockSignals(True)
        index = combo.findData(value)
        if index >= 0:
            combo.setCurrentIndex(index)
        combo.blockSignals(False)

    def _mark_config_controls_dirty(self, *_args):
        self._config_controls_dirty = True

    def _track_future(self, future, label):
        self._pending_futures.add(future)

        def done(completed):
            self._pending_futures.discard(completed)
            try:
                response = completed.result()
                success = bool(response.success)
                result = getattr(response, "result", "")
                message = getattr(response, "message", "")
                self.connection_label.setText(
                    f'{label}: {"OK" if success else "rejected"} {message or result}'
                )
                if hasattr(response, "status"):
                    self._config_controls_dirty = False
                    self._config_callback(response.status)
            except Exception as error:
                self.connection_label.setText(f"{label}: service error: {error}")

        future.add_done_callback(done)

    def _make_config_request(self, command):
        request = ManageConfigFlash.Request()
        request.command = command
        request.imu_driver = int(self.imu_combo.currentData() or 0)
        request.barometer_enabled = self.barometer_check.isChecked()
        request.uart3_driver = int(self.uart3_combo.currentData() or 0)
        request.servo_driver = int(self.servo_combo.currentData() or 0)
        request.attitude_estimation_enabled = self.attitude_estimation_check.isChecked()
        request.height_estimation_enabled = self.height_estimation_check.isChecked()
        request.position_estimation_enabled = self.position_estimation_check.isChecked()
        request.flight_control_enabled = self.flight_control_check.isChecked()
        request.motor_output_driver = int(self.motor_output_combo.currentData() or 0)
        return request

    def _config_validation_error(self):
        imu_enabled = self.imu_combo.currentData() != ConfigFlashStatus.IMU_DISABLED
        attitude_enabled = self.attitude_estimation_check.isChecked()
        if attitude_enabled and not imu_enabled:
            return "Attitude estimation requires an enabled IMU driver."
        if self.height_estimation_check.isChecked() and not (
            imu_enabled and self.barometer_check.isChecked() and attitude_enabled
        ):
            return "Height estimation requires IMU, Barometer, and Attitude estimation."
        if self.position_estimation_check.isChecked() and not (
            imu_enabled and attitude_enabled
        ):
            return "Position estimation requires IMU and Attitude estimation."
        if self.flight_control_check.isChecked() and (
            not attitude_enabled
            or self.motor_output_combo.currentData()
            == ConfigFlashStatus.MOTOR_OUTPUT_DISABLED
        ):
            return (
                "Flight control requires Attitude estimation and a motor-output driver."
            )
        return ""

    def _call_config(self, command):
        if not self.config_client.service_is_ready():
            self.connection_label.setText("Config Flash service is not available")
            return
        request = self._make_config_request(command)
        self._track_future(self.config_client.call_async(request), "Config Flash")

    def _stage_config(self):
        error = self._config_validation_error()
        if error:
            self.connection_label.setText(f"Config Flash: {error}")
            QMessageBox.warning(self.widget, "Invalid Config Flash selection", error)
            return
        self._call_config(ManageConfigFlash.Request.STAGE)

    def _call_flight(self, command):
        if not self.flight_client.service_is_ready():
            self.connection_label.setText("Flight Parameter service is not available")
            return
        request = ManageFlightParameters.Request()
        request.command = command
        self._track_future(self.flight_client.call_async(request), "Flight Parameters")

    def _reboot(self):
        answer = QMessageBox.question(
            self.widget,
            "Reboot FC",
            "Reboot the disarmed FC now? Motor outputs will be stopped first.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        if not self.reboot_client.service_is_ready():
            self.connection_label.setText("FC reboot service is not available")
            return
        self._track_future(
            self.reboot_client.call_async(Trigger.Request()), "FC reboot"
        )

    def _apply_and_reboot(self):
        error = self._config_validation_error()
        if error:
            self.connection_label.setText(f"Config Flash: {error}")
            QMessageBox.warning(self.widget, "Invalid Config Flash selection", error)
            return
        answer = QMessageBox.question(
            self.widget,
            "Apply Config Flash and reboot",
            "Stage and commit all displayed boot configuration, then reboot the FC?\n"
            "The operation is rejected while armed.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if answer != QMessageBox.Yes:
            return
        if not (
            self.config_client.service_is_ready()
            and self.reboot_client.service_is_ready()
        ):
            self.connection_label.setText(
                "Config Flash or FC reboot service is not available"
            )
            return

        stage = self._make_config_request(ManageConfigFlash.Request.STAGE)
        stage_future = self.config_client.call_async(stage)
        self._pending_futures.add(stage_future)

        def stage_done(completed):
            self._pending_futures.discard(completed)
            try:
                response = completed.result()
                self._config_controls_dirty = False
                self._config_callback(response.status)
                if not response.success:
                    self.connection_label.setText(
                        f"Automatic apply: stage rejected ({response.result})"
                    )
                    return
                commit = ManageConfigFlash.Request()
                commit.command = ManageConfigFlash.Request.COMMIT
                commit_future = self.config_client.call_async(commit)
                self._pending_futures.add(commit_future)
                commit_future.add_done_callback(commit_done)
            except Exception as error:
                self.connection_label.setText(f"Automatic apply: stage failed: {error}")

        def commit_done(completed):
            self._pending_futures.discard(completed)
            try:
                response = completed.result()
                self._config_controls_dirty = False
                self._config_callback(response.status)
                if not response.success:
                    self.connection_label.setText(
                        f"Automatic apply: commit failed ({response.result})"
                    )
                    return
                reboot_future = self.reboot_client.call_async(Trigger.Request())
                self._track_future(reboot_future, "Automatic apply/reboot")
            except Exception as error:
                self.connection_label.setText(
                    f"Automatic apply: commit failed: {error}"
                )

        stage_future.add_done_callback(stage_done)

    def shutdown_plugin(self):
        self.timer.stop()
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        if self._owns_rclpy and rclpy.ok():
            rclpy.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        pass
