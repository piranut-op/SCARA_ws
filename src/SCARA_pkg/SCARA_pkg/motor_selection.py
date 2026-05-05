import rclpy
from rclpy.node import Node
from PyQt5.QtWidgets import QApplication, QWidget, QPushButton, QVBoxLayout, QComboBox, QLabel
from PyQt5 import QtCore
import can
import subprocess
import struct
import threading
import time

class ODriveCANNode(Node):
    def __init__(self, can_id):
        super().__init__('odrive_can_node')
        self.can_id = can_id  # Use the selected CAN ID
        self.encoder_value = 0.0  # Store the latest encoder position
        self.setup_can_interface()
        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)
        # Start a dedicated thread to listen for incoming CAN messages
        self.listener_thread = threading.Thread(target=self.listen_can_messages,     daemon=True)
        self.listener_thread.start()
    def setup_can_interface(self):
        """Ensures the CAN interface (can0) is up."""
        result = subprocess.run(["ip", "link", "show", "can0"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if b"state UP" in result.stdout:
            self.get_logger().info("CAN interface can0 is already up.")
        else:
            self.get_logger().info("Bringing up CAN interface can0.")
            result = subprocess.run(["sudo", "ip", "link", "set", "can0", "up", "type", "can", "bitrate", "1000000"])
            if result.returncode == 0:
                self.get_logger().info("CAN interface can0 is up.")
            else:
                self.get_logger().error("Failed to bring up CAN interface can0.")

    def send_can_message(self, arbitration_id, data):
        """Sends a CAN message with the given arbitration ID and data bytes."""
        message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
        try:
            self.bus.send(message)
            self.get_logger().info(f"Sent message ID {hex(arbitration_id)}: {data}")
        except can.CanError as e:
            self.get_logger().error(f"Failed to send message: {e}")

    def get_encoder_value(self):
        """Return the latest encoder value that was received."""
        return self.encoder_value

    def listen_can_messages(self):
        """Continuously listen for incoming CAN messages and process them."""
        while rclpy.ok():
            msg = self.bus.recv(timeout=1.0)
            if msg is not None:
                self.handle_can_message(msg)

    def handle_can_message(self, msg):
        """Handles incoming CAN messages and extracts encoder data."""
        # self.get_logger().info(f"🛠 Received CAN Message: ID={hex(msg.arbitration_id)}, Data={msg.data}")

        # Compute the expected arbitration ID using the current node's CAN ID
        expected_arbitration_id = (self.can_id << 5) | 0x009  # e.g., for CAN ID 1, 0x029

        if msg.arbitration_id == expected_arbitration_id:
            if len(msg.data) == 8:
                pos_estimate, vel_estimate = struct.unpack('<ff', msg.data)  # Little-endian decoding
                # self.get_logger().info(f"✅ Encoder Position: {pos_estimate:.4f} rev, Velocity: {vel_estimate:.4f} rev/s")
                # Instead of shutting down, update the encoder value
                self.encoder_value = pos_estimate
            else:
                self.get_logger().error(f"❌ Unexpected data length: {len(msg.data)}")
        else:
            self.get_logger().debug(f"ℹ️ Ignored message with ID {hex(msg.arbitration_id)}")

    def full_calibration_sequences(self):
        """Sends the full calibration sequence command to the ODrive for the selected CAN ID."""
        axis_id = self.can_id  # Use the selected CAN ID
        command_id = 0x007  # Command ID for calibration
        arbitration_id = (axis_id << 5) | command_id  # Calculate arbitration ID
        data = [0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]  # Full calibration sequence data
        self.send_can_message(arbitration_id, data)
        self.get_logger().info(f"Full calibration sequence command sent for CAN ID {axis_id}.")

    def set_torque_control_mode(self):
        """Switches the ODrive to torque control mode for the selected CAN ID."""
        axis_id = self.can_id  # Use the selected CAN ID
        command_id = 0x00B  # Command ID to set control mode
        arbitration_id = (axis_id << 5) | command_id  # Calculate arbitration ID
        data = [0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]  # Torque control mode data
        self.send_can_message(arbitration_id, data)
        self.get_logger().info(f"Torque control mode command sent for CAN ID {axis_id}.")

    def set_current_position_to_zero_odrive(self):
        """Sets the current position of the motor to zero for the selected CAN ID."""
        axis_id = self.can_id  # Use the selected CAN ID
        command_id = 0x19  # Command ID to set absolute position
        arbitration_id = (axis_id << 5) | command_id  # Calculate arbitration ID
        data = struct.pack('<f', 0.0)  # Position set to 0.0 (zero position)
        self.send_can_message(arbitration_id, data)
        self.get_logger().info(f"Set current position to zero for CAN ID {axis_id}.")

    def set_closed_loop_control(self):
        """Sets the motor to closed-loop control mode for the selected CAN ID."""
        axis_id = self.can_id  # Use the selected CAN ID
        command_id = 0x007  # Command ID for closed-loop control
        arbitration_id = (axis_id << 5) | command_id  # Calculate arbitration ID
        data = [0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]  # Closed-loop control mode data
        self.send_can_message(arbitration_id, data)
        self.get_logger().info(f"Set Closed-Loop Control command sent for CAN ID {axis_id}.")

    def set_position_control_mode(self):
        """Sets the motor to position control mode for the selected CAN ID."""
        axis_id = self.can_id  # Use the selected CAN ID
        command_id = 0x00B  # Command ID to set control mode
        arbitration_id = (axis_id << 5) | command_id  # Calculate arbitration ID
        data = [0x03, 0x00, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00]  # Position control mode data (AXIS_STATE_POSITION_CONTROL = 3)
        self.send_can_message(arbitration_id, data)
        self.get_logger().info(f"Set Position Control Mode command sent for CAN ID {axis_id}.")



class SERVO42CANNode(Node):
    def __init__(self, can_id):
        super().__init__('stepper_can_node')
        self.can_id = can_id  # Use the selected CAN ID
        self.setup_can_interface()
        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=250000)

    def setup_can_interface(self):
        """Ensures the CAN interface (can0) is up."""
        result = subprocess.run(["ip", "link", "show", "can0"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        if b"state UP" in result.stdout:
            self.get_logger().info("CAN interface can0 is already up.")
        else:
            self.get_logger().info("Bringing up CAN interface can0.")
            result = subprocess.run(["sudo", "ip", "link", "set", "can0", "up", "type", "can", "bitrate", "250000"])
            if result.returncode == 0:
                self.get_logger().info("CAN interface can0 is up.")
            else:
                self.get_logger().error("Failed to bring up CAN interface can0.")

    def send_can_message(self, arbitration_id, data):
        """Sends a CAN message with the given arbitration ID and data bytes."""
        message = can.Message(arbitration_id=arbitration_id, data=data, is_extended_id=False)
        try:
            self.bus.send(message)
            self.get_logger().info(f"Sent message ID {hex(arbitration_id)}: {data}")
        except can.CanError as e:
            self.get_logger().error(f"Failed to send message: {e}")

    def calibrate_motor_servo(self):
        """
        Command 0x80 to calibrate, typically needed in closed-loop or vFOC modes.
        """
        code = 0x80
        data1 = 0x00
        chksum = (self.can_id + code + data1) & 0xFF
        data = [code, data1, chksum]
        self.send_can_message(self.can_id, data)
        self.get_logger().info(f"Sent calibration command to motor with CAN ID {self.can_id}.")

    def set_mode(self, mode):
        """
        Set the mode of the servo motor. Mode 3 corresponds to SR_OPEN.
        """
        code = 0x82
        checksum = (self.can_id + code + mode) & 0xFF
        data = [code, mode, checksum]
        self.send_can_message(self.can_id, data)
        self.get_logger().info(f"Sent mode change command to motor with CAN ID {self.can_id}, mode: {mode}")

    def set_axis_to_zero(self):
        """
        Sends the 'GoHome' command (0x92) to set the current axis position to zero
        without moving the motor.
        """
        code = 0x92
        data = [code]
        checksum = (self.can_id + code) & 0xFF
        data.append(checksum)
        self.send_can_message(self.can_id, data)
        self.get_logger().info(f"Sent GoHome (set axis to zero) command to motor with CAN ID {self.can_id}.")


class GUI(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle('Motor Control GUI')
        self.setGeometry(100, 100, 400, 400)

        # Initialize the default motor type and CAN ID
        self.motor_type = 'ODrive'
        self.can_id = 0
        self.ros2_node = ODriveCANNode(self.can_id)  # Default to ODrive

        # Main layout container
        self.main_layout = QVBoxLayout()

        # Motor Type Selection Dropdown
        self.motor_type_label = QLabel("Select Motor Type:", self)
        self.main_layout.addWidget(self.motor_type_label)
        self.motor_type_combo = QComboBox(self)
        self.motor_type_combo.addItems(["ODrive", "SERVO42"])  # Add both motor types
        self.motor_type_combo.currentIndexChanged.connect(self.update_motor_type)
        self.main_layout.addWidget(self.motor_type_combo)

        # CAN ID Selection Dropdown
        self.can_id_label = QLabel("Select CAN ID:", self)
        self.main_layout.addWidget(self.can_id_label)
        self.can_id_combo = QComboBox(self)
        self.can_id_combo.addItems([str(i) for i in range(5)])  # CAN IDs from 0 to 4
        self.can_id_combo.currentIndexChanged.connect(self.update_can_id)
        self.main_layout.addWidget(self.can_id_combo)

        # Encoder Value Display for ODrive
        self.encoder_value_label = QLabel("Encoder Value: 0.0000", self)
        self.encoder_value_label.setAlignment(QtCore.Qt.AlignCenter)
        self.main_layout.addWidget(self.encoder_value_label)

        # Placeholder for motor commands
        self.command_layout = QVBoxLayout()

        # Start with ODrive layout
        self.update_motor_layout()

        self.setLayout(self.main_layout)

        # Start a thread to update the encoder value display (only for ODrive)
        if self.motor_type == 'ODrive':
            self.encoder_thread = threading.Thread(target=self.update_encoder_display, daemon=True)
            self.encoder_thread.start()

    def update_motor_type(self):
        """Update the motor type and switch the layout."""
        self.motor_type = self.motor_type_combo.currentText()
        self.update_motor_layout()

    def update_motor_layout(self):
        """Update the GUI layout based on the motor type."""
        # Clear the previous command layout (if already present)
        for i in reversed(range(self.command_layout.count())):
            widget = self.command_layout.itemAt(i).widget()
            if widget is not None:
                widget.deleteLater()

        # Remove encoder display if it's SERVO42 and add it if it's ODrive
        if self.motor_type == 'ODrive':
            if not hasattr(self, 'encoder_value_label'):
                self.encoder_value_label = QLabel("Encoder Value: 0.0000", self)
                self.encoder_value_label.setAlignment(QtCore.Qt.AlignCenter)
                self.main_layout.addWidget(self.encoder_value_label)

            # Start the encoder update thread if not already running
            if not hasattr(self, 'encoder_thread'):
                self.encoder_thread = threading.Thread(target=self.update_encoder_display, daemon=True)
                self.encoder_thread.start()
        else:
            # Remove the encoder label from the layout if not ODrive
            if hasattr(self, 'encoder_value_label'):
                self.encoder_value_label.deleteLater()
                del self.encoder_value_label

        # Update the motor node
        self.update_motor_node()

        # Add buttons for motor commands based on the selected motor type
        if self.motor_type == 'ODrive':
            self.add_odrive_buttons()
        else:
            self.add_servo42_buttons()

        # Update the main layout with the new command layout
        if self.command_layout not in self.main_layout.children():
            self.main_layout.addLayout(self.command_layout)

    def update_motor_node(self):
        """Initialize the motor node based on the selected motor type and CAN ID."""
        if self.motor_type == 'ODrive':
            self.ros2_node = ODriveCANNode(self.can_id)
        else:
            self.ros2_node = SERVO42CANNode(self.can_id)

    def add_odrive_buttons(self):
        """Add buttons for ODrive motor commands."""
        self.calibrate_button = QPushButton('Calibrate Motor', self)
        self.calibrate_button.clicked.connect(self.calibrate_motor_odrive)
        self.command_layout.addWidget(self.calibrate_button)

        self.set_torque_control_button = QPushButton('Set Torque Control Mode', self)
        self.set_torque_control_button.clicked.connect(self.set_torque_control_mode)
        self.command_layout.addWidget(self.set_torque_control_button)

        self.set_zero_button = QPushButton('Set Current Position to Zero', self)
        self.set_zero_button.clicked.connect(self.set_current_position_to_zero_odrive)
        self.command_layout.addWidget(self.set_zero_button)

        self.set_closed_loop_button = QPushButton('Set Closed-Loop Control', self)
        self.set_closed_loop_button.clicked.connect(self.set_closed_loop_control)
        self.command_layout.addWidget(self.set_closed_loop_button)

        self.set_position_control_button = QPushButton('Set Position Control Mode', self)
        self.set_position_control_button.clicked.connect(self.set_position_control_mode)
        self.command_layout.addWidget(self.set_position_control_button)

    def add_servo42_buttons(self):
        """Add buttons for SERVO42 motor commands."""
        self.calibrate_button = QPushButton('Calibrate Motor', self)
        self.calibrate_button.clicked.connect(self.calibrate_motor_servo)
        self.command_layout.addWidget(self.calibrate_button)

        self.set_mode_button = QPushButton('Set Mode to CR_OPEN', self)
        self.set_mode_button.clicked.connect(self.set_mode_cr_open)
        self.command_layout.addWidget(self.set_mode_button)

        self.set_zero_button = QPushButton('Set Current Position to Zero', self)
        self.set_zero_button.clicked.connect(self.set_current_position_to_zero_servo)
        self.command_layout.addWidget(self.set_zero_button)

        self.set_mode_vfoc_button = QPushButton('Set Mode to SR_vFOC', self)
        self.set_mode_vfoc_button.clicked.connect(self.set_mode_sr_vfoc)
        self.command_layout.addWidget(self.set_mode_vfoc_button)

    def update_can_id(self):
        """Update the CAN ID and restart the motor node with the new CAN ID."""
        self.can_id = int(self.can_id_combo.currentText())
        self.update_motor_node()

    def calibrate_motor_odrive(self):
        """Called when the Calibrate button is pressed."""
        self.ros2_node.full_calibration_sequences()

    def set_torque_control_mode(self):
        """Called when the Torque Control Mode button is pressed."""
        self.ros2_node.set_torque_control_mode()

    def set_current_position_to_zero_odrive(self):
        """Called when the Set Current Position to Zero button is pressed."""
        self.ros2_node.set_current_position_to_zero_odrive()

    def set_closed_loop_control(self):
        """Called when the Set Closed-Loop Control button is pressed."""
        self.ros2_node.set_closed_loop_control()

    def set_position_control_mode(self):
        """Called when the Set Position Control Mode button is pressed."""
        self.ros2_node.set_position_control_mode()

    def calibrate_motor_servo(self):
        self.ros2_node.calibrate_motor_servo()

    def set_mode_cr_open(self):
        self.ros2_node.set_mode(0)

    def set_current_position_to_zero_servo(self):
        self.ros2_node.set_axis_to_zero()

    def set_mode_sr_vfoc(self):
        self.ros2_node.set_mode(5)

    def update_encoder_display(self):
        """Periodically update the encoder value label with the latest value."""
        while True:
            if self.motor_type == 'ODrive':  # Only get encoder value if ODrive is selected
                encoder_value = self.ros2_node.get_encoder_value()
                self.encoder_value_label.setText(f"Encoder Value: {encoder_value:.4f}")
            time.sleep(1)

    def closeEvent(self, event):
        self.ros2_node.bus.shutdown()
        event.accept()



def main(args=None):
    rclpy.init(args=args)
    app = QApplication([])
    window = GUI()
    window.show()
    app.exec_()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



