"""DCLab Smart Driver commands.

Firmware 2.6+: 4 bytes per motor (see dclab_frames). Topics:
  /smart_driver/command       SmartDriverCommand      one motor, DLC 4
  /smart_driver/pair_command  SmartDriverPairCommand  both motors sharing
                                                      one receive ID, DLC 8
  /publish_motor              MotorCommand            the V1 message, kept for
                                                      existing code
Parameter v1_frames (default false) sends V1's 8-byte frame to boards still
running V1 firmware."""

from custom_messages.msg import MotorCommand, SmartDriverCommand, SmartDriverPairCommand

from . import dclab_frames as frames


class DCLabSmartDriver:
    def __init__(self, node):
        self.node = node
        node.create_subscription(SmartDriverCommand, "/smart_driver/command", self.command, 10)
        node.create_subscription(SmartDriverPairCommand, "/smart_driver/pair_command",
                                 self.pair_command, 10)
        node.create_subscription(MotorCommand, "/publish_motor", self.motor_command, 10)

    def process_can_msg(self, msg):
        return False  # feedback is handled by DCLabEncoders

    def command(self, msg):
        try:
            self.node.send(msg.can_id, frames.smart_driver_command(msg.mode, msg.goal))
        except ValueError as error:
            self.node.get_logger().warn(f"Smart Driver command refused: {error}")

    def pair_command(self, msg):
        try:
            self.node.send(msg.can_id, frames.smart_driver_pair_command(
                msg.mode1, msg.goal1, msg.mode2, msg.goal2))
        except ValueError as error:
            self.node.get_logger().warn(f"Smart Driver command refused: {error}")

    def motor_command(self, msg):
        """V1 MotorCommand: flags become modes; stop is 0 V."""
        if self.node.v1_frames:
            self.node.send(msg.can_id, frames.legacy_smart_driver_command(
                msg.goal, msg.positionmode, msg.speedmode, msg.voltagemode, msg.stop, msg.reset))
            return
        if msg.reset:
            self.node.get_logger().warn(
                "MotorCommand.reset is not a CAN command since firmware 2.6; "
                "use Reset Position in the Super App", throttle_duration_sec=5.0)
        if msg.stop:
            mode, goal = frames.MODE_VOLTAGE, 0.0
        elif msg.positionmode:
            mode, goal = frames.MODE_POSITION, msg.goal
        elif msg.speedmode:
            mode, goal = frames.MODE_SPEED, msg.goal
        elif msg.voltagemode:
            mode, goal = frames.MODE_VOLTAGE, msg.goal
        else:
            return
        try:
            self.node.send(msg.can_id, frames.smart_driver_command(mode, goal))
        except ValueError as error:
            self.node.get_logger().warn(f"Smart Driver command refused: {error}")
