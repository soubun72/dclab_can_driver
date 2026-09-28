"""VESC motor controllers over CAN (extended IDs).

  /publish_vesc   VescCommand     one of pwm_mode (-1..1), current_mode (A),
                                  current_brake_mode (A), speed_mode (ERPM),
                                  position_mode (degrees)
  /vesc_status1..4                status frames 1-4 of IDs 0..255"""

import struct

from custom_messages.msg import VescCommand, VescStatusFour, VescStatusOne, VescStatusThree, VescStatusTwo

STATUS_1, STATUS_2, STATUS_3, STATUS_4 = 9, 14, 15, 16  # VESC CAN packet ids


class Vesc:
    def __init__(self, node):
        self.node = node
        node.create_subscription(VescCommand, "/publish_vesc", self.command, 10)
        self.publishers = {
            STATUS_1: node.create_publisher(VescStatusOne, "/vesc_status1", 10),
            STATUS_2: node.create_publisher(VescStatusTwo, "/vesc_status2", 10),
            STATUS_3: node.create_publisher(VescStatusThree, "/vesc_status3", 10),
            STATUS_4: node.create_publisher(VescStatusFour, "/vesc_status4", 10),
        }

    def command(self, msg):
        if msg.pwm_mode:
            packet, value = 0, max(-1.0, min(1.0, msg.goal)) * 1e5
        elif msg.current_mode:
            packet, value = 1, msg.goal * 1e3
        elif msg.current_brake_mode:
            packet, value = 2, msg.goal * 1e3
        elif msg.speed_mode:
            packet, value = 3, msg.goal
        elif msg.position_mode:
            packet, value = 4, msg.goal * 1e6
        else:
            self.node.get_logger().warn("VescCommand without a mode ignored",
                                        throttle_duration_sec=5.0)
            return
        self.node.send((packet << 8) | (msg.vesc_id & 0xFF), struct.pack(">i", int(value)),
                       extended=True)

    def process_can_msg(self, msg):
        if not msg.is_extended_id or msg.dlc != 8:
            return False
        packet, vesc_id = msg.arbitration_id >> 8, msg.arbitration_id & 0xFF
        publisher = self.publishers.get(packet)
        if publisher is None:
            return False
        data = bytes(msg.data[:8])
        if packet == STATUS_1:
            out = VescStatusOne()
            rpm, current, duty = struct.unpack(">ihh", data)
            out.rpm, out.current, out.duty = float(rpm), current * 0.1, duty * 0.001
        elif packet == STATUS_2:
            out = VescStatusTwo()
            used, charged = struct.unpack(">ii", data)
            out.amp_hours, out.amp_hours_charged = used * 1e-4, charged * 1e-4
        elif packet == STATUS_3:
            out = VescStatusThree()
            used, charged = struct.unpack(">ii", data)
            out.watt_hours, out.watt_hours_charged = used * 1e-4, charged * 1e-4
        else:
            out = VescStatusFour()
            fet, motor, current, position = struct.unpack(">hhhh", data)
            out.fet_temp, out.motor_temp = fet * 0.1, motor * 0.1
            out.input_current, out.pid_position = current * 0.1, position * 0.02
        out.esc_id = vesc_id
        publisher.publish(out)
        return True
