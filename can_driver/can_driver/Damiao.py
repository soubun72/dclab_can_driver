"""Damiao motors in speed mode: /publish_damiao (DamiaoCommand). The first
command with arm=true enables the motor; speed is in rad/s. Speed-mode ID is
0x200 + motor_id (standard)."""

import struct

from custom_messages.msg import DamiaoCommand

ENABLE = bytes([0xFF] * 7 + [0xFC])


class Damiao:
    def __init__(self, node):
        self.node = node
        node.create_subscription(DamiaoCommand, "/publish_damiao", self.command, 10)

    def process_can_msg(self, msg):
        return False

    def command(self, msg):
        can_id = 0x200 + msg.motor_id
        if msg.arm:
            self.node.send(can_id, ENABLE)
            return
        self.node.send(can_id, struct.pack("<f", float(msg.speed)))
