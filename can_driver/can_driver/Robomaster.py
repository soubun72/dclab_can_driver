"""DJI RoboMaster C610/C620 ESCs.

  /publish_robomaster_current  RobomasterCurrentCommand  4 currents (A) per ID
                                                         (0x200: motors 1-4,
                                                         0x1FF: motors 5-8)
  /robomaster_feedback         RobomasterFeedback        from IDs 0x201-0x208
type 0 = C610 (+-10 A), type 1 = C620 (+-20 A); currents are clamped."""

import struct

from custom_messages.msg import RobomasterCurrentCommand, RobomasterFeedback

LIMITS = {0: (10.0, 1000.0), 1: (20.0, 819.2)}  # (max A, counts per A)


class Robomaster:
    def __init__(self, node):
        self.node = node
        self.publisher = node.create_publisher(RobomasterFeedback, "/robomaster_feedback", 10)
        node.create_subscription(RobomasterCurrentCommand, "/publish_robomaster_current",
                                 self.current, 10)

    def process_can_msg(self, msg):
        if msg.is_extended_id or msg.dlc != 8 or not 0x201 <= msg.arbitration_id <= 0x208:
            return False
        position, speed, current = struct.unpack_from(">hhh", bytes(msg.data), 0)
        out = RobomasterFeedback()
        out.motor_id = msg.arbitration_id - 0x200
        out.position = float(position)
        out.speed = float(speed)
        out.current = current * 0.001
        out.temp = float(msg.data[6])
        self.publisher.publish(out)
        return True

    def current(self, msg):
        currents = [msg.current1, msg.current2, msg.current3, msg.current4]
        types = [msg.type1, msg.type2, msg.type3, msg.type4]
        data = bytearray(8)
        for i in range(4):
            max_current, scale = LIMITS.get(types[i], LIMITS[0])
            value = max(-max_current, min(max_current, float(currents[i])))
            struct.pack_into(">h", data, 2 * i, int(value * scale))
        self.node.send(msg.can_id, data)
