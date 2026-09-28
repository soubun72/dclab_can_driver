"""DCLab Swerve module: /publish_swerve (SwerveCommand) and
/swerve_feedback (SwerveFeedback). Parameter swerve_feedback_ids
([first, last] pairs, default [250, 299])."""

from custom_messages.msg import SwerveCommand, SwerveFeedback

from . import dclab_frames as frames
from .id_ranges import declare_ranges, in_ranges


class SwerveModule:
    def __init__(self, node):
        self.node = node
        self.ranges = declare_ranges(node, "swerve_feedback_ids", [250, 299])
        node.create_subscription(SwerveCommand, "/publish_swerve", self.command, 10)
        self.publisher = node.create_publisher(SwerveFeedback, "/swerve_feedback", 10)

    def process_can_msg(self, msg):
        if msg.is_extended_id or msg.dlc != 8 or not in_ranges(self.ranges, msg.arbitration_id):
            return False
        out = SwerveFeedback()
        out.can_id = msg.arbitration_id
        out.position, out.speed = frames.swerve_feedback(msg.data[:8])
        self.publisher.publish(out)
        return True

    def command(self, msg):
        self.node.send(msg.can_id, frames.swerve_command(msg.position, msg.speed))
