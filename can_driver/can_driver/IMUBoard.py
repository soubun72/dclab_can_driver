"""DCLab IMU board quaternion on /imu/quaternion (geometry_msgs/Quaternion).
Parameter imu_id (default 1000)."""

from geometry_msgs.msg import Quaternion

from . import dclab_frames as frames


class IMUBoard:
    def __init__(self, node):
        self.imu_id = int(node.declare_parameter("imu_id", 1000).value)
        self.publisher = node.create_publisher(Quaternion, "/imu/quaternion", 10)

    def process_can_msg(self, msg):
        if msg.is_extended_id or msg.arbitration_id != self.imu_id or msg.dlc != 8:
            return False
        q = Quaternion()
        q.w, q.x, q.y, q.z = frames.imu_quaternion(msg.data[:8])
        self.publisher.publish(q)
        return True
