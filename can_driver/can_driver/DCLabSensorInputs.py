"""DCLab Sensor board digital/analog inputs on /digital_analog_feedback.

Parameter sensor_input_ids ([first, last] pairs, default [500, 510])."""

from custom_messages.msg import DigitalAndAnalogFeedback

from . import dclab_frames as frames
from .id_ranges import declare_ranges, in_ranges


class DCLabSensorInputs:
    def __init__(self, node):
        self.ranges = declare_ranges(node, "sensor_input_ids", [500, 510])
        self.publisher = node.create_publisher(DigitalAndAnalogFeedback, "/digital_analog_feedback", 10)

    def process_can_msg(self, msg):
        if msg.is_extended_id or msg.dlc != 8 or not in_ranges(self.ranges, msg.arbitration_id):
            return False
        analog, digital = frames.sensor_inputs(msg.data[:8])
        out = DigitalAndAnalogFeedback()
        out.can_id = msg.arbitration_id
        (out.analog1_value, out.analog2_value, out.analog3_value,
         out.analog4_value, out.analog5_value) = analog
        (out.digital1_value, out.digital2_value, out.digital3_value,
         out.digital4_value) = digital
        self.publisher.publish(out)
        return True
