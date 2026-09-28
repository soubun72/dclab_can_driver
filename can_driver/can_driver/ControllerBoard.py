"""DCLab Controller board outputs (frame format unchanged from V1).

  /publish_pwm                PwmCommand                 PWM 1-4 duty, 0..1
  /publish_servo              ServoCommand               servo 1-4, 0..1
  /publish_digital_solenoid   DigitalAndSolenoidCommand  digital 1-4, solenoid 1-6
Values outside 0..1 are clamped."""

from custom_messages.msg import DigitalAndSolenoidCommand, PwmCommand, ServoCommand

from . import dclab_frames as frames


class ControllerBoard:
    def __init__(self, node):
        self.node = node
        node.create_subscription(ServoCommand, "/publish_servo", self.servo, 10)
        node.create_subscription(PwmCommand, "/publish_pwm", self.pwm, 10)
        node.create_subscription(DigitalAndSolenoidCommand, "/publish_digital_solenoid",
                                 self.digital, 10)

    def process_can_msg(self, msg):
        return False

    def pwm(self, msg):
        self.node.send(msg.can_id, frames.controller_pwm(
            [msg.pwm1_value, msg.pwm2_value, msg.pwm3_value, msg.pwm4_value]))

    def servo(self, msg):
        self.node.send(msg.can_id, frames.controller_servo(
            [msg.servo1_value, msg.servo2_value, msg.servo3_value, msg.servo4_value]))

    def digital(self, msg):
        self.node.send(msg.can_id, frames.controller_digital(
            [msg.digital1_value, msg.digital2_value, msg.digital3_value, msg.digital4_value],
            [msg.solenoid1_value, msg.solenoid2_value, msg.solenoid3_value,
             msg.solenoid4_value, msg.solenoid5_value, msg.solenoid6_value]))
