# Position of one DCLab encoder (Smart Driver motor, Sensor encoder,
# Absolute Encoder). slot 0 = the frame's first (or only) encoder, slot 1 =
# the second encoder when two share a CAN ID. speed is estimated by the
# driver from successive positions (V2 boards send position only).
uint16 can_id
uint8 slot
float32 speed
float32 position
