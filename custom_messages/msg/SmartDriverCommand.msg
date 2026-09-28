# One DCLab Smart Driver motor on its own receive ID (firmware 2.6+, DLC 4).
uint8 MODE_VOLTAGE=0
uint8 MODE_POSITION=1
uint8 MODE_SPEED=2
uint8 MODE_POSITION_RAMP=3  # goal = ramp rate until the board restarts
uint8 MODE_SPEED_RAMP=4
uint8 MODE_VOLTAGE_RAMP=5

uint16 can_id
uint8 mode
float32 goal   # V (voltage), rad (position), rad/s (speed)
