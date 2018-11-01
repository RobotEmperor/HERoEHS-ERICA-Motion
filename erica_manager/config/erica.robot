[ control info ]
control_cycle = 8   # milliseconds

[ port info ]
# PORT NAME  | BAUDRATE
/dev/ttyUSB0| 3000000  | l_arm_sh_y


[ device info ]
## TYPE   | PORT NAME    | ID   | MODEL             | PROTOCOL | DEV NAME     | BULK READ ITEMS
#dynamixel | /dev/ttyUSB0 |  1   | H54-100-S500-R-1  | 2.0      | l_arm_sh_p   | present_position
#dynamixel | /dev/ttyUSB0 |  2   | H54-100-S500-R-2  | 2.0      | r_arm_sh_p   | present_position

dynamixel | /dev/ttyUSB0 |  3   | XM540-W270-3      | 2.0      | l_arm_sh_r   | present_position
dynamixel | /dev/ttyUSB0 |  4   | XM540-W270-4      | 2.0      | r_arm_sh_r   | present_position
dynamixel | /dev/ttyUSB0 |  5   | XM540-W270-5      | 2.0      | l_arm_sh_y   | present_position
dynamixel | /dev/ttyUSB0 |  6   | XM540-W270-6      | 2.0      | r_arm_sh_y   | present_position
dynamixel | /dev/ttyUSB0 |  7   | XM540-W270-7      | 2.0      | l_arm_el_p   | present_position
dynamixel | /dev/ttyUSB0 |  8   | XM540-W270-8      | 2.0      | r_arm_el_p   | present_position

dynamixel | /dev/ttyUSB0 |  9   | XM-430-W350-9     | 2.0      | l_arm_wr_p   | present_position
dynamixel | /dev/ttyUSB0 | 10   | XM-430-W350-10    | 2.0      | r_arm_wr_p   | present_position
dynamixel | /dev/ttyUSB0 | 11   | XM-430-W350-11    | 2.0      | l_arm_wr_r   | present_position
dynamixel | /dev/ttyUSB0 | 12   | XM-430-W350-12    | 2.0      | r_arm_wr_r   | present_position


#dynamixel | /dev/ttyUSB0 | 13   | XM-430-W350-13    | 2.0      | head_yaw     | present_position
#dynamixel | /dev/ttyUSB0 | 14   | XM-430-W350-14    | 2.0      | head_pitch   | present_position
#dynamixel | /dev/ttyUSB0 | 15   | XM-430-W350-15    | 2.0      | head_roll    | present_position
