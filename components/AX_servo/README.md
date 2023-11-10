# Dynamixel AX-12A servo ESP32 component for ESP IDF
This is an ESP IDF component to steer Dynamixel AX-12A / AX-12+ servo with ESP32 microcontroller.
Feel free to use and contribute to it!

## AX_servo instruction list:
* void AX_servo_init(AX_conf_t AX) - init ESP32 UART connection
* void AX_servo_deinit(AX_conf_t AX) - deinit ESP32 UART connection
* void AX_servo_ping(AX_conf_t AX, uint8_t ID) - ping servo
* void AX_servo_set_pos(AX_conf_t AX, uint8_t id, uint16_t pos) - set goal position of servo
* void AX_servo_set_pos_w_spd(AX_conf_t AX, uint8_t ID, uint16_t pos, uint16_t speed) - set goal position of servo with desired speed
* void AX_servo_set_endless(AX_conf_t AX, uint8_t ID, bool status) - enable endless turn
* void AX_conf_turn(AX_conf_t AX, uint8_t ID, uint8_t side, uint16_t speed) - turn servo endlessly
* void AX_servo_set_max_torque(AX_conf_t AX, uint8_t ID, uint16_t torque) - set max torque of servo
* void AX_conf_torque_status(AX_conf_t AX, uint8_t ID, uint8_t status) - set torque status
* uint16_t AX_servo_get_pos(AX_conf_t AX, uint8_t ID) - get actual position of servo
* void AX_servo_set_angle_limit(AX_conf_t AX, uint8_t ID, uint16_t CWLimit, uint16_t CCWLimit) - set limit angles of servo
* void AX_servo_set_led(AX_conf_t AX, uint8_t ID, uint8_t status) - set LED level
* uint16_t AX_servo_read_register(AX_conf_t AX, uint8_t ID, uint8_t reg, uint8_t reg_len) - read value from register
* void AX_servo_write_register(AX_conf_t AX, uint8_t ID, uint8_t reg, uint8_t reg_val) - write value to register
* void AX_servo_reset(AX_conf_t AX, uint8_t ID) - factory reset (reset sets id to 1 and baudrate to 1000000 and sets back original values of them) [!!! might not work properly - esp32 does not like 10M baud !!!]
* void AX_servo_set_ID(AX_conf_t AX, uint8_t ID, uint8_t newID) - set servo ID
* void AX_servo_set_BD(AX_conf_t AX, uint8_t ID, uint8_t BAUD) - set servo baudrate
* int16_t AX_servo_read_temperature(AX_conf_t AX, uint8_t ID) - read temerature of servo
* uint16_t AX_servo_is_moving(AX_conf_t AX, uint8_t ID) - read if servo is still moving flag
* uint16_t AX_servo_read_load(AX_conf_t AX, uint8_t ID) - read load from servo
* void AX_servo_set_shutdown_alarm(AX_conf_t AX, uint8_t ID, uint8_t salarm) - set servo alarm
* void AX_servo_set_C_margin(AX_conf_t AX, uint8_t ID, uint8_t CWCMargin, uint8_t CCWCMargin) - set compliance margin
* void AX_servo_set_C_slope(AX_conf_t AX, uint8_t ID, uint8_t CWCSlope, uint8_t CCWCSlope) - set compliance slope
* void AX_servo_set_punch(AX_conf_t AX, uint8_t ID, uint16_t punch) - set punch
* void AX_servo_send_packet(AX_conf_t AX, uint8_t* packet, uint32_t len) - send command to servo
* void AX_servo_receive_response(AX_conf_t AX, uint8_t* response, uint32_t len) - receive response from servo
* bool AX_servo_check_response(uint8_t ID, uint8_t* response, uint32_t len) - double check if response is valid

It is recommended to wait 10 ms to make sure that servo has received critical UART package.

If read operation fails too many times (see UART_MAX_REPEAT and UART_TIMEOUT_MS), ESP32 will reset itself.

## Sources:
* https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
* http://www.da.isy.liu.se/vanheden/pdf/AX-12.pdf
* https://emanual.robotis.com/docs/en/dxl/protocol1/
* https://github.com/jumejume1/AX-12A-servo-library.git
