#pragma once

#include "led/led.h"
#include "rotary_encoder/rotary_encoder.h"
#include "tb6612fng/tb6612fng.h"
#include "motor_driver/motor_driver.h"
#include "dc_motor/dc_motor.h"
#include "motor_controller/motor_controller.h"
#include "console/console.h"

LED_t* board_get_status_led(void);

ROTARY_ENCODER_t* board_get_left_motor_encoder(void);

ROTARY_ENCODER_t* board_get_right_motor_encoder(void);

TB6612FNG_t* board_get_tb6612fng(void);

MOTOR_DRIVER_t* board_get_motor_driver(void);

DC_MOTOR_t* board_get_left_motor(void);

DC_MOTOR_t* board_get_right_motor(void);

MOTOR_CONTROLLER_t* board_get_left_motor_controller(void);

MOTOR_CONTROLLER_t* board_get_right_motor_controller(void);

CONSOLE_t* board_get_console(void);
