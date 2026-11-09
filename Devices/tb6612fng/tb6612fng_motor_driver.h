#pragma once

#include "motor_driver/motor_driver.h"
#include "tb6612fng.h"

MOTOR_DRIVER_Status_t tb6612fng_motor_driver_bind(MOTOR_DRIVER_t *motor_driver, TB6612FNG_t *tb6612fng);
