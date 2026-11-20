#include "dual_motor_controller.h"

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_init(DUAL_MOTOR_CONTROLLER_t *dual, MOTOR_CONTROLLER_t *left, MOTOR_CONTROLLER_t *right)
{
	if (dual == NULL || dual->left == NULL || dual->right == NULL) return DUAL_MOTOR_CONTROLLER_ERR;

	dual->left = left;
	dual->right = right;
	dual->enabled = false;

	return DUAL_MOTOR_CONTROLLER_OK;
}

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_set_rpm(DUAL_MOTOR_CONTROLLER_t *dual, float left_rpm, float right_rpm)
{
	if (dual == NULL || dual->left == NULL || dual->right == NULL) return DUAL_MOTOR_CONTROLLER_ERR;

	MOTOR_CONTROLLER_Status_t status = MOTOR_CONTROLLER_OK;

	status = motor_controller_set_rpm(dual->left, left_rpm);
	if (status != MOTOR_CONTROLLER_OK) return DUAL_MOTOR_CONTROLLER_ERR;

	status = motor_controller_set_rpm(dual->right, right_rpm);
	if (status != MOTOR_CONTROLLER_OK) return DUAL_MOTOR_CONTROLLER_ERR;

	return status;
}

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_update(DUAL_MOTOR_CONTROLLER_t *dual, uint32_t current_ms, float dt_seconds)
{
	if (dual == NULL || dual->left == NULL || dual->right == NULL) return DUAL_MOTOR_CONTROLLER_ERR;
	if (!dual->enabled) return DUAL_MOTOR_CONTROLLER_ERR;

	if (motor_controller_update(dual->left, current_ms, dt_seconds) != MOTOR_CONTROLLER_OK)
	{
		dual_motor_controller_disable(dual);
		return DUAL_MOTOR_CONTROLLER_ERR;
	}

	if (motor_controller_update(dual->right, current_ms, dt_seconds) != MOTOR_CONTROLLER_OK)
	{
		dual_motor_controller_disable(dual);
		return DUAL_MOTOR_CONTROLLER_ERR;
	}

	return DUAL_MOTOR_CONTROLLER_OK;;
}

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_set_feedforward(DUAL_MOTOR_CONTROLLER_t *dual, bool enabled)
{
	if (dual == NULL || dual->left == NULL || dual->right == NULL) return DUAL_MOTOR_CONTROLLER_ERR;

	    if (motor_controller_set_feedforward(dual->left, enabled) != MOTOR_CONTROLLER_OK) return DUAL_MOTOR_CONTROLLER_ERR;

	    if (motor_controller_set_feedforward(dual->right, enabled) != MOTOR_CONTROLLER_OK) return DUAL_MOTOR_CONTROLLER_ERR;

	    return DUAL_MOTOR_CONTROLLER_OK;
}

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_enable(DUAL_MOTOR_CONTROLLER_t *dual)
{
	if (dual == NULL || dual->left == NULL || dual->right == NULL) return DUAL_MOTOR_CONTROLLER_ERR;

	MOTOR_CONTROLLER_Status_t left_status = motor_controller_enable(dual->left);
	MOTOR_CONTROLLER_Status_t right_status = motor_controller_enable(dual->right);

	if (left_status != MOTOR_CONTROLLER_OK || right_status != MOTOR_CONTROLLER_OK)
	{
		dual_motor_controller_disable(dual);
		return DUAL_MOTOR_CONTROLLER_ERR;
	}

	dual->enabled = true;

	return DUAL_MOTOR_CONTROLLER_OK;
}

DUAL_MOTOR_CONTROLLER_Status_t dual_motor_controller_disable(DUAL_MOTOR_CONTROLLER_t *dual)
{
	if (dual == NULL || dual->left == NULL || dual->right == NULL) return DUAL_MOTOR_CONTROLLER_ERR;

	dual->enabled = false;

	MOTOR_CONTROLLER_Status_t left_status = motor_controller_disable(dual->left);

	MOTOR_CONTROLLER_Status_t right_status = motor_controller_disable(dual->right);

	if (left_status != MOTOR_CONTROLLER_OK || right_status != MOTOR_CONTROLLER_OK) return DUAL_MOTOR_CONTROLLER_ERR;

	return DUAL_MOTOR_CONTROLLER_OK;
}
