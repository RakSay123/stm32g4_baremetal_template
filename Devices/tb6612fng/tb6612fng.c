#include "tb6612fng.h"

static TB6612FNG_Channel_Config_t *tb6612fng_get_channel(TB6612FNG_t *tb6612fng, TB6612FNG_Channel_t channel)
{
	switch (channel)
	{
		case TB6612FNG_CHA:
			return tb6612fng->channel_a;

		case TB6612FNG_CHB:
			return tb6612fng->channel_b;

		default:
			return NULL;
	}
}

static void tb6612fng_set_inputs(TB6612FNG_Channel_Config_t *channel, GPIO_State_t in1, GPIO_State_t in2)
{
	gpio_write(channel->in1, in1);
	gpio_write(channel->in2, in2);
}

TB6612FNG_Status_t tb6612fng_init(TB6612FNG_t *tb6612fng)
{
	GPIO_Config_t stby = {
		.port = tb6612fng->stby->port,
		.pin = tb6612fng->stby->pin,
		.mode = GPIO_MODE_OUTPUT,
		.otype = GPIO_PUSH_PULL,
		.speed = GPIO_SPEED_HIGH,
		.pull = GPIO_NO_PULL,
		.alternate = GPIO_AF0
	};

	GPIO_Config_t ain1 = {
		.port = tb6612fng->channel_a->in1->port,
		.pin = tb6612fng->channel_a->in1->pin,
		.mode = GPIO_MODE_OUTPUT,
		.otype = GPIO_PUSH_PULL,
		.speed = GPIO_SPEED_HIGH,
		.pull = GPIO_NO_PULL,
		.alternate = GPIO_AF0
	};

	GPIO_Config_t ain2 = {
		.port = tb6612fng->channel_a->in2->port,
		.pin = tb6612fng->channel_a->in2->pin,
		.mode = GPIO_MODE_OUTPUT,
		.otype = GPIO_PUSH_PULL,
		.speed = GPIO_SPEED_HIGH,
		.pull = GPIO_NO_PULL,
		.alternate = GPIO_AF0
	};

	GPIO_Config_t bin1 = {
		.port = tb6612fng->channel_b->in1->port,
		.pin = tb6612fng->channel_b->in1->pin,
		.mode = GPIO_MODE_OUTPUT,
		.otype = GPIO_PUSH_PULL,
		.speed = GPIO_SPEED_HIGH,
		.pull = GPIO_NO_PULL,
		.alternate = GPIO_AF0
	};

	GPIO_Config_t bin2 = {
		.port = tb6612fng->channel_b->in2->port,
		.pin = tb6612fng->channel_b->in2->pin,
		.mode = GPIO_MODE_OUTPUT,
		.otype = GPIO_PUSH_PULL,
		.speed = GPIO_SPEED_HIGH,
		.pull = GPIO_NO_PULL,
		.alternate = GPIO_AF0
	};

	GPIO_Config_t pwma = {
		.port = tb6612fng->channel_a->pwm_pin->port,
		.pin = tb6612fng->channel_a->pwm_pin->pin,
		.mode = GPIO_MODE_AF,
		.otype = GPIO_PUSH_PULL,
		.speed = GPIO_SPEED_HIGH,
		.pull = GPIO_NO_PULL,
		.alternate = tb6612fng->channel_a->pwm_af
	};

	GPIO_Config_t pwmb = {
		.port = tb6612fng->channel_b->pwm_pin->port,
		.pin = tb6612fng->channel_b->pwm_pin->pin,
		.mode = GPIO_MODE_AF,
		.otype = GPIO_PUSH_PULL,
		.speed = GPIO_SPEED_HIGH,
		.pull = GPIO_NO_PULL,
		.alternate = tb6612fng->channel_b->pwm_af
	};

	gpio_init(&stby);

	gpio_init(&ain1);
	gpio_init(&ain2);

	gpio_init(&bin1);
	gpio_init(&bin2);

	gpio_init(&pwma);
	gpio_init(&pwmb);

	tb6612fng_disable(tb6612fng);

	tb6612fng_coast(tb6612fng, TB6612FNG_CHA);
	tb6612fng_coast(tb6612fng, TB6612FNG_CHB);

	timer_pwm_init(tb6612fng->channel_a->pwm);
	timer_pwm_init(tb6612fng->channel_b->pwm);

	tb6612fng_set_duty_cycle(tb6612fng, TB6612FNG_CHA, 0);
	tb6612fng_set_duty_cycle(tb6612fng, TB6612FNG_CHB, 0);

	tb6612fng_enable(tb6612fng);

	return TB6612FNG_OK;
}

TB6612FNG_Status_t tb6612fng_set_direction(TB6612FNG_t *tb6612fng, TB6612FNG_Channel_t channel, TB6612FNG_Direction_t direction)
{
	TB6612FNG_Channel_Config_t *ch = tb6612fng_get_channel(tb6612fng, channel);
	if (ch == NULL) return TB6612FNG_ERR;

	switch (direction)
	{
		case TB6612FNG_DIRECTION_CW:
			tb6612fng_set_inputs(ch, GPIO_HIGH, GPIO_LOW);
			break;

		case TB6612FNG_DIRECTION_CCW:
			tb6612fng_set_inputs(ch, GPIO_LOW, GPIO_HIGH);
			break;

		default:
			return TB6612FNG_ERR;
	}

	return TB6612FNG_OK;
}

TB6612FNG_Status_t tb6612fng_set_duty_cycle(TB6612FNG_t *tb6612fng, TB6612FNG_Channel_t channel, uint16_t duty_cycle)
{
	TB6612FNG_Channel_Config_t *ch = tb6612fng_get_channel(tb6612fng, channel);
	if (ch == NULL) return TB6612FNG_ERR;

	if (timer_pwm_set_compare_value(ch->pwm, duty_cycle) != TIM_OK) return TB6612FNG_ERR;

	return TB6612FNG_OK;
}

TB6612FNG_Status_t tb6612fng_brake(TB6612FNG_t *tb6612fng, TB6612FNG_Channel_t channel)
{
	TB6612FNG_Channel_Config_t *ch = tb6612fng_get_channel(tb6612fng, channel);
	if (ch == NULL) return TB6612FNG_ERR;

	tb6612fng_set_inputs(ch, GPIO_HIGH, GPIO_HIGH);

	return TB6612FNG_OK;
}

TB6612FNG_Status_t tb6612fng_coast(TB6612FNG_t *tb6612fng, TB6612FNG_Channel_t channel)
{
	TB6612FNG_Channel_Config_t *ch = tb6612fng_get_channel(tb6612fng, channel);
	if (ch == NULL) return TB6612FNG_ERR;

	tb6612fng_set_inputs(ch, GPIO_LOW, GPIO_LOW);

	return TB6612FNG_OK;
}

TB6612FNG_Status_t tb6612fng_enable(TB6612FNG_t *tb6612fng)
{
	gpio_write(tb6612fng->stby, GPIO_HIGH);

	return TB6612FNG_OK;
}

TB6612FNG_Status_t tb6612fng_disable(TB6612FNG_t *tb6612fng)
{
	gpio_write(tb6612fng->stby, GPIO_LOW);

	return TB6612FNG_OK;
}
