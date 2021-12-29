#include "user_lib.h"

//fast square root
float32_t invSqrt(float32_t num)
{
	float32_t halfnum = 0.5f * num;
	float32_t y = num;
    long i = *(long *)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float32_t *)&i;
    y = y * (1.5f - (halfnum * y * y));
    return y;
}

/**
  * @brief          ramp function initialization
  * @author         RM
  * @param[in]      ramp_source_type: ramp function struct pointer
  * @param[in]      frame_period: time interval, unit: s
  * @param[in]      max: maximum value
  * @param[in]      min: minimum value
  * @retval         none
  */
void ramp_init(ramp_function_source_t *ramp_source_type, float32_t frame_period, float32_t max, float32_t min)
{
    ramp_source_type->frame_period = frame_period;
    ramp_source_type->max_value = max;
    ramp_source_type->min_value = min;
    ramp_source_type->input = 0.0f;
    ramp_source_type->out = 0.0f;
}

/**
  * @brief          ramp function calculation. accumulate based on input.
  *                 input unit: per second, i.e. the input value should be
  *                 the one after one second.
  * @author         RM
  * @param[in]      ramp_source_type: ramp function struct pointer
  * @param[in]      input: input value
  * @retval         none
  */
void ramp_calc(ramp_function_source_t *ramp_source_type, float32_t input)
{
    ramp_source_type->input = input;
    ramp_source_type->out += ramp_source_type->input * ramp_source_type->frame_period;
    if (ramp_source_type->out > ramp_source_type->max_value)
    {
        ramp_source_type->out = ramp_source_type->max_value;
    }
    else if (ramp_source_type->out < ramp_source_type->min_value)
    {
        ramp_source_type->out = ramp_source_type->min_value;
    }
}
/**
  * @brief          first-order low-pass filter initialization
  * @author         RM
  * @param[in]      first_order_filter_type: first-order low-pass filter struct pointer
  * @param[in]      frame_period: time interval, unit: s
  * @param[in]      num: filter parameter
  * @retval         none
  */
void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float32_t frame_period, const float32_t num[1])
{
    first_order_filter_type->frame_period = frame_period;
    first_order_filter_type->num[0] = num[0];
    first_order_filter_type->input = 0.0f;
    first_order_filter_type->out = 0.0f;
}

/**
  * @brief          first-order low-pass filter calculation
  * @author         RM
  * @param[in]      first_order_filter_type: first-order low-pass filter struct pointer
  * @param[in]      input: input value
  * @retval         none
  */
void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float32_t input)
{
    first_order_filter_type->input = input;
    first_order_filter_type->out =
        first_order_filter_type->num[0] / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->out + first_order_filter_type->frame_period / (first_order_filter_type->num[0] + first_order_filter_type->frame_period) * first_order_filter_type->input;
}

//absolute limit
void abs_limit(float32_t *num, float32_t Limit)
{
    if (*num > Limit)
    {
        *num = Limit;
    }
    else if (*num < -Limit)
    {
        *num = -Limit;
    }
}

//check sign (positive or negative)
float32_t sign(float32_t value)
{
    if (value >= 0.0f)
    {
        return 1.0f;
    }
    else
    {
        return -1.0f;
    }
}

//dead zone limit (float32_t version)
float32_t fp32_deadline(float32_t Value, float32_t minValue, float32_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0.0f;
    }
    return Value;
}

//dead zone limit (int16_t version)
int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < maxValue && Value > minValue)
    {
        Value = 0;
    }
    return Value;
}

//constrain (float32_t version)
float32_t fp32_constrain(float32_t Value, float32_t minValue, float32_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//constrain (int16_t version)
int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue)
{
    if (Value < minValue)
        return minValue;
    else if (Value > maxValue)
        return maxValue;
    else
        return Value;
}

//loop constrain (float32_t version)
float32_t loop_fp32_constrain(float32_t Input, float32_t minValue, float32_t maxValue)
{
    if (maxValue < minValue)
    {
        return Input;
    }

    if (Input > maxValue)
    {
    	float32_t len = maxValue - minValue;
        while (Input > maxValue)
        {
            Input -= len;
        }
    }
    else if (Input < minValue)
    {
    	float32_t len = maxValue - minValue;
        while (Input < minValue)
        {
            Input += len;
        }
    }
    return Input;
}

//angle formatted in range -PI ~ +PI (radius version)
//this function is provided as a macro function in .h file

//angle formatted in range -180 ~ +180 (degree version)
float32_t theta_format(float32_t Ang)
{
    return loop_fp32_constrain(Ang, -180.0f, 180.0f);
}
