#ifndef USER_LIB_H
#define USER_LIB_H
#include "arm_math.h"

typedef struct
{
    float32_t input;        //input value
    float32_t out;          //output value
    float32_t min_value;    //constrain minimum value
    float32_t max_value;    //constrain maximum value
    float32_t frame_period; //time interval
} __packed ramp_function_source_t;

typedef struct
{
	float32_t input;        //input value
	float32_t out;          //output value after filtering
	float32_t num[1];       //filter parameter
	float32_t frame_period; //filter time interval, unit: s
} __packed first_order_filter_type_t;

//fast square root
extern float32_t invSqrt(float32_t num);

//ramp function initialization
void ramp_init(ramp_function_source_t *ramp_source_type, float32_t frame_period, float32_t max, float32_t min);

//ramp function calculation
void ramp_calc(ramp_function_source_t *ramp_source_type, float32_t input);
//first-order low-pass filter initialization
extern void first_order_filter_init(first_order_filter_type_t *first_order_filter_type, float32_t frame_period, const float32_t num[1]);
//first-order low-pass filter calculation
extern void first_order_filter_cali(first_order_filter_type_t *first_order_filter_type, float32_t input);
//absolute limit
extern void abs_limit(float32_t *num, float32_t Limit);
//check sign (positive or negative)
extern float32_t sign(float32_t value);
//dead zone limit (float32_t version)
extern float32_t fp32_deadline(float32_t Value, float32_t minValue, float32_t maxValue);
//dead zone limit (int16_t version)
extern int16_t int16_deadline(int16_t Value, int16_t minValue, int16_t maxValue);
//constrain (float32_t version)
extern float32_t fp32_constrain(float32_t Value, float32_t minValue, float32_t maxValue);
//constrain (int16_t version)
extern int16_t int16_constrain(int16_t Value, int16_t minValue, int16_t maxValue);
//loop constrain (float32_t version)
extern float32_t loop_fp32_constrain(float32_t Input, float32_t minValue, float32_t maxValue);
//angle formatted in range -180 ~ +180 (degree version)
extern float32_t theta_format(float32_t Ang);

//angle formatted in range -PI ~ +PI (radius version)
#define rad_format(Ang) loop_fp32_constrain((Ang), -PI, PI)

#endif
