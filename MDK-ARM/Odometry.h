#include "tim.h"
#include <math.h>
#define PI (float) 3.14159265358979323846
#define T 10

void set_right_encoder(TIM_HandleTypeDef* htim, TIM_TypeDef* TIM, int resolution, int precision, int sens);
void set_left_encoder(TIM_HandleTypeDef* htim, TIM_TypeDef* TIM, int resolution, int precision, int sens);
void set_dimentions(float right_wheel_radius, float left_wheel_radius, float encoder_spacing, float wheels_spacing);
void speed_calcul(void);

void read_right_encoder(void);
void read_left_encoder(void);

void update_position(void);
void reset_position(void);

float ticks_to_distance(int x, float r, int resolution, int precision);
float rad_to_deg(double x);
