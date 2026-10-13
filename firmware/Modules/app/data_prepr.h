#ifndef FILTER_PROCES_H
#define FILTER_PROCES_H

#include "../Fusion/Fusion.h"
#include "common.h"
#include "ism20948_driver.h"

#define SAMPLE_RATE 200.0f //should be equal to imu meas upd rate
#define GAIN_BETA   1.2f  //gain or beta coef in ahrs

void Madgwick_Filter_Init(void);
void update_orientation(imu_sample_t* meas);
imu_euler_orient_t* get_euler_orient_sample(void);
imu_quater_orient_t* get_quater_orient_sample(void);


#endif