#include "filter_proces.h"

FusionAhrs ahrs;
FusionAhrsFlags ahrs_flags;

uint64_t fusion_time_us = 0;
bool first_update = true;

imu_euler_orient_t euler_buf = { .roll = 0.0f, .pitch = 0.0f, .yaw = 0.0f, 
                         .time_us = 0};

imu_quater_orient_t quater_buf = { .w = 0.0f, 
                                   .x = 0.0f, 
                                   .y = 0.0f, 
                                   .z = 0.0f,
                                   .time_us = 0};

void Madgwick_Filter_Init(void){
    /*Madgwick filter cfg*/
    FusionAhrsInitialise(&ahrs);
    FusionAhrsSettings settings = {
        .sampleRate = SAMPLE_RATE,
        .convention = FusionConventionEnu, // FusionConventionEnu FusionConventionNwu FusionConventionNed,
        .gain = GAIN_BETA,
        .gyroscopeRange = 500.0f,
        .accelerationRejection = 20.0f,
        .magneticRejection = 20.0f,
        .rejectionTimeout = 5.0f
    };
    FusionAhrsSetSettings(&ahrs, &settings);
    
    ahrs_flags = FusionAhrsGetFlags(&ahrs);
    if (ahrs_flags.startup)
        set_led(LED1);
    else
        reset_led(LED1);
    //
    if (ahrs_flags.overrangeRecovery)
        set_led(LED2);
    else
        reset_led(LED2);
    //
    if (ahrs_flags.accelerationRecovery)
        set_led(LED3);
    else
        reset_led(LED3);
}

void update_orientation(imu_sample_t* meas){ 
    //Set real time delta in seconds
    if(first_update){
        first_update = false;
        FusionAhrsSetSamplePeriod(&ahrs, (1 / SAMPLE_RATE));
    }
    else{
        FusionAhrsSetSamplePeriod(&ahrs, (meas->time_us - fusion_time_us)*0.000001f);
    }
    fusion_time_us = meas->time_us; //copy

    FusionVector gyroscope = {meas->gyro[0], 
                              meas->gyro[1], 
                              meas->gyro[2]};
    FusionVector accelerometer = {meas->accel[0], 
                                  meas->accel[1], 
                                  meas->accel[2]};

    FusionVector magnetometer = {meas->mag[0], 
                                 meas->mag[1],
                                 meas->mag[2]};
    FusionAhrsUpdate(&ahrs, gyroscope, accelerometer, magnetometer);

    //FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer);

    ahrs_flags = FusionAhrsGetFlags(&ahrs);
    if (ahrs_flags.startup)
        set_led(LED1);
    else
        reset_led(LED1);
    //
    if (ahrs_flags.overrangeRecovery)
        set_led(LED2);
    else
        reset_led(LED2);
    //
    if (ahrs_flags.accelerationRecovery)
            set_led(LED3);
    else
        reset_led(LED3);
}


imu_euler_orient_t* get_euler_orient_sample(void){
    // Convertion from quaternion to euler angle 
    const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    euler_buf.roll = euler.angle.roll;
    euler_buf.pitch = euler.angle.pitch;
    euler_buf.yaw = euler.angle.yaw;
    euler_buf.time_us = fusion_time_us;
    return &euler_buf;
}

imu_quater_orient_t* get_quater_orient_sample(void){
    const FusionQuaternion q = FusionAhrsGetQuaternion(&ahrs);
    quater_buf.w = q.element.w;
    quater_buf.x = q.element.x;
    quater_buf.y = q.element.y;
    quater_buf.z = q.element.z;
    quater_buf.time_us = fusion_time_us;
    return &quater_buf;
}





