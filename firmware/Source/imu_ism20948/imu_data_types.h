#ifndef IMU_DATA_TYPES_H
#define IMU_DATA_TYPES_H

#include <stdint.h>

//imu raw measurements 
typedef struct{
    int16_t    r_accel[3];
    int16_t    r_gyro[3];
    int16_t    r_mag[3];
}imu_raw_meas_t;


//imu scaled measurements 
typedef struct{
    float	    s_accel[3];
    float	    s_gyro[3];
    float       s_mag[3];
}imu_scaled_meas_t;


//imu sample 
typedef struct{
    float	    accel[3];
    float	    gyro[3];
    float       mag[3];
    uint64_t    time_us;
}imu_sample_t;


// calibration params type struct
typedef struct{
    float gyro_bias[3];
    float accel_bias[3];
    float accel_mtx[3][3]; //alrdy inverted matrix!
    float mag_bias[3]; //hard iron
    //soft iron + scale + no-orthogonality 
    float mag_mtx[3][3];  //alrdy inverted matrix!
}imu_calib_params_t;

//imu orientation sample in Euler angles
typedef struct{
    float       roll;
    float       pitch;
    float       yaw;   
    uint64_t    time_us;  
}imu_euler_orient_t;

//imu orientation sample in quaternions
typedef struct{
    float       w;
    float       x;
    float       y;  
    float       z;   
    uint64_t    time_us;  
}imu_quater_orient_t;


#endif

