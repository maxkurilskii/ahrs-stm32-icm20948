#ifndef IMU_ISM_20948_H
#define IMU_ISM_20948_H

#include "common.h"
#include "imu_data_types.h"
#include "spi_driver.h"
#include "uart_driver.h"

//imu ism20948 registers addresses and bite field
#define WHO_AM_I	                0x00
#define REG_BANK_SEL_ADD	        0x7F
#define REG_BANK_SEL_USER_BANK_Pos  0x04
#define REG_BANK_SEL_USER_BANK_MASK (0x02 << REG_BANK_SEL_USER_BANK_Pos)

//USER BANK0:
#define USER_CTRL_ADD               0x03
#define USER_CTRL_I2C_IF_DIS_Pos    0x04
#define USER_CTRL_I2C_MST_EN_Pos    0x05

#define PWR_MGMT_1_ADD	                0x06 
#define PWR_MGMT_1_DEVICE_RESET_Pos	    0x07
#define PWR_MGMT_1_DEVICE_RESET	        (1U << PWR_MGMT_1_DEVICE_RESET_Pos)
#define PWR_MGMT_1_SLEEP_Pos	        0x06
#define PWR_MGMT_1_SLEEP_OFF	        (0U << PWR_MGMT_1_SLEEP_Pos)
#define PWR_MGMT_1_CLKSEL_Pos           0x00
#define PWR_MGMT_1_CLKSEL_Intenal	    (0U << PWR_MGMT_1_CLKSEL_Pos)
#define PWR_MGMT_1_CLKSEL_PLL           (2U << PWR_MGMT_1_CLKSEL_Pos) //1-5

//USER BANK2:
//gyroscope
#define	GYRO_SMPLRT_DIV_ADD	            0x00
#define GYRO_CONFIG_1_ADD	            0x01
#define GYRO_CONFIG_1_GYRO_DLPFCFG_Pos  0x03 
#define GYRO_CONFIG_1_GYRO_FS_SEL_Pos   0x01
#define GYRO_CONFIG_1_GYRO_FCHOICE_Pos  0x00
//special const macroses for gyro full scale selection
#define GYRO_FULL_SCALE_250_DPS	    (0x00 << GYRO_CONFIG_1_GYRO_FS_SEL_Pos)  //sens = 131 lsb/dps
#define GYRO_FULL_SCALE_500_DPS	    (0x01 << GYRO_CONFIG_1_GYRO_FS_SEL_Pos) //sens = 65.5  lsb/dps
#define GYRO_FULL_SCALE_1000_DPS	(0x02 << GYRO_CONFIG_1_GYRO_FS_SEL_Pos) //sens = 32.8 lsb/dps
#define GYRO_FULL_SCALE_2000_DPS	(0x03 << GYRO_CONFIG_1_GYRO_FS_SEL_Pos) //sens = 16.4 lsb/dps

//accelerometer
#define	ACCEL_SMPLRT_DIV_1_ADD	        0x10
#define	ACCEL_SMPLRT_DIV_2_ADD	        0x11
#define ACCEL_CONFIG_ADD	            0x14
#define ACCEL_CONFIG_ACCEL_DLPFCFG_Pos  0x03 
#define ACCEL_CONFIG_ACCEL_FS_SEL_Pos   0x01
#define ACCEL_CONFIG_ACCEL_FCHOICE_Pos  0x00
//special const macroses for accel full scale selection
#define ACCEL_FULL_SCALE_2G	    (0x00 << ACCEL_CONFIG_ACCEL_FS_SEL_Pos) //sens = 16384 lsb/g
#define ACCEL_FULL_SCALE_4G	    (0x01 << ACCEL_CONFIG_ACCEL_FS_SEL_Pos) //sens = 8192 lsb/g
#define ACCEL_FULL_SCALE_8G	    (0x02 << ACCEL_CONFIG_ACCEL_FS_SEL_Pos) //sens = 4096  lsb/g
#define ACCEL_FULL_SCALE_16G	(0x03 << ACCEL_CONFIG_ACCEL_FS_SEL_Pos) //sens = 2048 lsb/g

//auto-incrementing register addresses
#define	ACCEL_XOUT_H_ADD    0x2D  //ACCEL_XOUT_H -> ACCEL_ZOUT_L (6 bytes)
#define GYRO_XOUT_H_ADD     0x33 //GYRO_XOUT_H -> GYRO_ZOUT_L (6 bytes)
#define TEMP_OUT_H_ADD      0x39 //ACCEL_XOUT_H -> ACCEL_ZOUT_L (2 bytes)

//USER BANK3:
//external sensors control over i2c
#define I2C_MST_CTRL_ADD                    0x01
#define I2C_MST_CTRL_I2C_MST_CLK_Pos        0x00
//slave 0 for magnetometer 
#define I2C_SLV0_ADDR                       0x03
#define I2C_SLV0_ADDR_I2C_SLV0_RNW_Pos      0x07
#define I2C_SLV0_ADDR_I2C_ID_0_Pos          0x00
#define I2C_SLV0_REG_ADD                    0x04
#define I2C_SLV0_CTRL_ADD                   0x05
#define I2C_SLV0_CTRL_I2C_SLV0_EN_Pos       0x07
#define I2C_SLV0_CTRL_I2C_SLV0_REG_DIS_Pos  0x05 
#define I2C_SLV0_CTRL_I2C_SLV0_LENG_Pos     0x00
#define I2C_SLV0_DO_ADD 0x06 
#define EXT_SENS_DATA_00_ADD                0x3B

/*magnetometer AK09916 spec*/
#define MAG_I2C_ADD             0x0C //i2c address
#define MAG_DEVICE_ID_ADD       0x01 // DEVICE ID = 0x09
#define MAG_ST1_ADD             0x10 
#define MAG_ST1_DRDY_Pos        0x00 //data ready bit
//status registers
#define MAG_ST1_DOR_Pos         0x01 //data overrun bit
#define MAG_ST2_ADD             0x18 //status2 register address
#define MAG_ST2_HOFL_Pos        0x03 //magnetic field overflow bit
//control registers
#define MAG_CNTL2_ADD           0x31 
#define MAG_CNTL2_MODE_0_Pos    0x00 //single meas mode
#define MAG_CNTL2_MODE_1_Pos    0x01 //cont mode 
#define MAG_CNTL2_MODE_2_Pos    0x02 //cont mode 
#define MAG_CNTL2_MODE_3_Pos    0x03 //cont mode 
#define MAG_CNTL3_ADD           0x32
#define MAG_CNTL3_SRST_Pos      0x00 //soft reset
//measurement registers (lit endian!!@) - can be incremented
#define MAG_HXL_ADD             0x11 //low byte ad for X axis



/* ------------------------------------------------------------- */
#define TIM9_PERIOD_MS          5
#define EXECUTE_CALIB           1 //or 0
#define GYRO_CALIB_MEAS_NUMBER  1000

extern  uint8_t whoAmIValue;

/* initialization */
uint8_t test_imu_startup(void);
void IMU_Timer_Start(void);
void IMU_Timer_Stop(void);
void IMU20948_Init(void);
void calibrate_gyro(void);

/* getter to read ism20948 register value*/
uint8_t get_register_value(uint8_t reg_addr);
    
/* read measurements from imu using dma */
void start_reading_imu_measurement(void);

/* getters of scaled imu measurements from spi buffer */
imu_scaled_meas_t* get_imu_corrected_measurement(void); 
imu_scaled_meas_t* get_imu_scaled_measurement(void);

/* get data from spi buffer in imu_raw_data type*/
imu_raw_meas_t* get_imu_raw_measurement(void);

#endif