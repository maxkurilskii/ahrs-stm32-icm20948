#include "imu_ism20948.h"


uint8_t whoAmIValue = 0;

//buffer to hold raw data
static imu_raw_meas_t raw_meas_buf = {
    .r_accel = {0},
    .r_gyro = {0},
    .r_mag = {0},
};

//buffer to hold scaled data
static imu_scaled_meas_t scaled_meas_buf = {
    .s_accel = {0},
    .s_gyro = {0},
    .s_mag= {0},
};


//calibration params
imu_calib_params_t calib_params = {
    .gyro_bias = {0},
    .accel_bias = {0},
    .accel_mtx = {0},
    .mag_bias = {-35.823315143446436, 12.234985141360744, 10.116295654920302}, 
    
    .mag_mtx = {
            {1.545138, 0.071528, -0.149728},
            {0.071528, 1.429639, 0.032705},
            {-0.149728, 0.032705, 1.605678}}
};

/* Initialization functions */
static void powerup_imu(void);
static void configure_gyro(void);
static void configure_accel(void);
static void configure_magnetometer(void);
static void IMU_Timer_Init(void);

/* Internal proccessing functions */
static void convert_raw_to_scaled_meas(imu_raw_meas_t* r_meas, imu_scaled_meas_t* s_meas);
static imu_scaled_meas_t* correct_imu_scaled_measurement(imu_scaled_meas_t* meas);
    

// ----- "Public" functions* ----



void start_reading_imu_measurement(void){
    /* Read ACCEL_XOUT_H_ADD(6)->GYRO_XOUT_H_ADD(6)->TEMP_OUT_H_ADD(2)->EXT_SLV_SENS_DATA_00(9) */ 
    spi_read_async(ACCEL_XOUT_H_ADD, 23);
    //while(cur_spi_state != SPI_DATA_READY);
}

imu_scaled_meas_t* get_imu_corrected_measurement(void){
    get_imu_raw_measurement();
    convert_raw_to_scaled_meas(&raw_meas_buf, &scaled_meas_buf);
    correct_imu_scaled_measurement(&scaled_meas_buf);
    return &scaled_meas_buf;
}

imu_scaled_meas_t* get_imu_scaled_measurement(void){
    get_imu_raw_measurement();
    convert_raw_to_scaled_meas(&raw_meas_buf, &scaled_meas_buf);
    return &scaled_meas_buf;
}

imu_raw_meas_t* get_imu_raw_measurement(void){
    /*  
    Decode 23 BYTES data from spi data buffer and encode in imu raw data struct:
    accel(3 ax)[0:5] + gyro data(3 ax)[6:11] + temp[12,13] + mag_st1[14] + mag_data(3 ax)[15:20] + junk[21] + mag_st2[22] 
    */
    // Get current data from spi rx buffer 
    uint8_t* imu_resp = get_spi_received_data();
    
    // FOR ACCEL and GYRO <MSB first>: GYRO_X_OUT_H -> GYRO_X_OUT_L 
    raw_meas_buf.r_accel[0] =  (int16_t)((uint16_t)imu_resp[0]	<< 8 | imu_resp[1]);	 
	raw_meas_buf.r_accel[1] =  (int16_t)((uint16_t)imu_resp[2]	<< 8 | imu_resp[3]);
	raw_meas_buf.r_accel[2] =  (int16_t)((uint16_t)imu_resp[4]	<< 8 | imu_resp[5]);
    
	raw_meas_buf.r_gyro[0] = (int16_t)((uint16_t)imu_resp[6]	<< 8 | imu_resp[7]);	
	raw_meas_buf.r_gyro[1] = (int16_t)((uint16_t)imu_resp[8]	<< 8 | imu_resp[9]);	
	raw_meas_buf.r_gyro[2] = (int16_t)((uint16_t)imu_resp[10]	<< 8 | imu_resp[11]);
    
    // MAGNET decoding 
    uint8_t mag_sr1 = imu_resp[14]; 
    //if (mag_sr1 & (1U << MAG_ST1_DOR_Pos)) toggle_led(LED2);
    uint8_t mag_sr2 = imu_resp[22];
    //mag_responce[21] is dummy byte
    
    //check for mag field overflow (data are incorrect?)
    if (mag_sr2 & (1U << MAG_ST2_HOFL_Pos)){
        toggle_led(LED2);
    }
    // LITTLE ENDIAN frm AK09916 <LSB first>: H_X_OUT_L -> H_X_OUT_H
    raw_meas_buf.r_mag[0] = (int16_t)((uint16_t)imu_resp[16]  << 8  | imu_resp[15]); 
    raw_meas_buf.r_mag[1] = (int16_t)((uint16_t)imu_resp[18]  << 8  | imu_resp[17]); 
    raw_meas_buf.r_mag[2] = (int16_t)((uint16_t)imu_resp[20]  << 8  | imu_resp[19]); 
    
    /* Allign magnetometer*/
    // Mag is rotated along X axis over 180 deg with ref to gyro and accel frame
    // Rotation mtx around X: {{1,   0,  0}, {0,  -1,  0}, {0,   0, -1}}
    raw_meas_buf.r_mag[1] = - raw_meas_buf.r_mag[1];
    raw_meas_buf.r_mag[2] = - raw_meas_buf.r_mag[2];   
    
    return &raw_meas_buf;    
}


uint8_t get_register_value(uint8_t reg_addr){
    // Blocking function!
    spi_read_async(reg_addr, 1);
    while (cur_spi_state != SPI_DATA_READY);
    cur_spi_state = SPI_FREE;
    return get_spi_received_data()[0];
}

/* calibration of imu */ 
void calibrate_gyro(void){
    uint16_t meas_cnt = 0;
    //take mean for each axis
    float meas_sum[3] = {0};
    while(meas_cnt < GYRO_CALIB_MEAS_NUMBER){
        //cur_spi_state = SPI_READING; 
        start_reading_imu_measurement(); 
        while(cur_spi_state != SPI_DATA_READY); //blocking until dma sets flag
        //get data from spi buffer and write in local-file raw_meas_buf 
        get_imu_raw_measurement(); 
        convert_raw_to_scaled_meas(&raw_meas_buf, &scaled_meas_buf);
        meas_sum[0] += scaled_meas_buf.s_gyro[0];
        meas_sum[1] += scaled_meas_buf.s_gyro[1];
        meas_sum[2] += scaled_meas_buf.s_gyro[2];
        cur_spi_state = SPI_FREE; 
        meas_cnt++;
    }
    calib_params.gyro_bias[0] = meas_sum[0] / GYRO_CALIB_MEAS_NUMBER;
    calib_params.gyro_bias[1] = meas_sum[1] / GYRO_CALIB_MEAS_NUMBER;
    calib_params.gyro_bias[2] = meas_sum[2] / GYRO_CALIB_MEAS_NUMBER; 
}


void TIM1_BRK_TIM9_IRQHandler(void){
    if (TIM9->SR & TIM_SR_UIF){
        TIM9->SR &= ~TIM_SR_UIF; //clear flag!?
        if (cur_spi_state == SPI_FREE){
            cur_spi_state = SPI_READY;
        }
    }
}


void IMU_Timer_Start(void){
    TIM9->CR1 |= TIM_CR1_CEN;
}

void IMU_Timer_Stop(void){
    TIM9->CR1 &= ~TIM_CR1_CEN;
    TIM10->EGR |= TIM_EGR_UG;
    TIM9->SR &= ~TIM_SR_UIF; //???
    TIM9->CNT = 0;
}

void IMU20948_Init(void){
	/* Configure imu ism20948*/
    SPI1_Init_All();
  
    powerup_imu();
    configure_gyro();
    configure_accel();
    configure_magnetometer();
    // Reset USER BANK reg to default bank (0)
    spi_write_async(REG_BANK_SEL_ADD, (0x00 << REG_BANK_SEL_USER_BANK_Pos));
    transmit_byte_usart3_debug(get_register_value(REG_BANK_SEL_ADD));
    transmit_byte_usart3_debug(get_register_value(PWR_MGMT_1_ADD));
    transmit_byte_usart3_debug(get_register_value(USER_CTRL_ADD));
    transmit_byte_usart3_debug(get_register_value(WHO_AM_I));
    /* Set initial state */
    cur_spi_state = SPI_FREE;
    
    /* INITIALIZE IMU TIMER9*/
    IMU_Timer_Init();

    /* Calibrate unit in "blocking" manner */
    if (EXECUTE_CALIB){
        calibrate_gyro();
        //calibrate_accel(); not impl yet
        //calibrate_mag(); not impl yet    
    }
}

/* ------------------------------------------------------------------------ */




static void convert_raw_to_scaled_meas(imu_raw_meas_t* r_meas, imu_scaled_meas_t* s_meas){
    for(uint8_t i = 0; i < 3; i++){
        //Account accel data SENS for chosen FULL SCALE range (+/- 2g)
        s_meas->s_accel[i] =  r_meas->r_accel[i]	/ 16384.0f; 
        //Account gyro data SENS for chosen FULL SCALE range (+/- 500dps)
        s_meas->s_gyro[i] = r_meas->r_gyro[i] / 65.5f;
        //Account mag data SENS .15 
        s_meas->s_mag[i] = r_meas->r_mag[i] * 0.15; 
    }
}



static imu_scaled_meas_t* correct_imu_scaled_measurement(imu_scaled_meas_t* meas){
    /* GYRO corection: meas_corrected = meas - bias*/
    meas->s_gyro[0] -= calib_params.gyro_bias[0];
    meas->s_gyro[1] -= calib_params.gyro_bias[1];
    meas->s_gyro[2] -= calib_params.gyro_bias[2];
    
    //ACCEL - no correction yet
    
    /* MAG correction: meas_corrected = (meas - bias) x A_inv */
    float* mag_meas = meas->s_mag; //short form
    mag_meas[0]  -= calib_params.mag_bias[0];
    mag_meas[1]  -= calib_params.mag_bias[1];
    mag_meas[2]  -= calib_params.mag_bias[2];

    float mag0  = mag_meas[0]*calib_params.mag_mtx[0][0]  + mag_meas[1]*calib_params.mag_mtx[1][0] + mag_meas[2]*calib_params.mag_mtx[2][0];
    float mag1  = mag_meas[0]*calib_params.mag_mtx[0][1]  + mag_meas[1]*calib_params.mag_mtx[1][1] + mag_meas[2]*calib_params.mag_mtx[2][1];
    float mag2  = mag_meas[0]*calib_params.mag_mtx[0][2]  + mag_meas[1]*calib_params.mag_mtx[1][2] + mag_meas[2]*calib_params.mag_mtx[2][2];
        
    mag_meas[0] = mag0;
    mag_meas[1] = mag1;
    mag_meas[2] = mag2; 
    
    return meas;
}



static void IMU_Timer_Init(void){
    //Enable tim9(16bit timer) clock from APB2 (108 MHz)
    RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
    
    //Prescaler = 107: 108 Mhz / (107 + 1) = 1 Mhz (1us per tick)
    TIM9->PSC = 107; 
    //ARR is (TIM9_PERIOD_MS / 1000) * Fcnt_tick 
    TIM9->ARR = TIM9_PERIOD_MS * 1000; 
    
    //Update Prescaler and ARR registers before start
    TIM9->EGR |= TIM_EGR_UG;
    TIM9->SR &= ~TIM_SR_UIF; //cleat flag!
    
    //Enable interrupts
    TIM9->DIER |= TIM_DIER_UIE;
    NVIC_EnableIRQ(TIM1_BRK_TIM9_IRQn);
    
    //DO NOT Start timer in INIT!
    //TIM9->CR1 |= TIM_CR1_CEN;
}



static void powerup_imu(void){ 
    uint8_t reg_value = 0;//var to verify written data 
	//reset all registers to def state
    spi_write_async(PWR_MGMT_1_ADD, PWR_MGMT_1_DEVICE_RESET);
    delay_ms(200); //NECCESSARY TO WAIT pin timout read/write !!!
    
    // Choose Bank 0 
    spi_write_async(REG_BANK_SEL_ADD, (0x00 << REG_BANK_SEL_USER_BANK_Pos));   
    transmit_byte_usart3(get_register_value(REG_BANK_SEL_ADD)); // ожидаем 0x00

    // Disable sleep mode + select clock PLL to run gyro in best performance
    spi_write_async(PWR_MGMT_1_ADD, (PWR_MGMT_1_SLEEP_OFF | PWR_MGMT_1_CLKSEL_PLL));
    delay_ms(1);
    // Choose SPI mode only (immediately after restart) + enable i2c master
	spi_write_async(USER_CTRL_ADD, 
                           (1U << USER_CTRL_I2C_IF_DIS_Pos) | 
                           (1U << USER_CTRL_I2C_MST_EN_Pos));                       
    transmit_byte_usart3(get_register_value(PWR_MGMT_1_ADD)); //ожидаем 0x01 | 0x02 (if 0x00 - gyro won't work correctly)
    transmit_byte_usart3(get_register_value(USER_CTRL_ADD));// ожидаем 0x30
}


static void configure_gyro(void){
    /* ---- Gyroscope config ---- */
    uint8_t reg_value = 0; //var to verify written data 

    // Choose user bank to 2 (new register table)
    spi_write_async(REG_BANK_SEL_ADD, (0x02 << REG_BANK_SEL_USER_BANK_Pos));
    transmit_byte_usart3(get_register_value(REG_BANK_SEL_ADD)); // exp 0x20
   
	// Select def ODR freq = 1.1 kHz -> GYRO_SMPLRT_DIV = 0 (default)
	//spi_write(GYRO_SMPLRT_DIV_ADD, 0x00);
	
	// Enable gyro DLPF(FCHOICE = 1) + and NBW(noise bandwidth) = 73.3 (3) 
    uint8_t gyro_cfg1 = 0;
    gyro_cfg1 = (1U << GYRO_CONFIG_1_GYRO_FCHOICE_Pos) | (3U << GYRO_CONFIG_1_GYRO_DLPFCFG_Pos);
    //Gyro full scale +/-500 dps
    gyro_cfg1 |= GYRO_FULL_SCALE_500_DPS;
    spi_write_async(GYRO_CONFIG_1_ADD, gyro_cfg1);
    transmit_byte_usart3(get_register_value(GYRO_CONFIG_1_ADD));  // exp 0x1B
}



static void configure_accel(void){
    /* ---- Accelerometer config ---- */
    uint8_t reg_value = 0; //var to verify written data 
    
    // Change user bank to 2 (new register table)
    spi_write_async(REG_BANK_SEL_ADD, (0x02 << REG_BANK_SEL_USER_BANK_Pos));
    transmit_byte_usart3(get_register_value(REG_BANK_SEL_ADD)); // exp 0x20

    
	//Select def ODR freq = 1.125 kHz -> ACCEL_SMPLRT_DIV[0:1] = 0 (default)
    spi_write_async(ACCEL_SMPLRT_DIV_1_ADD, 0x00);

	//Enable accel DLPF(FCHOICE = 1) + and NBW(noise bandwidth) = 68.8  (3) 
	uint8_t acc_cfg = 0;
	acc_cfg = (1U << ACCEL_CONFIG_ACCEL_FCHOICE_Pos) | (3U << ACCEL_CONFIG_ACCEL_DLPFCFG_Pos); 
	//Acc full scale +/-2g
	acc_cfg |= ACCEL_FULL_SCALE_2G; //0x25
	spi_write_async(ACCEL_CONFIG_ADD, acc_cfg);
    transmit_byte_usart3(get_register_value(ACCEL_CONFIG_ADD)); // exp 0x19
}



static void configure_magnetometer(void){
    /* Configure external magnetometer over I2C on GY_ICM20948V2 board (master mode) */
    uint8_t reg_value = 0; //var to verify written data 
    
    // Set user bank 3
    spi_write_async(REG_BANK_SEL_ADD, (0x03 << REG_BANK_SEL_USER_BANK_Pos));
    transmit_byte_usart3(get_register_value(REG_BANK_SEL_ADD)); // exp 0x30
    
    //ODR settings: when gyro is active -> external odr is the same 
    //i2c master freq: according to recommended in table 23 is 345.60 kHz(46.67%)
    spi_write_async(I2C_MST_CTRL_ADD, (7U << I2C_MST_CTRL_I2C_MST_CLK_Pos));
    
     /* -------- Magnetometer internal setting configuration ---------------- */
    //set transfer for write +  mag i2c address  
    spi_write_async(I2C_SLV0_ADDR, 
                            (0 << I2C_SLV0_ADDR_I2C_SLV0_RNW_Pos) |
                            (MAG_I2C_ADD << I2C_SLV0_ADDR_I2C_ID_0_Pos));
    
    //Reset magnitometer (continous mode 4)
    spi_write_async(I2C_SLV0_REG_ADD, MAG_CNTL3_ADD); //select add of reg to write data to
    spi_write_async(I2C_SLV0_DO_ADD, (1 << MAG_CNTL3_SRST_Pos)); //reset magnetometer
    //tranfer len = 1 byte + enable i2c write
    spi_write_async(I2C_SLV0_CTRL_ADD, 
                            (1U << I2C_SLV0_CTRL_I2C_SLV0_LENG_Pos)|
                            (1U << I2C_SLV0_CTRL_I2C_SLV0_EN_Pos));
    delay_ms(1);
    spi_write_async(I2C_SLV0_CTRL_ADD, (0U << I2C_SLV0_CTRL_I2C_SLV0_EN_Pos));
    
    // Set mag freq to 100 Hz (continous mode 4)
    spi_write_async(I2C_SLV0_REG_ADD, MAG_CNTL2_ADD);
    spi_write_async(I2C_SLV0_DO_ADD, (1 << MAG_CNTL2_MODE_3_Pos)); 
    spi_write_async(I2C_SLV0_CTRL_ADD, 
                            (1U << I2C_SLV0_CTRL_I2C_SLV0_LENG_Pos)|
                            (1U << I2C_SLV0_CTRL_I2C_SLV0_EN_Pos));
    delay_ms(1);
    spi_write_async(I2C_SLV0_CTRL_ADD, (0U << I2C_SLV0_CTRL_I2C_SLV0_EN_Pos));
    /* --------------------------------------------------------------------- */
     
    //set transfer for read +  mag i2c address  
    spi_write_async(I2C_SLV0_ADDR, 
                            (1U << I2C_SLV0_ADDR_I2C_SLV0_RNW_Pos) |
                            (MAG_I2C_ADD << I2C_SLV0_ADDR_I2C_ID_0_Pos));
                          
    //set init address of data transfer from ST1 -> HXL_OUT -> ... ST2 (autoinc)
    spi_write_async(I2C_SLV0_REG_ADD, MAG_ST1_ADD);
    transmit_byte_usart3(get_register_value(I2C_SLV0_REG_ADD)); // exp 0x10 (MAG_ST1_ADD)
    
    //set length of read data (9 bytes): necessarry to read st2 reg when receiving measurements 
    uint8_t slave0_cntrl_data = (9U << I2C_SLV0_CTRL_I2C_SLV0_LENG_Pos);
    //enable reading for slave0 (magnetometer) -> data are stored in EXT_SENS_DATA_00 -> EXT_SENS_DATA_8
    slave0_cntrl_data |=  (1U << I2C_SLV0_CTRL_I2C_SLV0_EN_Pos);
    spi_write_async(I2C_SLV0_CTRL_ADD, slave0_cntrl_data);
}
