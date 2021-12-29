/**
  ************************(C) COPYRIGHT 2021 UNNC LANCET************************
  * @file       INS_task.c/h
  * @brief      provide INS task and pointer request functions, based on
  *             bsp_imu.c/h library
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V2.0.0     Feb-26-2021     YW              1. done
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ************************(C) COPYRIGHT 2021 UNNC LANCET************************
  */

#include "INS_task.h"
#include "cmsis_os.h"
//#include "detect_task.h"
#include "ist8310_reg.h"
#include "math.h"
#include "mpu6500_reg.h"
#include "user_lib.h"
#include "spi.h"
#include "struct_typedef.h"

extern SPI_HandleTypeDef hspi1;

#define MPU_HSPI hspi1
#define MPU_NSS_LOW HAL_GPIO_WritePin(MPU_SPI_NSS_GPIO_Port, MPU_SPI_NSS_Pin, GPIO_PIN_RESET)
#define MPU_NSS_HIGH HAL_GPIO_WritePin(MPU_SPI_NSS_GPIO_Port, MPU_SPI_NSS_Pin, GPIO_PIN_SET)
#define MPU_DELAY(x) osDelay(x)

#define IMU_KP 2.0f                                          /*
                                                              * proportional gain governs rate of 
                                                              * convergence to accelerometer/magnetometer 
															                                */
#define IMU_KI 0.01f                                         /*
                                                              * integral gain governs rate of 
                                                              * convergence of gyroscope biases 
                                                              */

static uint8_t mpu_write_byte(const uint8_t reg, const uint8_t data);
static uint8_t mpu_read_byte(const uint8_t reg);
static uint8_t mpu_read_bytes(const uint8_t regAddr, uint8_t* pData, uint8_t len);
static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data);
static uint8_t ist_reg_read_by_mpu(uint8_t addr);
static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num);
static uint8_t ist8310_init(void);
static uint8_t ist8310_get_data(uint8_t* buff);
static void mpu_get_data(void);
static uint8_t mpu_set_gyro_fsr(uint8_t fsr);
static uint8_t mpu_set_accel_fsr(uint8_t fsr);
static uint8_t mpu_device_init(void);
static void mpu_offset_call(void);
static void init_quaternion(void);
static void imu_ahrs_update(void);
static void imu_attitude_update(void);

static volatile float32_t q0 = 1.0f;
static volatile float32_t q1 = 0.0f;
static volatile float32_t q2 = 0.0f;
static volatile float32_t q3 = 0.0f;
static volatile float32_t exInt, eyInt, ezInt;                      /* error integral */
static volatile float32_t gx, gy, gz, ax, ay, az, mx, my, mz;
static volatile uint32_t  last_update, now_update;                  /* Sampling cycle count, ubit ms */
static uint8_t tx, rx;
static uint8_t tx_buff[14] = { 0xff };
static uint8_t mpu_buff[14];                                        /* buffer to save imu raw data */
static uint8_t ist_buff[6];                                         /* buffer to save IST8310 raw data */
mpu_data_t mpu_data;
imu_t imu={0};

//final output used in other place
static float32_t INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static float32_t INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit: rad

/**
  * @brief  write a byte of data to specified register
  * @param  reg:  the address of register to be written
  *         data: data to be written
  * @retval 
  * @usage  call in ist_reg_write_by_mpu(),         
  *                 ist_reg_read_by_mpu(), 
  *                 mpu_master_i2c_auto_read_config(), 
  *                 ist8310_init(), 
  *                 mpu_set_gyro_fsr(),             
  *                 mpu_set_accel_fsr(), 
  *                 mpu_device_init() functions
  */
static uint8_t mpu_write_byte(const uint8_t reg, const uint8_t data)
{
    MPU_NSS_LOW;
    tx = reg & 0x7F;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    tx = data;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    MPU_NSS_HIGH;
    return 0;
}

/**
  * @brief  read a byte of data from specified register
  * @param  reg: the address of register to be read
  * @retval 
  * @usage  call in ist_reg_read_by_mpu(),         
  *                 mpu_device_init() function
  */
static uint8_t mpu_read_byte(const uint8_t reg)
{
    MPU_NSS_LOW;
    tx = reg | 0x80;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    MPU_NSS_HIGH;
    return rx;
}

/**
  * @brief  read bytes of data from specified register
  * @param  reg: address from where data is to be written
  * @retval error code
  * @usage  call in ist8310_get_data(),         
  *                 mpu_get_data(), 
  *                 mpu_offset_call() functions
  */
static uint8_t mpu_read_bytes(const uint8_t regAddr, uint8_t* pData, uint8_t len)
{
    bool_t errorcode = 0;
    MPU_NSS_LOW;
    tx         = regAddr | 0x80;
    tx_buff[0] = tx;
    errorcode = HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    errorcode = HAL_SPI_TransmitReceive(&MPU_HSPI, tx_buff, pData, len, 55);
    MPU_NSS_HIGH;

    return errorcode;
}

/**
  * @brief  write IST8310 register through MPU6500's I2C master
  * @param  addr: the address to be written of IST8310's register
  *         data: data to be written
  * @retval
  * @usage  call in ist8310_init() function
  */
static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
{
    /* turn off slave 1 at first */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, addr);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, data);
    MPU_DELAY(2);
    /* turn on slave 1 with one byte transmitting */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    /* wait longer to ensure the data is transmitted from slave 1 */
    MPU_DELAY(10);
}

/**
  * @brief  write IST8310 register through MPU6500's I2C Master
  * @param  addr: the address to be read of IST8310's register
  * @retval
  * @usage  call in ist8310_init() function
  */
static uint8_t ist_reg_read_by_mpu(uint8_t addr)
{
    uint8_t retval;
    mpu_write_byte(MPU6500_I2C_SLV4_REG, addr);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x80);
    MPU_DELAY(10);
    retval = mpu_read_byte(MPU6500_I2C_SLV4_DI);
    /* turn off slave4 after read */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);
    return retval;
}

/**
  * @brief    initialize the MPU6500 I2C Slave 0 for I2C reading.
  * @param    device_address: slave device address, Address[6:0]
  * @retval   void
  * @note
  */
static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    /* 
	 * configure the device address of the IST8310
     * use slave1, auto transmit single measure mode 
	 */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
    MPU_DELAY(2);

    /* use slave0,auto read data */
    mpu_write_byte(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV0_REG, reg_base_addr);
    MPU_DELAY(2);

    /* every eight mpu6500 internal samples one i2c master read */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x03);
    MPU_DELAY(2);
    /* enable slave 0 and 1 access delay */
    mpu_write_byte(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    MPU_DELAY(2);
    /* enable slave 1 auto transmit */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
	/* Wait 6ms (minimum waiting time for 16 times internal average setup) */
    MPU_DELAY(6); 
    /* enable slave 0 with data_num bytes reading */
    mpu_write_byte(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
    MPU_DELAY(2);
}

/**
  * @brief  Initializes the IST8310 device
  * @param
  * @retval
  * @usage  call in mpu_device_init() function
  */
static uint8_t ist8310_init(void)
{
	/* enable iic master mode */
    mpu_write_byte(MPU6500_USER_CTRL, 0x30);
    MPU_DELAY(10);
	/* enable iic 400khz */
    mpu_write_byte(MPU6500_I2C_MST_CTRL, 0x0d); 
    MPU_DELAY(10);

    /* turn on slave 1 for ist write and slave 4 to ist read */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);  
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
    MPU_DELAY(10);

    /* IST8310_R_CONFB 0x01 = device rst */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
    MPU_DELAY(10);
    if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
    {
        return 1;
    }

	/* soft reset */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01); 
    MPU_DELAY(10);

	/* config as ready mode to access register */
    ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00); 
    if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
    {
        return 2;
    }
    MPU_DELAY(10);

	/* normal state, no int */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00);
    if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
    {
        return 3;
    }
    MPU_DELAY(10);
		
    /* config low noise mode, x,y,z axis 16 time 1 avg */
    ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24); //100100
    if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
    {
        return 4;
    }
    MPU_DELAY(10);

    /* Set/Reset pulse duration setup,normal mode */
    ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
    if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
    {
        return 5;
    }
    MPU_DELAY(10);

    /* turn off slave1 & slave 4 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);

    /* configure and turn on slave 0 */
    mpu_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    MPU_DELAY(100);
    return 0;
}

/**
  * @brief  get the data of IST8310
  * @param  buff: the buffer to save the data of IST8310
  * @retval
  * @usage  call in mpu_get_data() function
  */
static uint8_t ist8310_get_data(uint8_t* buff)
{
    return mpu_read_bytes(MPU6500_EXT_SENS_DATA_00, buff, 6);
}


/**
  * @brief  get the data of imu
  * @param  
  * @retval
  * @usage  call in INS_task() function
  */
static void mpu_get_data(void)
{
    static bool_t mpu_error_code = 0;
    static bool_t ist8310_error_code = 0;

    mpu_error_code = mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);
    if(mpu_error_code == HAL_OK)
    {
        //detect_hook(BOARD_MPU6500_TOE);
    }

    mpu_data.ax   = mpu_buff[0] << 8 | mpu_buff[1];
    mpu_data.ay   = mpu_buff[2] << 8 | mpu_buff[3];
    mpu_data.az   = mpu_buff[4] << 8 | mpu_buff[5];
    mpu_data.temp = mpu_buff[6] << 8 | mpu_buff[7];

    mpu_data.gx = ((mpu_buff[8]  << 8 | mpu_buff[9])  - mpu_data.gx_offset);
    mpu_data.gy = ((mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset);
    mpu_data.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset);

    ist8310_error_code = ist8310_get_data(ist_buff);
    if(ist8310_error_code == HAL_OK)
    {
        //detect_hook(BOARD_IST8310_TOE);
    }

    memcpy(&mpu_data.mx, ist_buff, 6);

    memcpy(&imu.ax, &mpu_data.ax, 6 * sizeof(int16_t));
	
    imu.temp = 21 + mpu_data.temp / 333.87f;
	/* 2000dps -> rad/s */
	imu.wx   = mpu_data.gx / 16.384f / 57.3f;
    imu.wy   = mpu_data.gy / 16.384f / 57.3f; 
    imu.wz   = mpu_data.gz / 16.384f / 57.3f;
}


/**
  * @brief  set imu 6500 gyroscope measure range
  * @param  fsr: range(0,±250dps;1,±500dps;2,±1000dps;3,±2000dps)
  * @retval
  * @usage  call in mpu_device_init() function
  */
static uint8_t mpu_set_gyro_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_GYRO_CONFIG, fsr << 3);
}


/**
  * @brief  set imu 6050/6500 accelerate measure range
  * @param  fsr: range(0,±2g;1,±4g;2,±8g;3,±16g)
  * @retval
  * @usage  call in mpu_device_init() function
  */
static uint8_t mpu_set_accel_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_ACCEL_CONFIG, fsr << 3); 
}

static uint8_t id;

/**
  * @brief  initialize imu mpu6500 and magnet meter ist3810
  * @param  
  * @retval
  * @usage  call in INS_task() function
  */
static uint8_t mpu_device_init(void)
{
	MPU_DELAY(100);

	id                               = mpu_read_byte(MPU6500_WHO_AM_I);
	uint8_t i                        = 0;
	uint8_t MPU6500_Init_Data[10][2] = {{ MPU6500_PWR_MGMT_1, 0x80 },     /* Reset Device */ 
										{ MPU6500_PWR_MGMT_1, 0x03 },     /* Clock Source - Gyro-Z */
										{ MPU6500_PWR_MGMT_2, 0x00 },     /* Enable Acc & Gyro */
										{ MPU6500_CONFIG, 0x04 },         /* LPF 41Hz */
										{ MPU6500_GYRO_CONFIG, 0x18 },    /* +-2000dps */
										{ MPU6500_ACCEL_CONFIG, 0x10 },   /* +-8G */
										{ MPU6500_ACCEL_CONFIG_2, 0x02 }, /* enable LowPassFilter  Set Acc LPF */
										{ MPU6500_USER_CTRL, 0x20 },};    /* Enable AUX */
	for (i = 0; i < 10; i++)
	{
		mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
		MPU_DELAY(1);
	}

	mpu_set_gyro_fsr(3); 		
	mpu_set_accel_fsr(2);

	ist8310_init();
	mpu_offset_call();
	return 0;
}

/**
  * @brief  get the offset data of MPU6500
  * @param  
  * @retval
  * @usage  call in mpu_device_init() function
  */
static void mpu_offset_call(void)
{
	int i;
	for (i=0; i<300;i++)
	{
		mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

		mpu_data.ax_offset += mpu_buff[0] << 8 | mpu_buff[1];
		mpu_data.ay_offset += mpu_buff[2] << 8 | mpu_buff[3];
		mpu_data.az_offset += mpu_buff[4] << 8 | mpu_buff[5];
	
		mpu_data.gx_offset += mpu_buff[8]  << 8 | mpu_buff[9];
		mpu_data.gy_offset += mpu_buff[10] << 8 | mpu_buff[11];
		mpu_data.gz_offset += mpu_buff[12] << 8 | mpu_buff[13];

		MPU_DELAY(5);
	}
	mpu_data.ax_offset=mpu_data.ax_offset / 300;
	mpu_data.ay_offset=mpu_data.ay_offset / 300;
	mpu_data.az_offset=mpu_data.az_offset / 300;
	mpu_data.gx_offset=mpu_data.gx_offset / 300;
	mpu_data.gy_offset=mpu_data.gx_offset / 300;
	mpu_data.gz_offset=mpu_data.gz_offset / 300;
}



/**
  * @brief  Initialize quaternion
  * @param  
  * @retval
  * @usage  call in INS_task() function
  */
static void init_quaternion(void)
{
	int16_t hx, hy;//hz;
	
	hx = imu.mx;
	hy = imu.my;
	//hz = imu.mz;
	
	#ifdef BOARD_DOWN
	if (hx < 0 && hy < 0) 
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = -0.005;
			q1 = -0.199;
			q2 = 0.979;
			q3 = -0.0089;
		}
		else
		{
			q0 = -0.008;
			q1 = -0.555;
			q2 = 0.83;
			q3 = -0.002;
		}
		
	}
	else if (hx < 0 && hy > 0)
	{
		if (fabs(hx / hy)>=1)   
		{
			q0 = 0.005;
			q1 = -0.199;
			q2 = -0.978;
			q3 = 0.012;
		}
		else
		{
			q0 = 0.005;
			q1 = -0.553;
			q2 = -0.83;
			q3 = -0.0023;
		}
		
	}
	else if (hx > 0 && hy > 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.0012;
			q1 = -0.978;
			q2 = -0.199;
			q3 = -0.005;
		}
		else
		{
			q0 = 0.0023;
			q1 = -0.83;
			q2 = -0.553;
			q3 = 0.0023;
		}
		
	}
	else if (hx > 0 && hy < 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.0025;
			q1 = 0.978;
			q2 = -0.199;
			q3 = 0.008;			
		}
		else
		{
			q0 = 0.0025;
			q1 = 0.83;
			q2 = -0.56;
			q3 = 0.0045;
		}		
	}
	#else
		if (hx < 0 && hy < 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.195;
			q1 = -0.015;
			q2 = 0.0043;
			q3 = 0.979;
		}
		else
		{
			q0 = 0.555;
			q1 = -0.015;
			q2 = 0.006;
			q3 = 0.829;
		}
		
	}
	else if (hx < 0 && hy > 0)
	{
		if(fabs(hx / hy) >= 1)
		{
			q0 = -0.193;
			q1 = -0.009;
			q2 = -0.006;
			q3 = 0.979;
		}
		else
		{
			q0 = -0.552;
			q1 = -0.0048;
			q2 = -0.0115;
			q3 = 0.8313;
		}
		
	}
	else if (hx > 0 && hy > 0)
	{
		if(fabs(hx / hy) >= 1)
		{
			q0 = -0.9785;
			q1 = 0.008;
			q2 = -0.02;
			q3 = 0.195;
		}
		else
		{
			q0 = -0.9828;
			q1 = 0.002;
			q2 = -0.0167;
			q3 = 0.5557;
		}
		
	}
	else if (hx > 0 && hy < 0)
	{
		if(fabs(hx / hy) >= 1)
		{
			q0 = -0.979;
			q1 = 0.0116;
			q2 = -0.0167;
			q3 = -0.195;			
		}
		else
		{
			q0 = -0.83;
			q1 = 0.014;
			q2 = -0.012;
			q3 = -0.556;
		}		
	}
	#endif
}

/**
  * @brief  update imu AHRS
  * @param  
  * @retval
  * @usage  call in INS_task() function
  */
static void imu_ahrs_update(void)
{
    float32_t norm;
    float32_t hx, hy, hz, bx, bz;
    float32_t vx, vy, vz, wx, wy, wz;
    float32_t ex, ey, ez, halfT;
    float32_t tempq0,tempq1,tempq2,tempq3;

    float32_t q0q0 = q0*q0;
    float32_t q0q1 = q0*q1;
    float32_t q0q2 = q0*q2;
    float32_t q0q3 = q0*q3;
    float32_t q1q1 = q1*q1;
    float32_t q1q2 = q1*q2;
    float32_t q1q3 = q1*q3;
    float32_t q2q2 = q2*q2;
    float32_t q2q3 = q2*q3;
    float32_t q3q3 = q3*q3;

	gx = imu.wx;
	gy = imu.wy;
	gz = imu.wz;
	ax = imu.ax;
	ay = imu.ay;
	az = imu.az;
	mx = imu.mx;
	my = imu.my;
	mz = imu.mz;

	now_update  = HAL_GetTick(); //ms
	halfT       = ((float32_t)(now_update - last_update) / 2000.0f);
	last_update = now_update;
	
	/* Fast inverse square-root */
	norm = invSqrt(ax * ax + ay * ay + az * az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	
	#ifdef IST8310
		norm = invSqrt(mx * mx + my * my + mz * mz);
		mx = mx * norm;
		my = my * norm;
		mz = mz * norm; 
	#else
		mx = 0;
		my = 0;
		mz = 0;		
	#endif
	/* compute reference direction of flux */
	hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);
	hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);
	hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
	bx = sqrt((hx * hx) + (hy * hy));
	bz = hz; 
	
	/* estimated direction of gravity and flux (v and w) */
	vx = 2.0f * (q1q3 - q0q2);
	vy = 2.0f * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
	wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
	wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);
	
	/* 
	 * error is sum of cross product between reference direction 
	 * of fields and direction measured by sensors 
	 */
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

	/* PI */
	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		exInt = exInt + ex * IMU_KI * halfT;
		eyInt = eyInt + ey * IMU_KI * halfT;
		ezInt = ezInt + ez * IMU_KI * halfT;
		
		gx = gx + IMU_KP * ex + exInt;
		gy = gy + IMU_KP * ey + eyInt;
		gz = gz + IMU_KP * ez + ezInt;
	}
	
	tempq0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
	tempq1 = q1 + (q0*gx + q2*gz - q3*gy) * halfT;
	tempq2 = q2 + (q0*gy - q1*gz + q3*gx) * halfT;
	tempq3 = q3 + (q0*gz + q1*gy - q2*gx) * halfT;  

	/* normalise quaternion */
	norm = invSqrt(tempq0 * tempq0 + tempq1 * tempq1 + tempq2 * tempq2 + tempq3 * tempq3);
	q0 = tempq0 * norm;
	q1 = tempq1 * norm;
	q2 = tempq2 * norm;
	q3 = tempq3 * norm;
}

/**
  * @brief  update imu attitude
  * @param  
  * @retval
  * @usage  call in INS_task() function
  */
static void imu_attitude_update(void)
{
    //the output is in unit in radius
    //if you need unit in degree, please multiple all these three with 57.3
	/* yaw    -pi----pi */
	imu.yaw = -atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1);
	/* pitch  -pi/2----pi/2 */
	imu.pit = -asin(-2*q1*q3 + 2*q0*q2);
	/* roll   -pi----pi  */	
	imu.rol =  atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1);
}

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t INS_high_water;
#endif

/**
  * @brief          imu task, init mpu6500, ist8310, calculate the euler angle
  * @param[in]      pvParameters: NULL
  * @retval         none
  */
void INS_task(void *pvParameters)
{
    //wait a time
    osDelay(INS_TASK_INIT_TIME);

    mpu_device_init();
    init_quaternion();
    osDelay(100);

    while (1)
    {
        //update value
        mpu_get_data();
        imu_ahrs_update();
        imu_attitude_update();

        //assign value
        INS_angle[INS_YAW_ADDRESS_OFFSET] = imu.yaw;
        INS_angle[INS_PITCH_ADDRESS_OFFSET] = imu.pit;
        INS_angle[INS_ROLL_ADDRESS_OFFSET] = imu.rol;
        INS_gyro[INS_GYRO_X_ADDRESS_OFFSET] = gx;
        INS_gyro[INS_GYRO_Y_ADDRESS_OFFSET] = gy;
        INS_gyro[INS_GYRO_Z_ADDRESS_OFFSET] = gz;

#if INCLUDE_uxTaskGetStackHighWaterMark
        INS_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
  * @brief          get the euler angle, 0:yaw, 1:pitch, 2:roll, unit: rad
  * @param[in]      none
  * @retval         the pointer of INS_angle
  */
const float32_t get_INS_angle_point(void)
{
//    return INS_angle;
	return INS_angle[0];
}

/**
  * @brief          get the rotation speed, 0:x-axis, 1:y-axis, 2:roll-axis, unit: rad/s
  * @param[in]      none
  * @retval         the pointer of INS_gyro
  */
const float32_t * get_gyro_data_point(void)
{
    return INS_gyro;
}

