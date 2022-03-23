/*
* icm20948.c
*
*  Created on: Dec 26, 2020
*      Author: mokhwasomssi
*/

#include "cmsis_os.h"
#include "icm20948.h"

static float gyro_scale_factor;
static float accel_scale_factor;

void i2c_bypass(I2C_HandleTypeDef *hi2c1){
	write_single_icm20948_reg(hi2c1,ub_0, B0_INT_PIN_CFG, 0x02);
}

void select_user_bank(I2C_HandleTypeDef* hi2c1, userbank ub)
{
	uint8_t write_reg[2];
	write_reg[0] = WRITE | REG_BANK_SEL;
	write_reg[1] = ub;

	HAL_I2C_Master_Transmit(hi2c1,SLAVE_ADDR,write_reg,2,10);
	HAL_Delay(2);
	//HAL_SPI_Transmit(ICM20948_SPI, write_reg, 2, 10);

}

uint8_t read_single_icm20948_reg(I2C_HandleTypeDef* hi2c1,userbank ub, uint8_t reg)
{
	uint8_t read_reg = READ | reg;
	uint8_t reg_val;
	select_user_bank(hi2c1,ub);

	HAL_I2C_Master_Transmit(hi2c1,SLAVE_ADDR,&read_reg,1,1000);
	HAL_Delay(2);
	HAL_I2C_Master_Receive(hi2c1,SLAVE_ADDR,&reg_val,1,1000);
	HAL_Delay(2);

	return reg_val;
}

uint8_t* read_multiple_icm20948_reg(I2C_HandleTypeDef* hi2c1,userbank ub, uint8_t reg, uint8_t len)
{
	uint8_t read_reg = READ | reg;
	static uint8_t reg_val[6];
	select_user_bank(hi2c1,ub);


	HAL_I2C_Master_Transmit(hi2c1,SLAVE_ADDR,&read_reg,1,1000);
	osDelay(2);
	HAL_I2C_Master_Receive(hi2c1,SLAVE_ADDR,reg_val,len,1000);
	osDelay(2);
	return reg_val;
}

void write_single_icm20948_reg(I2C_HandleTypeDef* hi2c1,userbank ub, uint8_t reg, uint8_t val)
{
	uint8_t write_reg[2];
	write_reg[0] = WRITE | reg;
	write_reg[1] = val;

	select_user_bank(hi2c1,ub);

	HAL_I2C_Master_Transmit(hi2c1,SLAVE_ADDR,write_reg,2,1000);
	osDelay(2);
}

void write_multiple_icm20948_reg(I2C_HandleTypeDef* hi2c1,userbank ub, uint8_t reg, uint8_t* val, uint8_t len)
{
	uint8_t write_reg = WRITE | reg;
	select_user_bank(hi2c1,ub);
	HAL_I2C_Master_Transmit(hi2c1,SLAVE_ADDR,&write_reg,1,1000);
	osDelay(2);
	HAL_I2C_Master_Transmit(hi2c1,SLAVE_ADDR,val,len,1000);
	osDelay(2);
}

uint8_t read_single_ak09916_reg(I2C_HandleTypeDef* hi2c1,uint8_t reg)
{

	write_single_icm20948_reg(hi2c1,ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
	write_single_icm20948_reg(hi2c1,ub_3, B3_I2C_SLV0_REG, reg);
	write_single_icm20948_reg(hi2c1,ub_3, B3_I2C_SLV0_CTRL, 0x81);

	return read_single_icm20948_reg(hi2c1,ub_0, B0_EXT_SLV_SENS_DATA_00);
}

uint8_t* read_multiple_ak09916_reg(I2C_HandleTypeDef* hi2c1,uint8_t reg, uint8_t len)
{

	write_single_icm20948_reg(hi2c1,ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
	write_single_icm20948_reg(hi2c1,ub_3, B3_I2C_SLV0_REG, reg);
	write_single_icm20948_reg(hi2c1,ub_3, B3_I2C_SLV0_CTRL, 0x80 | len);

	return read_multiple_icm20948_reg(hi2c1,ub_0, B0_EXT_SLV_SENS_DATA_00, len);
}

void write_single_ak09916_reg(I2C_HandleTypeDef* hi2c1,uint8_t reg, uint8_t val)
{

	write_single_icm20948_reg(hi2c1,ub_3, B3_I2C_SLV0_ADDR, WRITE | MAG_SLAVE_ADDR);
	write_single_icm20948_reg(hi2c1,ub_3, B3_I2C_SLV0_REG, reg);
	write_single_icm20948_reg(hi2c1,ub_3, B3_I2C_SLV0_DO, val);
	write_single_icm20948_reg(hi2c1,ub_3, B3_I2C_SLV0_CTRL, 0x81);
}




void icm20948_init(I2C_HandleTypeDef *hi2c1)
{


	while(!icm20948_who_am_i(hi2c1));
	icm20948_device_reset(hi2c1);
	osDelay(5); //tf need reset time
	icm20948_wakeup(hi2c1);

	icm20948_clock_source(hi2c1,1);
	icm20948_odr_align_enable(hi2c1);

	//icm20948_spi_slave_enable(hi2c1);

	icm20948_gyro_low_pass_filter(hi2c1,1);
	icm20948_accel_low_pass_filter(hi2c1,2);
//
	icm20948_gyro_sample_rate_divider(hi2c1,0);
	icm20948_accel_sample_rate_divider(hi2c1,0);
//
	icm20948_gyro_calibration(hi2c1);
	icm20948_accel_calibration(hi2c1);
//
	icm20948_gyro_full_scale_select(hi2c1,_2000dps);
	icm20948_accel_full_scale_select(hi2c1,_16g);
}



void ak09916_init(I2C_HandleTypeDef *hi2c1)
{
	icm20948_i2c_master_reset(hi2c1);
	osDelay(5);
	icm20948_i2c_master_enable(hi2c1);
	icm20948_i2c_master_clk_frq(hi2c1,7);

	//if(!ak09916_who_am_i(hi2c1)) return;

	ak09916_soft_reset(hi2c1);
	//ak09916_operation_mode_setting(hi2c1,continuous_measurement_100hz);
	ak09916_operation_mode_setting(hi2c1,	continuous_measurement_100hz);

}



void icm20948_gyro_read(I2C_HandleTypeDef *hi2c1,axises* data)
{
	uint8_t* temp = read_multiple_icm20948_reg(hi2c1,ub_0, B0_GYRO_XOUT_H, 6);

	data->x = (int16_t)(temp[0] << 8 | temp[1]);
	data->y = (int16_t)(temp[2] << 8 | temp[3]);
	data->z = (int16_t)(temp[4] << 8 | temp[5]);
}

void icm20948_accel_read(I2C_HandleTypeDef *hi2c1,axises* data)
{
	uint8_t* temp = read_multiple_icm20948_reg(hi2c1,ub_0, B0_ACCEL_XOUT_H, 6);

	data->x = (int16_t)(temp[0] << 8 | temp[1]);
	data->y = (int16_t)(temp[2] << 8 | temp[3]);
	data->z = (int16_t)(temp[4] << 8 | temp[5]); //+ accel_scale_factor;
	// Add scale factor because calibraiton function offset gravity acceleration.
}

bool ak09916_mag_read(I2C_HandleTypeDef *hi2c1,axises* data)
{


//	if(!ak09916_who_am_i(hi2c1)) {
		write_single_icm20948_reg(hi2c1,ub_3, B3_I2C_SLV0_ADDR, READ | MAG_SLAVE_ADDR);
//		data->x = (int16_t)(0);
//		data->y = (int16_t)(0);
//		data->z = (int16_t)(0);
//		return false;
//	}else{
		uint8_t* temp;
		uint8_t drdy, hofl;	// data ready, overflow

		drdy = read_single_ak09916_reg(hi2c1,MAG_ST1) & 0x01;
	//	if(!drdy)	return false;
		if (drdy) {
			temp = read_multiple_ak09916_reg(hi2c1,MAG_HXL, 6);

			data->x = (float)(temp[0] << 8 | temp[1]);
			data->y = (float)(temp[2] << 8 | temp[3]);
			data->z = (float)(temp[4] << 8 | temp[5]);
		}
		hofl = read_single_ak09916_reg(hi2c1,MAG_ST2) & 0x08;
		if(hofl) return false;



		return true;
//	}
}

void icm20948_gyro_read_dps(I2C_HandleTypeDef *hi2c1,axises* data)
{
	icm20948_gyro_read(hi2c1,data);

	data->x /= gyro_scale_factor;
	data->y /= gyro_scale_factor;
	data->z /= gyro_scale_factor;
}

void icm20948_accel_read_g(I2C_HandleTypeDef *hi2c1,axises* data)
{
	icm20948_accel_read(hi2c1,data);

	data->x /= accel_scale_factor;
	data->y /= accel_scale_factor;
	data->z /= accel_scale_factor;

}

bool ak09916_mag_read_uT(I2C_HandleTypeDef *hi2c1,axises* data)
{
	axises temp;
	bool new_data = ak09916_mag_read(hi2c1,&temp);
	if(!new_data)return false;

	data->x = (float)(temp.x * 0.15);
	data->y = (float)(temp.y * 0.15);
	data->z = (float)(temp.z * 0.15);

	return true;
}
//Sub Function

//Who am I
bool icm20948_who_am_i(I2C_HandleTypeDef* hi2c1)
{
	uint8_t icm20948_id = read_single_icm20948_reg(hi2c1,ub_0, B0_WHO_AM_I);

	if(icm20948_id == ICM20948_ID)
		return true;
	else
		return false;
}

bool ak09916_who_am_i(I2C_HandleTypeDef* hi2c1)
{
	uint8_t ak09916_id = read_single_ak09916_reg(hi2c1,MAG_WIA2);

	if(ak09916_id == AK09916_ID)
		return true;
	else
		return false;
}

//Power Management

/*device reset*/
void icm20948_device_reset(I2C_HandleTypeDef *hi2c1)
{
	//write_single_icm20948_reg(hi2c1,ub_0, B0_PWR_MGMT_1, 0x80 | 0x41);
	write_single_icm20948_reg(hi2c1,ub_0, B0_PWR_MGMT_1, 0x80);
}
/*soft reset magnetometer*/
void ak09916_soft_reset(I2C_HandleTypeDef * hi2c1)
{
	write_single_ak09916_reg(hi2c1,MAG_CNTL3, 0x01);
}
/*wake up the imu*/
void icm20948_wakeup(I2C_HandleTypeDef* hi2c1)
{
	uint8_t new_val = read_single_icm20948_reg(hi2c1,ub_0, B0_PWR_MGMT_1);
	new_val &= 0xBF;

	write_single_icm20948_reg(hi2c1,ub_0, B0_PWR_MGMT_1, new_val);
}
/*put imu to sleep*/
void icm20948_sleep(I2C_HandleTypeDef* hi2c1)
{
	uint8_t new_val = read_single_icm20948_reg(hi2c1,ub_0, B0_PWR_MGMT_1);
	new_val |= 0x40;

	write_single_icm20948_reg(hi2c1,ub_0, B0_PWR_MGMT_1, new_val);
}

/* write to imu the sampling rate */
void icm20948_gyro_sample_rate_divider(I2C_HandleTypeDef *hi2c1,uint8_t divider)
{
	write_single_icm20948_reg(hi2c1,ub_2, B2_GYRO_SMPLRT_DIV, divider);
}
/* write to imu the sampling rate*/
void icm20948_accel_sample_rate_divider(I2C_HandleTypeDef *hi2c1,uint16_t divider)
{
	uint8_t divider_1 = (uint8_t)(divider >> 8);
	uint8_t divider_2 = (uint8_t)(0x0F & divider);

	write_single_icm20948_reg(hi2c1,ub_2, B2_ACCEL_SMPLRT_DIV_1, divider_1);
	write_single_icm20948_reg(hi2c1,ub_2, B2_ACCEL_SMPLRT_DIV_2, divider_2);
}

/* config the low pass filter for gyro*/
void icm20948_gyro_low_pass_filter(I2C_HandleTypeDef *hi2c1,uint8_t config)
{
	uint8_t new_val = read_single_icm20948_reg(hi2c1,ub_2, B2_GYRO_CONFIG_1);
	new_val |= config << 3;

	write_single_icm20948_reg(hi2c1,ub_2, B2_GYRO_CONFIG_1, new_val);
}
/* config the low pass filter for accelerometer*/
void icm20948_accel_low_pass_filter(I2C_HandleTypeDef *hi2c1,uint8_t config)
{
	uint8_t new_val = read_single_icm20948_reg(hi2c1,ub_2, B2_ACCEL_CONFIG);
	new_val |= config << 3;

	write_single_icm20948_reg(hi2c1,ub_2, B2_ACCEL_CONFIG, new_val);
}
/* gyro calibration */
void icm20948_gyro_calibration(I2C_HandleTypeDef *hi2c1)
{
	axises temp;
	int32_t gyro_bias[3] = {0};
	uint8_t gyro_offset[6] = {0};

	for(int i = 0; i < 100; i++)
	{
		icm20948_gyro_read(hi2c1,&temp);
		gyro_bias[0] += temp.x;
		gyro_bias[1] += temp.y;
		gyro_bias[2] += temp.z;
	}

	gyro_bias[0] /= 100;
	gyro_bias[1] /= 100;
	gyro_bias[2] /= 100;

	// Construct the gyro biases for push to the hardware gyro bias registers,
	// which are reset to zero upon device startup.
	// Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format.
	// Biases are additive, so change sign on calculated average gyro biases
	gyro_offset[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF;
	gyro_offset[1] = (-gyro_bias[0] / 4)       & 0xFF;
	gyro_offset[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
	gyro_offset[3] = (-gyro_bias[1] / 4)       & 0xFF;
	gyro_offset[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
	gyro_offset[5] = (-gyro_bias[2] / 4)       & 0xFF;

	write_multiple_icm20948_reg(hi2c1,ub_2, B2_XG_OFFS_USRH, gyro_offset, 6);
}
/* accel calibration */
void icm20948_accel_calibration(I2C_HandleTypeDef *hi2c1)
{
	axises temp;
	uint8_t* temp2;
	uint8_t* temp3;
	uint8_t* temp4;

	int32_t accel_bias[3] = {0};
	int32_t accel_bias_reg[3] = {0};
	uint8_t accel_offset[6] = {0};

	for(int i = 0; i < 100; i++)
	{
		icm20948_accel_read(hi2c1,&temp);
		accel_bias[0] += temp.x;
		accel_bias[1] += temp.y;
		accel_bias[2] += temp.z;
	}

	accel_bias[0] /= 100;
	accel_bias[1] /= 100;
	accel_bias[2] /= 100;

	uint8_t mask_bit[3] = {0, 0, 0};

	temp2 = read_multiple_icm20948_reg(hi2c1,ub_1, B1_XA_OFFS_H, 2);
	accel_bias_reg[0] = (int32_t)(temp2[0] << 8 | temp2[1]);
	mask_bit[0] = temp2[1] & 0x01;

	temp3 = read_multiple_icm20948_reg(hi2c1,ub_1, B1_YA_OFFS_H, 2);
	accel_bias_reg[1] = (int32_t)(temp3[0] << 8 | temp3[1]);
	mask_bit[1] = temp3[1] & 0x01;

	temp4 = read_multiple_icm20948_reg(hi2c1,ub_1, B1_ZA_OFFS_H, 2);
	accel_bias_reg[2] = (int32_t)(temp4[0] << 8 | temp4[1]);
	mask_bit[2] = temp4[1] & 0x01;

	accel_bias_reg[0] -= (accel_bias[0] / 8);
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	accel_offset[0] = (accel_bias_reg[0] >> 8) & 0xFF;
 	accel_offset[1] = (accel_bias_reg[0])      & 0xFE;
	accel_offset[1] = accel_offset[1] | mask_bit[0];

	accel_offset[2] = (accel_bias_reg[1] >> 8) & 0xFF;
 	accel_offset[3] = (accel_bias_reg[1])      & 0xFE;
	accel_offset[3] = accel_offset[3] | mask_bit[1];

	accel_offset[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	accel_offset[5] = (accel_bias_reg[2])      & 0xFE;
	accel_offset[5] = accel_offset[5] | mask_bit[2];

	write_multiple_icm20948_reg(hi2c1,ub_1, B1_XA_OFFS_H, &accel_offset[0], 2);
	write_multiple_icm20948_reg(hi2c1,ub_1, B1_YA_OFFS_H, &accel_offset[2], 2);
	write_multiple_icm20948_reg(hi2c1,ub_1, B1_ZA_OFFS_H, &accel_offset[4], 2);
}

void icm20948_gyro_full_scale_select(I2C_HandleTypeDef *hi2c1,gyro_full_scale full_scale)
{
	uint8_t new_val = read_single_icm20948_reg(hi2c1,ub_2, B2_GYRO_CONFIG_1);

	switch(full_scale)
	{
		case _250dps :
			new_val |= 0x00;
			gyro_scale_factor = 131.0;
			break;
		case _500dps :
			new_val |= 0x02;
			gyro_scale_factor = 65.5;
			break;
		case _1000dps :
			new_val |= 0x04;
			gyro_scale_factor = 32.8;
			break;
		case _2000dps :
			new_val |= 0x06;
			gyro_scale_factor = 16.4;
			break;
	}

	write_single_icm20948_reg(hi2c1,ub_2, B2_GYRO_CONFIG_1, new_val);
}

void icm20948_accel_full_scale_select(I2C_HandleTypeDef *hi2c1,accel_full_scale full_scale)
{
	uint8_t new_val = read_single_icm20948_reg(hi2c1,ub_2, B2_ACCEL_CONFIG);

	switch(full_scale)
	{
		case _2g :
			new_val |= 0x00;
			accel_scale_factor = 16384;
			break;
		case _4g :
			new_val |= 0x02;
			accel_scale_factor = 8192;
			break;
		case _8g :
			new_val |= 0x04;
			accel_scale_factor = 4096;
			break;
		case _16g :
			new_val |= 0x06;
			accel_scale_factor = 2048;
			break;
	}

	write_single_icm20948_reg(hi2c1,ub_2, B2_ACCEL_CONFIG, new_val);
}

/* write to imu the operation mode magnetometer*/
void ak09916_operation_mode_setting(I2C_HandleTypeDef *hi2c1,operation_mode mode)
{
	write_single_ak09916_reg(hi2c1,MAG_CNTL2, mode);
}


///* Sub Functions */
//void icm20948_spi_slave_enable(I2C_HandleTypeDef hi2c1)
//{
//	uint8_t new_val = read_single_icm20948_reg(hi2c1,ub_0, B0_USER_CTRL);
//	new_val |= 0x10;
//
//	write_single_icm20948_reg(hi2c1,ub_0, B0_USER_CTRL, new_val);
//}



void icm20948_i2c_master_reset(I2C_HandleTypeDef *hi2c1)
{
	uint8_t new_val = read_single_icm20948_reg(hi2c1,ub_0, B0_USER_CTRL);
	new_val |= 0x02;

	write_single_icm20948_reg(hi2c1,ub_0, B0_USER_CTRL, new_val);
}

void icm20948_i2c_master_enable(I2C_HandleTypeDef *hi2c1)
{
	uint8_t new_val = read_single_icm20948_reg(hi2c1,ub_0, B0_USER_CTRL);
	new_val |= 0x20;

	write_single_icm20948_reg(hi2c1,ub_0, B0_USER_CTRL, new_val);
}

void icm20948_i2c_master_clk_frq(I2C_HandleTypeDef *hi2c1,uint8_t config)
{
	uint8_t new_val = read_single_icm20948_reg(hi2c1,ub_3, B3_I2C_MST_CTRL);
	new_val |= config;

	write_single_icm20948_reg(hi2c1,ub_3, B3_I2C_MST_CTRL, new_val);
}

void icm20948_clock_source(I2C_HandleTypeDef *hi2c1,uint8_t source)
{
	uint8_t new_val = read_single_icm20948_reg(hi2c1,ub_0, B0_PWR_MGMT_1);
	new_val |= source;

	write_single_icm20948_reg(hi2c1,ub_0, B0_PWR_MGMT_1, new_val);
}

void icm20948_odr_align_enable(I2C_HandleTypeDef *hi2c1)
{
	write_single_icm20948_reg(hi2c1,ub_2, B2_ODR_ALIGN_EN, 0x01);
}




