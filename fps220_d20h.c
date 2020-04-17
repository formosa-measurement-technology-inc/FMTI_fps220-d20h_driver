/*
 * Copyright (C) 2015 Formosa Measurement Technology Inc. Ltd. All rights
 * reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

/* Driver description
 *
 * File Name    :
 * Authors      : conrad@fmti.com.tw
 * Version      : 1.0.0
 * Date         : 2020/1/6
 * Description  : FPS220 pressure sensor API for MCU of ARM_M0 core
 *
 */

/* Revised history
 * 1.0.0: first release
 *
 */

#include "fps220_d20h.h"

volatile uint32_t TMR0_Ticks;
volatile uint32_t fps220_update_rdy;

static void fps220_us_delay(uint32_t us);
#ifdef SPI
static uint8_t fps220_spi_writeblock(uint8_t reg_addr, uint32_t cnt, const uint8_t *reg_data);
static uint8_t fps220_spi_readblock(uint8_t reg_addr, uint32_t cnt, uint8_t *reg_data);
#else
static uint8_t fps220_i2c_writeblock(uint8_t reg_addr, uint32_t cnt, const uint8_t *reg_data);
static uint8_t fps220_i2c_readblock(uint8_t reg_addr, uint32_t cnt, uint8_t *reg_data);
#endif
static int32_t fps220_startMeasure_temp(struct fps220_data *barom);
static int32_t fps220_get_raw_temperature(struct fps220_data *barom);
static int32_t fps220_startMeasure_press(struct fps220_data *barom);
static int32_t fps220_get_raw_pressure(struct fps220_data *barom);
static int32_t fps220_read_store_otp_data(struct fps220_data *barom);
static int32_t fps220_set_oversampling_rate(struct fps220_data *barom
        , enum fps220_osr osr_setting);
static int32_t fps220_chipid_check(struct fps220_data *barom);
static int32_t fps220_version_identification(struct fps220_data *barom);
static int32_t fps220_calculation(struct fps220_data *barom);

/**
 * { pointer of fps220 data }
 */
static struct fps220_data fps220_barom;
struct fps220_data *barom = &fps220_barom;

#ifdef SPI
static uint8_t fps220_spi_writeblock(uint8_t reg_addr, uint32_t cnt, const uint8_t *reg_data)
{
	/* This is just an example. This function have to
	   be implemented according to your platform. */
	uint8_t cmd, i;

	switch (cnt) {
	case 1:
		cmd = FPS220_SPI_WRITE | FPS220_SPI_1BYTE;
		break;
	case 2:
		cmd = FPS220_SPI_WRITE | FPS220_SPI_2BYTE;
		break;
	case 3:
		cmd = FPS220_SPI_WRITE | FPS220_SPI_3BYTE;
		break;
	default:
		cmd = FPS220_SPI_WRITE | FPS220_SPI_4BYTE;
	}
	SPI_SET_SS0_LOW(SPI0);
	/* Write to TX register */
	SPI_WRITE_TX0(SPI0, cmd);
	/* Trigger SPI data transfer */
	SPI_TRIGGER(SPI0);
	while (SPI_IS_BUSY(SPI0));
	SPI_WRITE_TX0(SPI0, (reg_addr + (cnt - 1)));
	SPI_TRIGGER(SPI0);
	while (SPI_IS_BUSY(SPI0));
	for (i = 0; i < cnt; i++)
	{
		SPI_WRITE_TX0(SPI0, *(reg_data + i));
		SPI_TRIGGER(SPI0);
		/* Check SPI0 busy status */
		while (SPI_IS_BUSY(SPI0));
	}
	SPI_SET_SS0_HIGH(SPI0);

	return 0;
}
static uint8_t fps220_spi_readblock(uint8_t reg_addr, uint32_t cnt, uint8_t *reg_data)
{
	/* This is just an example. This function have to
	   be implemented according to your platform. */

	int8_t i;
	uint8_t cmd;
	uint32_t tmp;

	switch (cnt) {
	case 1:
		cmd = FPS220_SPI_READ | FPS220_SPI_1BYTE;
		break;
	case 2:
		cmd = FPS220_SPI_READ | FPS220_SPI_2BYTE;
		break;
	case 3:
		cmd = FPS220_SPI_READ | FPS220_SPI_3BYTE;
		break;
	default:
		cmd = FPS220_SPI_READ | FPS220_SPI_4BYTE;
	}
	SPI_SET_SS0_LOW(SPI0);
	/* Write to TX register */
	SPI_WRITE_TX0(SPI0, cmd);
	SPI_TRIGGER(SPI0);
	while (SPI_IS_BUSY(SPI0));
	SPI_WRITE_TX0(SPI0, reg_addr + (cnt - 1));
	SPI_TRIGGER(SPI0);
	while (SPI_IS_BUSY(SPI0));
	for (i = (cnt - 1); i >= 0; i--)
	{
		SPI_WRITE_TX0(SPI0, 0x00);//dummy clock
		SPI_TRIGGER(SPI0);
		while (SPI_IS_BUSY(SPI0));
		tmp = SPI_READ_RX0(SPI0);
//			printf("SPI read: %#x\n\r", tmp);
		*(reg_data + i) = tmp;
	}
	SPI_SET_SS0_HIGH(SPI0);

	return 0;
}
#endif

#ifdef I2C
static uint8_t fps220_i2c_writeblock(uint8_t reg_addr, uint32_t cnt, const uint8_t *reg_data)
{
	/* This is just an example. This function have to
	   be implemented according to your platform. */
	uint8_t status;
	uint32_t cnt_write;
	cnt_write = I2C_WriteMultiBytesOneReg(I2C0, FPS220_I2C_SLAVE_ADDR, reg_addr \
	                                      , reg_data, cnt);
	status = (cnt_write > 0) ?  0 : 1;
	return status;
}
static uint8_t fps220_i2c_readblock(uint8_t reg_addr, uint32_t cnt, uint8_t *reg_data)
{
	/* This is just an example. This function have to
	   be implemented according to your platform. */
	uint8_t status;
	uint32_t cnt_read;

	cnt_read = I2C_ReadMultiBytesOneReg(I2C0, FPS220_I2C_SLAVE_ADDR\
	                                    , reg_addr, reg_data, cnt);
	status = (cnt_read > 0) ?  0 : 1;
	return status;
}
#endif
/**
 * @brief      { API for fps220 delay }
 *
 * @param[in]  us    { delay time in microseconds }
 */
static void fps220_us_delay(uint32_t us)
{
	/* This is just an example. This function have to
	   be implemented according to your platform. */
	CLK_SysTickDelay(us);
}
/**
 * @brief      { API for assigning function pointers, as bus read/write
 *               and delay. }
 *
 * @return     { 0, succeeded
 *              -1, failed }
 */
int8_t fps220_init(void)
{
	int32_t err;
	uint8_t data_buf;

	fps220_barom.delay_usec = fps220_us_delay;
	/* The minimum start up time of fps220 is 25ms */
	barom->delay_usec(1000 * 25);

#ifdef SPI
	fps220_barom.bus_write = fps220_spi_writeblock;
	fps220_barom.bus_read = fps220_spi_readblock;
	/* The default of SPI is in 3 wires mode after power on reset. If 4 wires SPI
    mode is preffered, the following statements will be needed. */
    #define SPI_4_WIRES_MODE
	#ifdef SPI_4_WIRES_MODE
		/* Set SPI bus as 4 wires mode */
		data_buf = FPS220_SPI_CTRL_REG_SDO_ACTIVE_EN;
		barom->bus_write(FPS220_SPI_CTRL_REG, sizeof(uint8_t), &data_buf);
	#endif
#else
	fps220_barom.bus_write = fps220_i2c_writeblock;
	fps220_barom.bus_read = fps220_i2c_readblock;
#endif
	
	err = fps220_chipid_check(barom);
	if (err) {
		err = -1;
		goto err_chip_id_chk;
	} else {
#ifdef DEBUG_FPS220
		printf("%s:fps220_chipid_check() passed!\n", __func__);
#endif
	}

	err = fps220_version_identification(barom);
	if (err) {
		err = -2;
		goto err_version_identification;
	} else {
#ifdef DEBUG_FPS220
		printf("%s:fps220_version_identification() passed!\n", __func__);
#endif
	}

	err = fps220_read_store_otp_data(barom);
	if (err) {
		err = -3;
		goto err_read_otp;
	} else {
#ifdef DEBUG_FPS220
		printf("%s:fps220_read_store_otp_data() passed!\n", __func__);
#endif
	}
	err = 0;

	fps220_set_oversampling_rate(barom, OVERSAMPLING_RATE_DEFAULT);
	/* Setting the P_CONFIG_REG_GAIN */
#define P_CONFIG_REG_GAIN_SETTING FPS220_P_CONFIG_REG_GAIN_X16
	barom->bus_read(FPS220_P_CONFIG_REG, sizeof(uint8_t), &data_buf);
	data_buf &= ~(FPS220_P_CONFIG_REG_GAIN_MAK);
	data_buf |= P_CONFIG_REG_GAIN_SETTING;
	barom->bus_write(FPS220_P_CONFIG_REG, sizeof(uint8_t), &data_buf);
#ifdef DEBUG_FPS220
	printf("%s:Setting of FPS220_P_CONFIG_REG_GAIN: %#x\n", __func__, P_CONFIG_REG_GAIN_SETTING);
#endif

#ifdef DEBUG_FPS220
	printf("fps220_init succeeded!\n");
#endif
    return err;

err_chip_id_chk:
#ifdef DEBUG_FPS220
	printf("%s:fps220_init() failed!; fps220_ID:%#x,err:%d\n", __func__, fps220_barom.chip_id, err);
#endif
	return err;
err_version_identification:
#ifdef DEBUG_FPS220
	printf("%s:fps220_init() failed!; fps220 version:%#x,err:%d\n", __func__, fps220_barom.hw_ver, err);
#endif
	return err;
err_read_otp:
#ifdef DEBUG_FPS220
	printf("%s:fps220_init() failed!; fps220 otp reading failed!,err:%d\n", __func__, err);
#endif
	return err;
}

int32_t fps220_read_raw_t(void)
{
	return barom->raw_temperature;
}
/**
 * @brief      API for read real temperature value in unit of degree Celsius
 *
 * @return     { temperature value in unit of degree Celsius }
 */
float fps220_read_temperature(void)
{
	fps220_calculation(barom);
	return barom->real_temperature * 0.01;
}

int32_t fps220_read_raw_p(void)
{
	return barom->raw_pressure;
}
/**
 * @brief      API for read real pressure value in unit of Pa
 *
 * @return     { pressure value in unit of Pa }
 */
float fps220_read_pressure(void)
{
	fps220_calculation(barom);
	return barom->real_pressure * 0.01;
}
/**
 * @brief      API for read real temperature and pressure values
 *             stored in fps220_data structure
 *
 * @param      real_pressure     The pointer for saving real pressure value
 *                               Pressure unit: 0.01 Pa
 * @param      real_temperature  The pointer for saving real temperature value
 *                               Temperature unit: 0.01 degree Celsius
 */
void fps220_read_data(int32_t *real_pressure, int32_t *real_temperature)
{
	fps220_calculation(barom);
	*real_pressure = barom->real_pressure;
	*real_temperature = barom->real_temperature;
	return;
}

/**
 * @brief      { This api ignite a measurement procedure. It writes data into
 *               the register of FPS220_TAKE_MEAS_REG. }
 *
 * @param      barom  pointer of fps220 data structure
 *
 * @return     { return of bus_write() }
 */
static int fps220_startMeasure_temp(struct fps220_data *barom)
{
	int err;
	uint8_t bus_wr_data;

	bus_wr_data = FPS220_MEAS_TEMP;
	err = barom->bus_write(FPS220_TAKE_MEAS_REG, sizeof(uint8_t), &bus_wr_data);

	return err;
}
/**
 * @brief      { This api gets the data from the registers of FPS220_READ_MEAS_REG_U
 *               , FPS220_READ_MEAS_REG_L and FPS220_READ_MEAS_REG_XL. And the data are
 *               stored in "barom->raw_temperature". }
 *
 * @param      barom  pointer of fps220 data structure
 *
 * @return     { return of bus_read() }
 */
static int fps220_get_raw_temperature(struct fps220_data *barom)
{
	int err;
	uint8_t buf[3] = {0};

	err = barom->bus_read(FPS220_READ_MEAS_REG_U, 3 * sizeof(uint8_t), buf);
	barom->raw_temperature = ((uint32_t)buf[0] << 16) + ((uint32_t)buf[1] << 8) + buf[2];

#ifdef DEBUG_FPS220
	printf("%s: uncompensated temperature: %d\n", DEVICE_NAME, barom->raw_temperature);
#endif
	return err;
}
/**
 * @brief      { This api ignite a measurement procedure. It writes data into
 *               the register of FPS220_TAKE_MEAS_REG. }
 *
 * @param      barom  pointer of fps220 data structure
 *
 * @return     { return of bus_write() }
 */
static int32_t fps220_startMeasure_press(struct fps220_data *barom)
{
	int32_t err;
	uint8_t bus_wr_data;

	bus_wr_data = barom->cmd_start_p;
	err = barom->bus_write(FPS220_TAKE_MEAS_REG, sizeof(uint8_t), &bus_wr_data);

	return err;
}
/**
 * @brief      { This api gets the data from the registers of FPS220_READ_MEAS_REG_U
 *               , FPS220_READ_MEAS_REG_L and FPS220_READ_MEAS_REG_XL. And the data are
 *               stored in "barom->raw_temperature". }
 *
 * @param      barom  pointer of fps220 data structure
 *
 * @return     { return of bus_read() }
 */
static int32_t fps220_get_raw_pressure(struct fps220_data *barom)
{
	int32_t err;
	uint8_t buf[3] = {0};

	err = barom->bus_read(FPS220_READ_MEAS_REG_U, 3 * sizeof(uint8_t), buf);
	barom->raw_pressure = ((uint32_t)buf[0] << 16) + ((uint32_t)buf[1] << 8) + buf[2];

#ifdef DEBUG_FPS220
	printf("%s: uncompensated pressure:  %d\n", DEVICE_NAME, barom->raw_pressure);
#endif

	return err;
}
/**
 * @brief      { API for reading calibration data saved in OTP memory }
 *
 * @param      barom  FPS220 data structure
 *
 * @return     { description_of_the_return_value }
 */
static int32_t fps220_read_store_otp_data(struct fps220_data *barom)
{
	struct fps220_calibration_data *cali = &(barom->calibration);
	int32_t status;
	uint16_t R[10] = {0};
	uint8_t tmp[FPS220_CALIBRATION_DATA_LENGTH] = {0};	

	status = barom->bus_read(FPS220_CALIBRATION_DATA_START0,
	                         (FPS220_CALIBRATION_DATA_LENGTH - 2) * sizeof(uint8_t),
	                         (uint8_t *)tmp);

	if (status < 0)
		goto exit;
	status = barom->bus_read(FPS220_CALIBRATION_DATA_START1, sizeof(uint8_t), (uint8_t *)tmp + 18 );
	if (status < 0)
		goto exit;
	status = barom->bus_read(FPS220_CALIBRATION_DATA_START2, sizeof(uint8_t), (uint8_t *)tmp + 19);
	if (status < 0)
		goto exit;
	/* Read OTP data here */
	R[0] = (tmp[0] << 8 | tmp[1]);
	R[1] = (tmp[2] << 8 | tmp[3]);
	R[2] = (tmp[4] << 8 | tmp[5]);
	R[3] = (tmp[6] << 8 | tmp[7]);
	R[4] = (tmp[8] << 8 | tmp[9]);
	R[5] = (tmp[10] << 8 | tmp[11]);
	R[6] = (tmp[12] << 8 | tmp[13]);
	R[7] = (tmp[14] << 8 | tmp[15]);
	R[8] = (tmp[16] << 8 | tmp[17]);
	R[9] = (tmp[18] << 8 | tmp[19]);

	/* Coefficient reconstruction */
	switch (barom->hw_ver) {
	case hw_ver_b0:
	case hw_ver_b1:
	case hw_ver_b2:
		cali->C0 = R[0] >> 3;
		cali->C1 = R[1] >> 8;
		cali->C2 = R[2] >> 5;
		cali->C3 = R[3] >> 5;
		cali->C4 = ((uint32_t)R[4] << 3) | (R[0] & 7);
		cali->C5 = R[5];
		cali->C6 = R[6] >> 1;
		cali->C7 = ((uint32_t)R[7] << 3) | (R[8] & 7);
		cali->C8 = R[8] >> 3;
		cali->C9 = R[9] >> 1;
		cali->C10 = R[1] & 0xFF;
		cali->C11 = ((R[2] & 0x1F) << 3) | ((R[3] & 0x1C) >>2);
		cali->C12 = ((R[3] & 0x3) << 2) | ((R[6] & 0x1) << 1 | (R[9] & 0x1));
        break;
    case hw_ver_unknown:
		break;
	};

#if defined(DEBUG_FPS220) || defined(MSG_LOG)
	printf("%s: R0= %#x\n", DEVICE_NAME, R[0]);
	printf("%s: R1= %#x\n", DEVICE_NAME, R[1]);
	printf("%s: R2= %#x\n", DEVICE_NAME, R[2]);
	printf("%s: R3= %#x\n", DEVICE_NAME, R[3]);
	printf("%s: R4= %#x\n", DEVICE_NAME, R[4]);
	printf("%s: R5= %#x\n", DEVICE_NAME, R[5]);
	printf("%s: R6= %#x\n", DEVICE_NAME, R[6]);
	printf("%s: R7= %#x\n", DEVICE_NAME, R[7]);
	printf("%s: R8= %#x\n", DEVICE_NAME, R[8]);
	printf("%s: R9= %#x\n", DEVICE_NAME, R[9]);
	printf("%s: C0= %d\n", DEVICE_NAME, cali->C0);
	printf("%s: C1= %d\n", DEVICE_NAME, cali->C1);
	printf("%s: C2= %d\n", DEVICE_NAME, cali->C2);
	printf("%s: C3= %d\n", DEVICE_NAME, cali->C3);
	printf("%s: C4= %d\n", DEVICE_NAME, cali->C4);
	printf("%s: C5= %d\n", DEVICE_NAME, cali->C5);
	printf("%s: C6= %d\n", DEVICE_NAME, cali->C6);
	printf("%s: C7= %d\n", DEVICE_NAME, cali->C7);
	printf("%s: C8= %d\n", DEVICE_NAME, cali->C8);
	printf("%s: C9= %d\n", DEVICE_NAME, cali->C9);
	printf("%s: C10= %d\n", DEVICE_NAME, cali->C10);
	printf("%s: C11= %d\n", DEVICE_NAME, cali->C11);
	printf("%s: C12= %d\n", DEVICE_NAME, cali->C12);
#endif
exit:
	return status;
}

/**
 * @brief      { API for reading hardware version }
 *
 * @param      barom  FPS220 data structure
 *
 * @return     { description_of_the_return_value }
 */
static int fps220_version_identification(struct fps220_data *barom)
{
	int32_t err;
	uint8_t buf[2] = {0};
	uint8_t version = 0;
	uint8_t bus_wr_data;

	bus_wr_data = FPS220_SOFTRESET_CMD;
	barom->bus_write(FPS220_SOFTRESET_REG, sizeof(uint8_t), &bus_wr_data);
	/* The minimum start up time of fps220 is 25ms */
    barom->delay_usec(1000 * 25); 
	err = barom->bus_read(FPS220_TAKE_MEAS_REG, sizeof(uint8_t), buf);
	err = barom->bus_read(FPS220_VERSION_REG, sizeof(uint8_t), buf + 1);

	version = ((buf[0] & 0xC0) >> 6) | ((buf[1] & 0x70) >> 2);
#if defined(DEBUG_FPS220) || defined(MSG_LOG)
	printf("%s: The value of version: %#x\n", __func__, version);
#endif

	switch (version)	{
	case hw_ver_b0:
		barom->hw_ver = hw_ver_b0;
#if defined(DEBUG_FPS220) || defined(MSG_LOG)
		printf("%s: The version of sensor is B0.\n", __func__);
#endif		
		break;
	case hw_ver_b1:
		barom->hw_ver = hw_ver_b1;
#if defined(DEBUG_FPS220) || defined(MSG_LOG)
        printf("%s: The version of sensor is B1.\n", __func__);
#endif		
		break;
	case hw_ver_b2:
		barom->hw_ver = hw_ver_b2;
#if defined(DEBUG_FPS220) || defined(MSG_LOG)
		printf("%s: The version of sensor is B2.\n", __func__);
#endif
		break;
	default:
		barom->hw_ver = hw_ver_unknown;
#if defined(DEBUG_FPS220) || defined(MSG_LOG)
		printf("%s: The version of sensor is unknown.\n", __func__);
#endif
		break;
	}
	return err;
}
static int32_t fps220_set_oversampling_rate(struct fps220_data *barom
        , enum fps220_osr osr_setting)
{
	uint8_t reg_addr;
	uint8_t data_buf;

	barom->oversampling_rate = osr_setting;
#ifdef DEBUG_FPS220
	printf("Setting of oversampling_rate:%#x\r\n", barom->oversampling_rate);
#endif			

	/* Setting conversion time for pressure measurement */
	switch (osr_setting) {
	case osr_1024:
		barom->cnvTime_press = FPS220_CONVERSION_usTIME_OSR1024;
		barom->cmd_start_p = FPS220_MEAS_PRESS_OVERSAMP_0;
		break;
	case osr_2048:
		barom->cnvTime_press = FPS220_CONVERSION_usTIME_OSR2048;
		barom->cmd_start_p = FPS220_MEAS_PRESS_OVERSAMP_1;
		break;
	case osr_4096:
		barom->cnvTime_press = FPS220_CONVERSION_usTIME_OSR4096;
		barom->cmd_start_p = FPS220_MEAS_PRESS_OVERSAMP_2;
		break;
	case osr_8192:
		barom->cnvTime_press = FPS220_CONVERSION_usTIME_OSR8192;
		barom->cmd_start_p = FPS220_MEAS_PRESS_OVERSAMP_3;
		break;
	case osr_16384:
		barom->cnvTime_press = FPS220_CONVERSION_usTIME_OSR16384;
		reg_addr = 0xa6;
		barom->bus_read(reg_addr, sizeof(uint8_t), &data_buf);
		data_buf &= 0xf8;
		data_buf |= 0x6;
		barom->bus_write(reg_addr, sizeof(uint8_t), &data_buf);
		barom->cmd_start_p = FPS220_MEAS_PRESS_OVERSAMP_2;
		barom->bus_read(0xA6, sizeof(uint8_t), &data_buf);
#ifdef DEBUG_FPS220
		printf("reg_0xA6:%#x\n\r", data_buf);
#endif
		break;
	}
	/* Setting conversion time for temperature measurement */
	barom->cnvTime_temp = FPS220_CONVERSION_usTIME_OSR1024;

	return 0;
}
static int32_t fps220_chipid_check(struct fps220_data *barom)
{
	int32_t err;
	uint8_t chip_id_read;

	err = barom->bus_read(FPS220_CHIP_ID_REG, sizeof(uint8_t), &chip_id_read);
#ifdef DEBUG_FPS220
	printf("%s: chip_id reading is %#x \n", __func__, chip_id_read);
#endif

	if (chip_id_read != FPS220_CHIP_ID) {
		err = -1;
		return err;
	} else {
		barom->chip_id = chip_id_read;
		return err = 0;
	}
}
/**
 * @brief      { API for triggering measurement procedure and updating
 *               the temperature and pressure data in fps220_data structure. }
 */
void fps220_update_data(void)
{
	static uint32_t t_start_flag = 0;
	static uint32_t p_start_flag = 0;
	static uint32_t tick_current;
	static uint32_t tick_last;
	static uint32_t tick_diff;

	tick_current = TMR0_Ticks;
	tick_diff = tick_current - tick_last;

	if (t_start_flag == 0 && !fps220_update_rdy)
	{
#ifdef DEBUG_FPS220
		printf("start t_measurement\r\n");
#endif			
		fps220_startMeasure_temp(barom);
		t_start_flag = 1;
		tick_last = TMR0_Ticks;
	}
	else if ((tick_diff * 1000 > barom->cnvTime_temp ) && (p_start_flag == 0))
	{
#ifdef DEBUG_FPS220
		printf("start p_measurement\r\n");
#endif			
		fps220_get_raw_temperature(barom);
		fps220_startMeasure_press(barom);
		p_start_flag = 1;
		tick_last = TMR0_Ticks;
	}
	else if (tick_diff * 1000 > barom->cnvTime_press )
	{
#ifdef DEBUG_FPS220
		printf("read pressure\r\n");
#endif			
		fps220_get_raw_pressure(barom);
		t_start_flag = 0;
		p_start_flag = 0;
		tick_current = 0;
		tick_last = 0;
		TMR0_Ticks = 0;
		fps220_update_rdy = 1;
	}
#ifdef DEBUG_FPS220
	printf("tick_current:%d\r\n", tick_current);
	printf("tick_last:%d\r\n", tick_last);
	printf("FPS220 is updating %d\r\n", TMR0_Ticks);
#endif		
	return ;
}
/**
 * @brief      { API for calculating real temperature and pressure values.
 *               The results are stored in fps220_data structure.
 *               "barom->real_temperature" is represented real temperature value.
 *               "barom->real_temperature" is in uint of 0.01 drgree Celsius.
 *               "barom->real_pressure" is represented real pressure value.
 *               "barom->real_pressure" is in unit of 0.01 Pa. }
 *
 * @param      barom  pointer of fps220 data structure
 *
 * @return     { description_of_the_return_value }
 */
int fps220_calculation(struct fps220_data *barom)
{
	struct fps220_calibration_data *cali = &barom->calibration;
	int32_t X01, X02, X11, X12, X13, X21, X22, X23, X24, X25, X26, X27, X31, X32;
	int32_t PP1, PP2, PP3, PP4, CF;
	int32_t RT, RP, UT, UP, DT, DT2;

	switch (barom->hw_ver) {
	case hw_ver_b0:
	case hw_ver_b1:
	case hw_ver_b2:		
        UT = barom->raw_temperature;
        UP = barom->raw_pressure;
        /* calculation for real temperature value*/		
		DT = ((UT - 8388608) >> 4) - ((cali->C0 + 707L) << 4);
		X01 = (cali->C1 + 617L) * DT;
		X02 = (((((cali->C2 - 1024L) * DT) >> 14) * DT) >> 3) - X01;
		DT2 = ((((((((cali->C3 - 1024L) * DT) >> 14) * DT) >> 17) * DT) >> 2) + X02) >> 9;
		RT =  ((2500L << 3) + DT2) >> 3;
		/* calculation for real pressure value*/		
		X11 = ((cali->C5 - 32768L) * DT2);
    	X12 = ((((cali->C6 - 16384L) * DT2) >> 16) * DT2) >> 1;
    	X13 = ((X11 + X12) >> 11) + ((cali->C4 - 325059) << 4);
    	X21 = (cali->C8 + 5305L) * DT2;
    	X22 = ((((((cali->C9 - 16384L) * DT2) >> 16) * DT2) >> 1) + X21) >> 10;
    	X23 = X22 > 0 ? X22 : -X22;
    	X24 = (X23 >> 11) * (cali->C7 + 193138);
    	X25 = ((((X23 & 0x7FF) * (cali->C7 + 193138)) >> 11) + X24) >> 11;
    	X26 = X22 > 0 ? ((cali->C7 + 193138) + X25) >> 2 : ((cali->C7 + 193138) - X25) >> 2;
        X27 = X26 > 0 ? X26 : -X26;
    	PP1 = ((UP - 8388608) - X13) >> 4;
    	PP2 = (X27 >> 11) * PP1;
    	PP3 = ((((((X27 & 0x7FF) * PP1) >> 11) + PP2) >> 5) * 100) >> 6;
    	RP = X26 > 0 ? PP3 : -PP3;
        if ((cali->C10 == 0) && (cali->C11 == 0) && (cali->C12 == 0));
        else
        {
            PP4 = ((((X27 & 0x7FF) * PP1) >> 11) + PP2) >>11;
            CF = (cali->C12 - 8L) * DT2 + 1048576;
            X31 = (((cali->C10 - 128L) * CF) >> 9) * PP4;
            X32 = ((((((((cali->C11 - 128L) * CF) >> 9) * PP4) >> 10) * PP4 + X31) >> 8) * 100) >> 13;
            RP -= X32;
        }
		break;
    case hw_ver_unknown:
        break;
	};

	barom->real_temperature = RT; //uint:0.01 degree Celsius
	barom->real_pressure = RP; //uint: 0.01 Pa

#ifdef DEBUG_FPS220
	printf("%s: calibrated pressure: %d\n", DEVICE_NAME, RP);
#endif

	return 0;
}
