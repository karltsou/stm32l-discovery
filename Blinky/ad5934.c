/*
 * AD5934 Impedance Converter, Network Analyzer
 *
 * Copyright 2011 Analog Devices Inc.
 *
 * Licensed under the GPL-2.
 *
 * Karl Tsou <karl.funlab@gmail.com>
 * Michael Hennerich <hennerich@blackfin.uclinux.org>
 */

#include <stdio.h>
#include "STM32L1xx.h"
#include "stm32l1xx_i2c.h"

/* AD5934 SLAVE ADDRESS */
#define AD5934_ADDRESS          0x0d

/* AD5933/AD5934 Registers */
#define AD5933_REG_CONTROL_HB		0x80	/* R/W, 2 bytes */
#define AD5933_REG_CONTROL_LB		0x81	/* R/W, 2 bytes */
#define AD5933_REG_FREQ_START		0x82	/* R/W, 3 bytes */
#define AD5933_REG_FREQ_INC		0x85	/* R/W, 3 bytes */
#define AD5933_REG_INC_NUM		0x88	/* R/W, 2 bytes, 9 bit */
#define AD5933_REG_SETTLING_CYCLES	0x8A	/* R/W, 2 bytes */
#define AD5933_REG_STATUS		0x8F	/* R, 1 byte */
#define AD5933_REG_TEMP_DATA		0x92	/* R, 2 bytes*/
#define AD5933_REG_REAL_DATA		0x94	/* R, 2 bytes*/
#define AD5933_REG_IMAG_DATA		0x96	/* R, 2 bytes*/

/* AD5933_REG_CONTROL_HB Bits */
#define AD5933_CTRL_INIT_START_FREQ	(0x1 << 4)
#define AD5933_CTRL_START_SWEEP		(0x2 << 4)
#define AD5933_CTRL_INC_FREQ		(0x3 << 4)
#define AD5933_CTRL_REPEAT_FREQ		(0x4 << 4)
#define AD5933_CTRL_MEASURE_TEMP	(0x9 << 4)
#define AD5933_CTRL_POWER_DOWN		(0xA << 4)
#define AD5933_CTRL_STANDBY		(0xB << 4)

#define AD5933_CTRL_RANGE_2000mVpp	(0x0 << 1)
#define AD5933_CTRL_RANGE_200mVpp	(0x1 << 1)
#define AD5933_CTRL_RANGE_400mVpp	(0x2 << 1)
#define AD5933_CTRL_RANGE_1000mVpp	(0x3 << 1)
#define AD5933_CTRL_RANGE(x)		((x) << 1)

#define AD5933_CTRL_PGA_GAIN_1		(0x1 << 0)
#define AD5933_CTRL_PGA_GAIN_5		(0x0 << 0)

/* AD5933_REG_CONTROL_LB Bits */
#define AD5933_CTRL_RESET		(0x1 << 4)
#define AD5933_CTRL_INT_SYSCLK		(0x0 << 3)
#define AD5933_CTRL_EXT_SYSCLK		(0x1 << 3)

/* AD5933_REG_STATUS Bits */
#define AD5933_STAT_TEMP_VALID		(0x1 << 0)
#define AD5933_STAT_DATA_VALID		(0x1 << 1)
#define AD5933_STAT_SWEEP_DONE		(0x1 << 2)

/* I2C Block Commands */
#define AD5933_I2C_BLOCK_WRITE		0xA0
#define AD5933_I2C_BLOCK_READ		0xA1
#define AD5933_I2C_ADDR_POINTER		0xB0

/* Device Specs */
#define AD5933_INT_OSC_FREQ_Hz		16776000
#define AD5933_MAX_OUTPUT_FREQ_Hz	100000
#define AD5933_MAX_RETRIES		100

#define AD5933_OUT_RANGE		1
#define AD5933_OUT_RANGE_AVAIL		2
#define AD5933_OUT_SETTLING_CYCLES	3
#define AD5933_IN_PGA_GAIN		4
#define AD5933_IN_PGA_GAIN_AVAIL	5
#define AD5933_FREQ_POINTS		6

#define AD5933_POLL_TIME_ms		10
#define AD5933_INIT_EXCITATION_TIME_ms	100

/**
 * struct ad5933_platform_data - platform specific data
 * @ext_clk_Hz:		the external clock frequency in Hz, if not set
 *			the driver uses the internal clock (16.776 MHz)
 * @vref_mv:		the external reference voltage in millivolt
 */

struct ad5933_platform_data {
	unsigned long			ext_clk_Hz;
	unsigned short			vref_mv;
};

typedef struct __ad5933_state {
	struct ad5933_platform_data	*pdata;
	unsigned long			mclk_hz;
	unsigned char			ctrl_hb;
	unsigned char			ctrl_lb;
	unsigned			range_avail[4];
	unsigned short			vref_mv;
	unsigned short			settling_cycles;
	unsigned short			freq_points;
	unsigned			freq_start;
	unsigned			freq_inc;
	unsigned			state;
	unsigned			poll_time_jiffies;
} AD5933_STATE, *PAD5933_STATE;

#define do_div(x, y) (x = x / y);

#define swap32(x) \
( (((x)>>24)&0xff)|(((x)>>8)&0xff00)|(((x)<<8)&0xff0000)|(((x)<<24)&0xff000000) );
#define cpu_to_be32(x) \
swap32(x)

#define swap16(x) \
( (((x)>>8)&0xff)|(((x)<<8)&0xff00) );
#define cpu_to_be16(x) \
swap16(x)

/*
  External func headfile

*/

ErrorStatus I2C_Write(
 I2C_TypeDef* I2Cx,
 const uint8_t* buf,
 uint32_t nbyte,
 uint8_t SlaveAddress);

ErrorStatus I2C_Read(
 I2C_TypeDef* I2Cx,
 uint8_t *buf,
 uint32_t nbyte,
 uint8_t SlaveAddress);

void Delay (
	uint32_t dlyTicks);
 
/*
  Global Variable

 */

static AD5933_STATE g_st;
static struct ad5933_platform_data ad5933_default_pdata;
//= {
//  .ext_clk_Hz = AD5933_INT_OSC_FREQ_Hz,
//	.vref_mv = 3300,
//};

/*
  AD5934 I2C read func

*/
static int ad5933_i2c_read(uint8_t reg, uint8_t len, uint8_t *data)
{
	uint8_t ptbuf[2];
	uint8_t datbuf[2] = {0,0};

	ptbuf[0] = 0xb0;
	ptbuf[1] = reg;

	I2C_Write(I2C1, ptbuf, 2, AD5934_ADDRESS << 1);	  /* Register Pointer */
	I2C_Read(I2C1, datbuf, len, AD5934_ADDRESS << 1); /* Register Data    */

  data[0] = datbuf[0];
	if (len > 1)
		data[1] = datbuf[1];

	return 0;
}

/*
 AD5934 I2C write func

*/
static int ad5933_i2c_write(uint8_t reg, uint8_t len, uint8_t dat)
{
	uint8_t cmdbuf[2];

	cmdbuf[0] = reg;     /* Register Address */
	cmdbuf[1] = dat;     /* Register Data    */

	return I2C_Write(I2C1, cmdbuf, len, AD5934_ADDRESS << 1);
}

/*
 AD5934 CONTROL REGISTER (REGISTER ADDRESS 0x80)

 D15 D14 D13 D12
   0   0   0   0   No operation
   0   0   0   1   Initialize with start frequency
   0   0   1   0   Start frequency sweep
   0   0   1   1   Increment frequency
   0   1   0   0   Repeat frequency
   1   0   0   0   No operation
   1   0   0   1   No operation
   1   0   1   0   Power-down mode
   1   0   1   1   Standby mode
   1   1   0   0   No operation
   1   1   0   1   No operation

*/
static int ad5933_cmd(AD5933_STATE *st, unsigned char cmd)
{
	unsigned char dat = st->ctrl_hb | cmd;

	return ad5933_i2c_write(AD5933_REG_CONTROL_HB, 2, dat);
}

/*

 AD5934 Reset

*/
static int ad5933_reset(AD5933_STATE *st)
{
	unsigned char dat = st->ctrl_lb | AD5933_CTRL_RESET;

	return ad5933_i2c_write(AD5933_REG_CONTROL_LB, 2, dat);
}

/*
  AD5934
  START FREQUENCY REGISTER
  -------------------------------------------------------------------------------------------------
  @REGISTER ADDRESS 0x82
  @REGISTER ADDRESS 0x83
  @REGISTER ADDRESS 0x84

  The start frequency register contains the 24-bit digital representation of the frequency
  from where the subsequent frequency sweep is initiated.
  For example, if the user requires the sweep to start from a frequency of 30 kHz using a 16.0 MHz clock,
  the user must program the value 0x3D to Register Address 0x82,
  the value 0x70 to Register Address 0x83, and the value 0xA3 to Register Address 0x84.
  Doing this ensures the output frequency starts at 30 kHz.


  START FREQUENCY REGISTER
  -------------------------------------------------------------------------------------------------
  @REGISTER ADDRESS 0x82
  @REGISTER ADDRESS 0x83
	@REGISTER ADDRESS 0x84

  The frequency increment register contains a 24-bit representation of the frequency increment
	between consecutive frequency points along the sweep.
	For example, if the user requires an increment step of 30 Hz using a 16.0 MHz clock
	the user must program the value 0x00 to Register Address 0x85,
	the value 0x0F to Register Address 0x86, and the value 0xBA to Register Address 0x87.

*/
static int ad5933_set_freq(AD5933_STATE *st,
			   unsigned reg, unsigned long freq)
{
	unsigned long long freqreg;
	int ret = SUCCESS;
	union {
		uint32_t d32;
		uint8_t d8[4];
	} dat;

	freqreg = (uint64_t) freq * (uint64_t) (1 << 27);
	do_div(freqreg, st->mclk_hz / 16);

	switch (reg) {
	case AD5933_REG_FREQ_START:
		st->freq_start = freq;
		break;
	case AD5933_REG_FREQ_INC:
		st->freq_inc = freq;
		break;
	default:
		return -1;
	}

	dat.d32 = cpu_to_be32(freqreg);
	ret = ad5933_i2c_write(reg,   2, dat.d8[0]);
	ret = ad5933_i2c_write(reg+1, 2, dat.d8[1]);
	ret = ad5933_i2c_write(reg+2, 2, dat.d8[2]);

	return ret;
}

/*
  AD5934
  - Reset
  - Start Freguency
  - Increntment Frequency
*/
static int ad5933_setup(AD5933_STATE *st)
{
	union {
	  uint16_t d16;
		uint8_t  d8[2];
	} dat;
	int ret;

	ret = ad5933_reset(st);
	if (ret == ERROR)
		return ret;

	ret = ad5933_set_freq(st, AD5933_REG_FREQ_START, 10000);
	if (ret == ERROR)
		return ret;

	ret = ad5933_set_freq(st, AD5933_REG_FREQ_INC, 200);
	if (ret == ERROR)
		return ret;

	/*
	NUMBER OF SETTLING TIME CYCLES REGISTER
	@REGISTER ADDRESS 0x8A
	@REGISTER ADDRESS 0x8B

	The maximum number of output cycles that can be programmed is 511 × 4 = 2044 cycles.
	For example, consider an excitation signal of 30 kHz,
	the maximum delay between the programming of this frequency
	and the time that this signal is first sampled by the ADC is ˜ 511 × 4 × 33.33 µs = 68.126 ms.
	The ADC takes 1024 samples, and the result is stored as real data and imaginary data in Register Address 0x94
	to Register Address 0x97.
	The conversion process takes approximately 1 ms using a 16.777 MHz clock.
	*/

	st->settling_cycles = 10;
	dat.d16 = cpu_to_be16(st->settling_cycles);

	ret = ad5933_i2c_write(AD5933_REG_SETTLING_CYCLES, 2, dat.d8[0]);
	ret = ad5933_i2c_write(AD5933_REG_SETTLING_CYCLES+1, 2, dat.d8[1]);
	if (ret == ERROR)
		return ret;

	/*
	NUMBER OF INCREMENTS REGISTER
	@REGISTER ADDRESS 0x88
	@REGISTER ADDRESS 0x89

	This register determines the number of frequency points in the frequency sweep.
	The number of frequency points is represented by a 9-bit word,
	D8 to D0. D15 to D9 are don’t care bits.
	This register in conjunction with the start frequency register and
	the frequency increment register determine the frequency sweep range for the sweep operation.
	The maximum number of increments that can be programmed is 511.
	*/
	st->freq_points = 100;
	dat.d16 = cpu_to_be16(st->freq_points);

	ret = ad5933_i2c_write(AD5933_REG_INC_NUM, 2, dat.d8[0]);
	ret = ad5933_i2c_write(AD5933_REG_INC_NUM+1, 2, dat.d8[1]);

	return ret;
}

/*
  AD5934 Work

*/
static int ad5933_work(AD5933_STATE *st, uint8_t *buf)
{
	//AD5933_STATE *st = &g_st;

	//struct iio_dev *indio_dev = i2c_get_clientdata(st->client);
	uint8_t *datbuf = buf;
	uint16_t real;
	uint16_t magnitude;
	unsigned char status;

	//st = &g_st;

	// mutex_lock(&indio_dev->mlock);
	if (st->state == AD5933_CTRL_INIT_START_FREQ) {
		/* start sweep */
		ad5933_cmd(st, AD5933_CTRL_START_SWEEP);
		st->state = AD5933_CTRL_START_SWEEP;

		//schedule_delayed_work(&st->work, st->poll_time_jiffies);
		//mutex_unlock(&indio_dev->mlock);
		return 0;
	}

	ad5933_i2c_read(AD5933_REG_STATUS, 1, &status);

	if (status & AD5933_STAT_DATA_VALID) {
		//int scan_count = bitmap_weight(indio_dev->active_scan_mask,
		//			       indio_dev->masklength);
		//ad5933_i2c_read(st->client,
		//		test_bit(1, indio_dev->active_scan_mask) ?
		//		AD5933_REG_REAL_DATA : AD5933_REG_IMAG_DATA,
		//		scan_count * 2, (u8 *)buf);

		ad5933_i2c_read(
				AD5933_REG_REAL_DATA,
				2, datbuf);

		real = datbuf[0] << 8 | datbuf[1];

		ad5933_i2c_read(
				AD5933_REG_IMAG_DATA,
				2, datbuf);

		magnitude = datbuf[0] << 8 | datbuf[1];
		//if (scan_count == 2) {
		//	buf[0] = be16_to_cpu(buf[0]);
		//	buf[1] = be16_to_cpu(buf[1]);
		//} else {
		//	buf[0] = be16_to_cpu(buf[0]);
		//}
		//iio_push_to_buffers(indio_dev, buf);
	}
	else {
		/* no data available - try again later */
		//schedule_delayed_work(&st->work, st->poll_time_jiffies);
		//mutex_unlock(&indio_dev->mlock);
		return 0;
	}

	if (status & AD5933_STAT_SWEEP_DONE) {
		/* last sample received - power down do nothing until
		 * the ring enable is toggled */
		ad5933_cmd(st, AD5933_CTRL_POWER_DOWN);
	} else {
		/* we just received a valid datum, move on to the next */
		ad5933_cmd(st, AD5933_CTRL_INC_FREQ);
		//schedule_delayed_work(&st->work, st->poll_time_jiffies);
		return 0;
	}

	//mutex_unlock(&indio_dev->mlock);
	return 1;
}

int ad5933_ring_preenable()
{
	AD5933_STATE *st = &g_st;
	int32_t ret;

	ret = ad5933_reset(st);
	if (ret == ERROR)
		return ret;

	ret = ad5933_cmd(st, AD5933_CTRL_STANDBY);
	if (ret == ERROR)
		return ret;

	ret = ad5933_cmd(st, AD5933_CTRL_INIT_START_FREQ);
	if (ret == ERROR)
		return ret;

	st->state = AD5933_CTRL_INIT_START_FREQ;

	return 0;
}

int ad5933_ring_postenable()
{
	AD5933_STATE *st = &g_st;
	uint8_t buf[4] = {0,0,0,0};
	uint8_t status = 0;

	/* AD5933_CTRL_INIT_START_FREQ:
	 * High Q complex circuits require a long time to reach steady state.
	 * To facilitate the measurement of such impedances, this mode allows
	 * the user full control of the settling time requirement before
	 * entering start frequency sweep mode where the impedance measurement
	 * takes place. In this mode the impedance is excited with the
	 * programmed start frequency (ad5933_ring_preenable),
	 * but no measurement takes place.
	 */

	//schedule_delayed_work(&st->work,
	//		      msecs_to_jiffies(AD5933_INIT_EXCITATION_TIME_ms));
	while (!status) {
	  Delay(AD5933_INIT_EXCITATION_TIME_ms);
	  status = ad5933_work(st, buf);
	}

	//
	// Fill Real & Image into another buffer that
	// uesd to be send to DAC in NEXT step.
	//

	return 0;
}

int ad5933_ring_postdisable()
{
	AD5933_STATE *st = &g_st;

	//cancel_delayed_work_sync(&st->work);
	return ad5933_cmd(st, AD5933_CTRL_POWER_DOWN);
}

static void ad5933_calc_out_ranges(AD5933_STATE *st)
{
	int i;
	unsigned normalized_3v3[4] = {1980, 198, 383, 970};

	for (i = 0; i < 4; i++)
		st->range_avail[i] = normalized_3v3[i] * st->vref_mv / 3300;
}

/*
  AD5934 entry

*/
void ad5933_probe()
{
	AD5933_STATE *st = &g_st;

	//
	// Default Settings
	//
	ad5933_default_pdata.vref_mv = 3300;
  ad5933_default_pdata.ext_clk_Hz = AD5933_INT_OSC_FREQ_Hz;

	st->pdata = &ad5933_default_pdata;
	st->vref_mv = st->pdata->vref_mv;

	//
	// External MCLK 16 MHz,
	//
  if (st->pdata->ext_clk_Hz) {
		st->mclk_hz = st->pdata->ext_clk_Hz;
		st->ctrl_lb = AD5933_CTRL_EXT_SYSCLK;
	}
	//else {
	//	st->mclk_hz = AD5933_INT_OSC_FREQ_Hz;
	//	st->ctrl_lb = AD5933_CTRL_INT_SYSCLK;
	//}

	ad5933_calc_out_ranges(st);
  ad5933_setup(st);
}
