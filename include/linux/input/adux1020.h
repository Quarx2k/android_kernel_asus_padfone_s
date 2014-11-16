/**
@file adux1020.h
@brief ADUX1020  - Low level driver  Header 'H' File
*/
#ifndef _ADUX1020_H_
#define _ADUX1020_H_
/**
	@brief ADUX1020 i2c slave address
*/
#define ADUX1020_DEVID		0x64
/**
	@brief maximum General register count
*/
#define GENERAL_REG_CNT		80
/**
	@brief maximum Proximity register count
*/
#define PROXIMITY_REG_CNT	80
/**
	@brief maximum Gesture register count
*/
#define GESTURE_REG_CNT		80
/**
	@brief maximum Sample register count
*/
#define SAMPLE_REG_CNT		80
/**
	@brief number of Gesture sign value
*/
#define NUM_OF_GESTURE_VALUE	28


/**
	@brief ADUX1020 Platform Data
*/
struct adux_platform_data {
	u16 config;
	u32 general_reg_cnt;
	u32 proximity_reg_cnt;
	u32 gesture_reg_cnt;
	u32 sample_xyi_reg_cnt;
	u32 sample_raw_reg_cnt;
	u32 general_regs[GENERAL_REG_CNT];
	u32 proximity_regs[PROXIMITY_REG_CNT];
	u32 gesture_regs[GESTURE_REG_CNT];
	u32 sample_xyi_regs[SAMPLE_REG_CNT];
	u32 sample_raw_regs[SAMPLE_REG_CNT];
};
#endif
