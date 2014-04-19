/*
 * hp203b.h
 *
 *  Created on: Feb 11, 2014
 *      Author: v.panasiuk
 */

#ifndef HP203B_H_
#define HP203B_H_

#include "stdint.h"

#define HP203B_USAGE	1		// Using of HP203B sensor (0:don't use,1:use)
#define HP203C_USAGE	1		// Using of HP203C sensor (0:don't use,1:use)

/* INT_EN bits define */
#define PA_RDY_EN		5
#define T_RDY_EN		4
#define PA_TRAV_EN		3
#define T_TRAV_EN		2
#define PA_WIN_EN		1
#define T_WIN_EN		0

/* INT_CFG bits define */
#define PA_MODE			6
#define PA_RDY_CFG		5
#define T_RDY_CFG		4
#define PA_TRAV_CFG		3
#define T_TRAV_CFG		2
#define PA_WIN_CFG		1
#define T_WIN_CFG		0

/* INT_SRC bits define */
#define TH_ERR			7
#define DEV_RDY			6		// Indicates that the sensor is ready (don't do any operation and in a sleep mode)
#define PA_RDY			5		// Indicates that the pressure (or altitude) measurement is done and the result is ready to read
#define T_RDY			4		// Indicate that the temperature measurement is done and the result is ready to read
#define PA_TRAV			3		// Indicate that the pressure (or altitude) value has traversed the middle threshold during the last measurement
#define T_TRAV			2		// Indicate that the temperature value has traversed the middle threshold during the last measurement
#define PA_WIN			1		// Indicate that the pressure (or altitude) value locates outside the pre-defined window (the value in between the upper bound and lower bound thresholds) during the last measurement
#define T_WIN			0		// Indicate that the temperature value locates outside the pre-defined window (the value in between the upper bound and lower bound thresholds) during the last measurement

/* INT_DIR bits define */
#define CMPS_EN			7
#define P_TRAV_DIR		3
#define T_TRAV_DIR		2
#define P_WIN_DIR		1
#define T_WIN_DIR		0

/* PARA bits define */
#define CMPS_EN			7

/* The 2-bit channel (CHNL) parameter tells the device the data from which channel(s) shall be converted by the internal ADC */
typedef enum {
	PT_CHANNEL	= 0x00,		// Sensor pressure and temperature channel
	T_CHANNEL	= 0x01		// Temperature channel
} HP203x_Channel;

/* The 3-bit OSR defines the decimation rate of the internal digital filter */
typedef enum {
	OSR4096		= 0x00,		// 4096
	OSR2048		= 0x01,		// 2048
	OSR1024		= 0x02,		// 1024
	OSR512		= 0x03,		// 512
	OSR256		= 0x04,		// 256
	OSR128		= 0x05		// 128
} HP203x_OSR;

/* Command set for HP203B sensor */
typedef enum {
	SOFT_RST	= 0x06,		// Soft reset the device
	READ_PT		= 0x10,		// Read the temperature and pressure values
	READ_AT		= 0x11,		// Read the temperature and altitude values
	ANA_CAL		= 0x28,		// Re-calibrate the internal analog blocks
	READ_P		= 0x30,		// Read the pressure value only
	READ_A		= 0x31,		// Read the altitude value only
	READ_T		= 0x32,		// Read the temperature value only
	ADC_CVT		= 0x40,		// Perform ADC conversion
	READ_REG	= 0x80,		// Read out the control registers
	WRITE_REG	= 0xC0		// Write in the control registers
} HP203x_Command;

typedef enum {
	ALT_OFF_LSB	= 0x00,
	ALT_OFF_MSB	= 0x01,
	PA_H_TH_LSB	= 0x02,
	PA_H_TH_MSB	= 0x03,
	PA_M_TH_LSB	= 0x04,
	PA_M_TH_MSB	= 0x05,
	PA_L_TH_LSB	= 0x06,
	PA_L_TH_MSB	= 0x07,
	T_H_TH		= 0x08,
	T_M_TH		= 0x09,
	T_L_TH		= 0x0A,
	INT_EN		= 0x0B,
	INT_CFG		= 0x0C,
	INT_SRC		= 0x0D,
	INT_DIR		= 0x0E,
	PARA		= 0x0F
} HP203x_Registers;

/* HP203B control registers */
typedef struct {
	uint8_t ALT_OFF_LSB;	// Altitude Offset Compensation
	uint8_t ALT_OFF_MSB;
	uint8_t PA_H_TH_LSB;	// Set pressure/altitude upper bound threshold
	uint8_t PA_H_TH_MSB;
	uint8_t PA_M_TH_LSB;	// Set pressure/altitude middle bound threshold
	uint8_t PA_M_TH_MSB;
	uint8_t PA_L_TH_LSB;	// Set pressure/altitude lower bound threshold
	uint8_t PA_L_TH_MSB;
	uint8_t T_H_TH;			// Set temperature upper bound threshold
	uint8_t T_M_TH;			// Set temperature middle bound threshold
	uint8_t T_L_TH;			// Set temperature lower bound threshold
	uint8_t INT_EN;			// Interrupts enable/disable
	uint8_t INT_CFG;		// Interrupts configure
	uint8_t INT_SRC;		// Interrupts status
	uint8_t INT_DIR;
	uint8_t PARA;			// Data compensation enable/disable
} HP203x_CR_TypeDef;

typedef struct {
	int16_t  ALT_OFF;		// Altitude offset (cm)
	uint16_t P_H_TH;		// Pressure upper bound threshold (Pa)
	uint16_t P_M_TH;		// Pressure middle bound threshold (Pa)
	uint16_t P_L_TH;		// Pressure lower bound threshold (Pa)
	int16_t  A_H_TH;		// Altitude upper bound threshold (m)
	int16_t  A_M_TH;		// Altitude middle bound threshold (m)
	int16_t  A_L_TH;		// Altitude lower bound threshold (m)
	int8_t   T_H_TH;		// Temperature upper bound threshold (°C)
	int8_t   T_M_TH;		// Temperature middle bound threshold (°C)
	int8_t   T_L_TH;		// Temperature lower bound threshold (°C)
	int8_t   P_or_A;		// 0 for pressure, 1 for altitude
} HP203x_TH_TypeDef;

/* HP203B measuring data */
typedef struct {
	int32_t  T;				// Temperature
	uint32_t P;				// Pressure
	int32_t  H;				// Altitude
} HP203x_Data_TypeDef;

#if HP203B_USAGE
/* HP203B internal commands */
void HP203B_SoftReset(void);
void HP203B_StartConvert(uint8_t, uint8_t);
void HP203B_ReadPT(void);
void HP203B_ReadAT(void);
void HP203B_ReadP(void);
void HP203B_ReadA(void);
void HP203B_ReadT(void);
void HP203B_Calibration(HP203x_CR_TypeDef*);
void HP203B_ReadReg(HP203x_CR_TypeDef*, uint8_t);
void HP203B_WriteReg(HP203x_CR_TypeDef*, uint8_t);

/* HP203B general commands */
void HP203B_ReadAllReg(HP203x_CR_TypeDef*);
void HP203B_WriteAllReg(HP203x_CR_TypeDef*);
void HP203B_Init(HP203x_CR_TypeDef*, HP203x_TH_TypeDef*);
#endif

#if HP203C_USAGE
/* HP203C internal commands */
void HP203C_SoftReset(void);
void HP203C_StartConvert(uint8_t, uint8_t);
void HP203C_ReadPT(void);
void HP203C_ReadAT(void);
void HP203C_ReadP(void);
void HP203C_ReadA(void);
void HP203C_ReadT(void);
void HP203C_Calibration(HP203x_CR_TypeDef*);
void HP203C_ReadReg(HP203x_CR_TypeDef*, uint8_t);
void HP203C_WriteReg(HP203x_CR_TypeDef*, uint8_t);

/* HP203C general commands */
void HP203C_ReadAllReg(HP203x_CR_TypeDef*);
void HP203C_WriteAllReg(HP203x_CR_TypeDef*);
void HP203C_Init(HP203x_CR_TypeDef*, HP203x_TH_TypeDef*);
#endif


#endif /* HP203B_H_ */
