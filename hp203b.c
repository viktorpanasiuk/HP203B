/*
 * hp203b.c
 *
 *
 *
 *  Created on: Feb 11, 2014
 *      Author: v.panasiuk
 */
#include "hp20xx.h"
#include "lpc17xx_i2c.h"
#include "LPC17xx.h"

#define CSB_LEVEL	0			// Define the level of CSB pin
#define CSB_GPIO	LPC_GPIO0
#define INT0_GPIO	LPC_GPIOINT
#define INT1_GPIO	LPC_GPIOINT
#define CSB_PIN		24
#define INT0_PIN	25
#define INT1_PIN	26

/* PRIVATE VARIABLES =============================================================================================================================== */
static I2C_M_SETUP_Type	TransferCfg;
static uint8_t			I2CMasterBuffer[8];
static uint8_t			I2CSlaveBuffer[8];

/* PUBLIC VARIABLES ================================================================================================================================ */
HP203x_TH_TypeDef		HP203B_TH_Struct;
HP203x_CR_TypeDef		HP203B_CR_Struct;
HP203x_Data_TypeDef		HP203B_Data;

/* PRIVATE FUNCTIONS =============================================================================================================================== */
/************************************************************
 * Configure I2C transfer structure for HP203B
 * - this structure define the I2C transfer
 * - this function exist because we are using certain I2C driver
 ***********************************************************/
static void HP203B_TrasferCfgInit(void) {
	if(CSB_GPIO->FIOPIN & (1<<CSB_PIN))
		TransferCfg.sl_addr7bit = 0x76;			// Slave address of HP203B if CSB high
	else
		TransferCfg.sl_addr7bit = 0x77;			// Slave address of HP203B if CSB low
	TransferCfg.tx_data = I2CMasterBuffer;		// Pointer to transmit buffer
//	TransferCfg.tx_length = 0;					// Number of bytes for write
	TransferCfg.tx_count = 0;					// Counter for writing bytes
	TransferCfg.rx_data = I2CSlaveBuffer;		// Pointer to receive buffer
//	TransferCfg.rx_length = 0;					// Number of bytes for read
	TransferCfg.rx_count = 0;					// Counter of reading bytes
	TransferCfg.retransmissions_max = 5;		// Number of attempts
	TransferCfg.retransmissions_count = 0;		// Counter of retransmissions
	TransferCfg.status = 0xF8;					// Current status of I2C bus
	TransferCfg.callback = NULL;				// Pointer to function that's executes after I2C transfer (NULL if none)
}

/************************************************************
 * Write 1 byte
 * - this function is write 1 byte from I2CMasterBuffer to I2C bus
 * - this function depend on I2C driver which we are using
 * - for other I2C driver the main goal of this function must be the same
 ***********************************************************/
static void write1byte(void) {
	TransferCfg.tx_length = 1;					// Number of write bytes
	TransferCfg.rx_length = 0;					// Number of read bytes
	HP203B_TrasferCfgInit();					// Configure other parameters of I2C transfer
	while(I2C_MasterTransferData(LPC_I2C0, &TransferCfg, I2C_TRANSFER_POLLING) != SUCCESS);	// Start the I2C transfer and wait until it finish successful
}

/************************************************************
 * Write 2 bytes
 * - this function is write 2 bytes from I2CMasterBuffer to I2C bus
 * - this function depend on I2C driver which we are using
 * - for other I2C driver the main goal of this function must be the same
 ***********************************************************/
static void write2byte(void) {
	TransferCfg.tx_length = 2;					// Number of write bytes
	TransferCfg.rx_length = 0;					// Number of read bytes
	HP203B_TrasferCfgInit();					// Configure other parameters of I2C transfer
	while(I2C_MasterTransferData(LPC_I2C0, &TransferCfg, I2C_TRANSFER_POLLING) != SUCCESS);	// Start the I2C transfer and wait until it finish successful
}

/************************************************************
 * Read 1 byte
 * - this function is read 1 byte to the I2CSlaveBuffer from I2C bus
 * - this function depend on I2C driver which we are using
 * - for other I2C driver the main goal of this function must be the same
 ***********************************************************/
static void read1byte(void) {
	TransferCfg.tx_length = 0;					// Number of write bytes
	TransferCfg.rx_length = 1;					// Number of read bytes
	HP203B_TrasferCfgInit();					// Configure other parameters of I2C transfer
	while(I2C_MasterTransferData(LPC_I2C0, &TransferCfg, I2C_TRANSFER_POLLING) != SUCCESS);	// Start the I2C transfer and wait until it finish successful
}

/************************************************************
 * Read 3 byte
 * - this function is read 3 bytes to the I2CSlaveBuffer from I2C bus
 * - this function depend on I2C driver which we are using
 * - for other I2C driver the main goal of this function must be the same
 ***********************************************************/
static void read3byte(void) {
	TransferCfg.tx_length = 0;					// Number of write bytes
	TransferCfg.rx_length = 3;					// Number of read bytes
	HP203B_TrasferCfgInit();					// Configure other parameters of I2C transfer
	while(I2C_MasterTransferData(LPC_I2C0, &TransferCfg, I2C_TRANSFER_POLLING) != SUCCESS);	// Start the I2C transfer and wait until it finish successful
}

/************************************************************
 * Read 6 byte
 * - this function is read 6 bytes to the I2CSlaveBuffer from I2C bus
 * - this function depend on I2C driver which we are using
 * - for other I2C driver the main goal of this function must be the same
 ***********************************************************/
static void read6byte(void) {
	TransferCfg.tx_length = 0;					// Number of write bytes
	TransferCfg.rx_length = 6;					// Number of read bytes
	HP203B_TrasferCfgInit();					// Configure other parameters of I2C transfer
	while(I2C_MasterTransferData(LPC_I2C0, &TransferCfg, I2C_TRANSFER_POLLING) != SUCCESS);	// Start the I2C transfer and wait until it finish successful
}

/************************************************************
 * Initialize the structure of altitude offset and thresholds
 * - read comments for understanding values metric
 ***********************************************************/
static void HP203B_TH_StructInit(HP203x_TH_TypeDef *HP203B_TH_InitStruct) {
	HP203B_TH_InitStruct->ALT_OFF = 0;		// Altitude offset (cm)
	HP203B_TH_InitStruct->P_H_TH  = 0;		// Pressure upper bound threshold (Pa)
	HP203B_TH_InitStruct->P_M_TH  = 0;		// Pressure middle bound threshold (Pa)
	HP203B_TH_InitStruct->P_L_TH  = 0;		// Pressure lower bound threshold (Pa)
	HP203B_TH_InitStruct->A_H_TH  = 0;		// Altitude upper bound threshold (m)
	HP203B_TH_InitStruct->A_M_TH  = 0;		// Altitude middle bound threshold (m)
	HP203B_TH_InitStruct->A_L_TH  = 0;		// Altitude lower bound threshold (m)
	HP203B_TH_InitStruct->T_H_TH  = 0;		// Temperature upper bound threshold (°C)
	HP203B_TH_InitStruct->T_M_TH  = 0;		// Temperature middle bound threshold (°C)
	HP203B_TH_InitStruct->T_L_TH  = 0;		// Temperature lower bound threshold (°C)
	HP203B_TH_InitStruct->P_or_A  = 1;		// 0 for pressure, 1 for altitude
}

/************************************************************
 * Initialize the HP203B control registers structure
 * - read comments for understanding changes which you are can adjust
 ***********************************************************/
static void HP203B_CR_StructInit(HP203x_CR_TypeDef *HP203B_CR_InitStruct, HP203x_TH_TypeDef *HP203B_TH_Struct) {
	HP203B_TH_StructInit(HP203B_TH_Struct);												// Configure the structure of HP203B thresholds
	HP203B_CR_InitStruct->ALT_OFF_LSB = (uint8_t)(HP203B_TH_Struct->ALT_OFF);			// Write LSB of altitude offset (1cm per count)
	HP203B_CR_InitStruct->ALT_OFF_MSB = (uint8_t)(HP203B_TH_Struct->ALT_OFF >> 8);		// Write MSB of altitude offset
	if(!HP203B_TH_Struct->P_or_A) {														// If P_or_A = 0 (pressure)
		HP203B_CR_InitStruct->PA_H_TH_LSB = (uint8_t)(HP203B_TH_Struct->P_H_TH >> 1);	// Write LSB for upper bound of pressure (0.02mbar or 2Pa per count)
		HP203B_CR_InitStruct->PA_H_TH_MSB = (uint8_t)(HP203B_TH_Struct->P_H_TH >> 9);	// Write MSB for upper bound of pressure
		HP203B_CR_InitStruct->PA_M_TH_LSB = (uint8_t)(HP203B_TH_Struct->P_M_TH >> 1);	// Write LSB for middle bound of pressure (0.02mbar or 2Pa per count)
		HP203B_CR_InitStruct->PA_M_TH_MSB = (uint8_t)(HP203B_TH_Struct->P_M_TH >> 9);	// Write MSB for middle bound of pressure
		HP203B_CR_InitStruct->PA_L_TH_LSB = (uint8_t)(HP203B_TH_Struct->P_L_TH >> 1);	// Write LSB for lower bound of pressure (0.02mbar or 2Pa per count)
		HP203B_CR_InitStruct->PA_L_TH_MSB = (uint8_t)(HP203B_TH_Struct->P_L_TH >> 9);	// Write MSB for lower bound of pressure
	}
	else {																				// else if P_or_A = 1 (altitude)
		HP203B_CR_InitStruct->PA_H_TH_LSB = (uint8_t)(HP203B_TH_Struct->A_H_TH);		// Write LSB for upper bound of altitude (1m per count)
		HP203B_CR_InitStruct->PA_H_TH_MSB = (uint8_t)(HP203B_TH_Struct->A_H_TH >> 8);	// Write MSB for upper bound of altitude
		HP203B_CR_InitStruct->PA_M_TH_LSB = (uint8_t)(HP203B_TH_Struct->A_M_TH);		// Write LSB for middle bound of altitude (1m per count)
		HP203B_CR_InitStruct->PA_M_TH_MSB = (uint8_t)(HP203B_TH_Struct->A_M_TH >> 8);	// Write MSB for middle bound of altitude
		HP203B_CR_InitStruct->PA_L_TH_LSB = (uint8_t)(HP203B_TH_Struct->A_L_TH);		// Write LSB for lower bound of altitude (1m per count)
		HP203B_CR_InitStruct->PA_L_TH_MSB = (uint8_t)(HP203B_TH_Struct->A_L_TH >> 8);	// Write MSB for lower bound of altitude
	}
	HP203B_CR_InitStruct->T_H_TH = HP203B_TH_Struct->T_H_TH;							// Write upper bound of temperature (1°C per count)
	HP203B_CR_InitStruct->T_M_TH = HP203B_TH_Struct->T_M_TH;							// Write middle bound of temperature (1°C per count)
	HP203B_CR_InitStruct->T_L_TH = HP203B_TH_Struct->T_L_TH;							// Write lower bound of temperature (1°C per count)

	HP203B_CR_InitStruct->INT_EN =  // Disable/enable interrupt (0:disable,1:enable)
				1 << PA_RDY_EN	|	// Pressure/altitude is ready to reading
				0 << T_RDY_EN	|	// Temperature is ready to reading
				0 << PA_TRAV_EN	|	// Pressure/altitude traversed the middle threshold
				0 << T_TRAV_EN	|	// Temperature traversed the middle threshold
				0 << PA_WIN_EN	|	// Pressure/altitude is outside predefined window
				0 << T_WIN_EN;		// Temperature is outside predefined window

	HP203B_CR_InitStruct->INT_CFG =	// Select pin for interrupt output (0:INT0,1:INT1)
				HP203B_TH_Struct->P_or_A << PA_MODE |	// Selects whether the event detection parameters and the interrupts registers prefixed by
														// a ‘PA_’ corresponds to the pressure or the altitude measurement (0:pressure,1:altitude)
				0 << PA_RDY_CFG	|	// Pressure/altitude is ready to reading
				0 << T_RDY_CFG	|	// Temperature is ready to reading
				0 << PA_TRAV_CFG|	// Pressure/altitude traversed the middle threshold
				0 << T_TRAV_CFG	|	// Temperature traversed the middle threshold
				0 << PA_WIN_CFG	|	// Pressure/altitude is outside predefined window
				0 << T_WIN_CFG;		// Temperature is outside predefined window

	HP203B_CR_InitStruct->INT_SRC =	// Flags that indicates interrupt status (read only)
				0 << TH_ERR		|	// Indicates that improper settings for thresholds are set (lower bound above higher bound for example)
				0 << DEV_RDY	|	// Indicates whether the HP203B is ready (in the sleep state and not performing any operation) or not (0:busy,1:ready)
				0 << PA_RDY		|	// Pressure/altitude is ready to reading
				0 << T_RDY		|	// Temperature is ready to reading
				0 << PA_TRAV	|	// Pressure/altitude traversed the middle threshold
				0 << T_TRAV		|	// Temperature traversed the middle threshold
				0 << PA_WIN		|	// Pressure/altitude is outside predefined window
				0 << T_WIN;			// Temperature is outside predefined window

	HP203B_CR_InitStruct->INT_DIR =	// Check details of traversal or window interrupt events (read only)
				0 << CMPS_EN	|	// The data compensation enabled or disabled (0:disabled,1:enabled)
				0 << P_TRAV_DIR	|	// 1 if pressure rising from low to high, 0 if pressure falling from high to low (through middle level)
				0 << T_TRAV_DIR	|	// 1 if temperature rising from low to high, 0 if temperature falling from high to low (through middle level)
				0 << P_WIN_DIR	|	// 1 if pressure above the window, 0 if pressure below the window
				0 << T_WIN_DIR;		// 1 if temperature above the window, 0 if temperature below the window

	HP203B_CR_InitStruct->PARA = 1 << CMPS_EN;	// Enable/disable data compensation (0:disable,1:enable)
}

/* PUBLIC FUNCTIONS ================================================================================================================================= */
/************************************************************
 * Soft reset
 * - executed immediately and no matter what it has doing
 * - all memories reset to their default values
 * - automatically performed power-up sequence
 ***********************************************************/
void HP203B_SoftReset(void) {
	I2CMasterBuffer[0] = SOFT_RST;	// Command
	write1byte();					// Write 1 byte
}

/************************************************************
 * Start convert
 * - start convert temperature/pressure/altitude to digital value
 * - OSR must be one of following:
					OSR4096
					OSR2048
					OSR1024
					OSR512
					OSR256
					OSR128
 * - Channel must be one of following:
 * 					PT_CHANNEL
					T_CHANNEL
 ***********************************************************/
void HP203B_StartConvert(uint8_t OSR, uint8_t Channel) {
	I2CMasterBuffer[0] = ADC_CVT | OSR<<2 | Channel;	// Command
	write1byte();										// Write 1 byte
}

/************************************************************
 * Get pressure and temperature
 * - read the temperature and the pressure values
 * - 3 bytes for temperature MSB first
 * - 3 bytes for pressure MSB first
 ***********************************************************/
void HP203B_ReadPT(void) {
	I2CMasterBuffer[0] = READ_PT; // Command
	write1byte(); // Write 1 byte
	read6byte(); // Read 6 bytes
	HP203B_Data.T = (int32_t)I2CSlaveBuffer[0]<<16 | (int32_t)I2CSlaveBuffer[1]<<8 | (int32_t)I2CSlaveBuffer[2]; // Save the temperature value to the data structure
	if(HP203B_Data.T & (1<<23))	// If temperature below 0
		HP203B_Data.T |= 0xFF<<24; // Rewrite 8 MSBs by 1
	HP203B_Data.P = (uint32_t)I2CSlaveBuffer[3]<<16 | (uint32_t)I2CSlaveBuffer[4]<<8 | (uint32_t)I2CSlaveBuffer[5]; // Save the pressure value to the data structure
}

/************************************************************
 * Get altitude and temperature
 * - read the temperature and the altitude values
 * - 3 bytes for temperature MSB first
 * - 3 bytes for altitude MSB first
 ***********************************************************/
void HP203B_ReadAT(void) {
	I2CMasterBuffer[0] = READ_AT; // Command
	write1byte(); // Write 1 byte
	read6byte(); // Read 6 bytes
	HP203B_Data.T = (int32_t)I2CSlaveBuffer[0]<<16 | (int32_t)I2CSlaveBuffer[1]<<8 | (int32_t)I2CSlaveBuffer[2]; // Save the temperature values to the data structure
	if(HP203B_Data.T & (1<<23)) // If temperature below 0
		HP203B_Data.T |= 0xFF<<24; // Rewrite 8 MSBs by 1
	HP203B_Data.H = (int32_t)I2CSlaveBuffer[3]<<16 | (int32_t)I2CSlaveBuffer[4]<<8 | (int32_t)I2CSlaveBuffer[5]; // Save the altitude values ti the data structure
	if(HP203B_Data.H & (1<<23)) // If altitude below 0
		HP203B_Data.H |= 0xFF<<24; // Rewrite 8 MSBs by 1
}

/************************************************************
 * Get pressure
 * - read the pressure value
 * - 3 bytes for pressure MSB first
 ***********************************************************/
void HP203B_ReadP(void) {
	I2CMasterBuffer[0] = READ_P; // Command
	write1byte(); // Write 1 byte
	read3byte(); // Read 3 bytes
	HP203B_Data.P = (uint32_t)I2CSlaveBuffer[0]<<16 | (uint32_t)I2CSlaveBuffer[1]<<8 | (uint32_t)I2CSlaveBuffer[2]; // Save the pressure value to the data structure
}

/************************************************************
 * Get altitude
 * - read the altitude value
 * - 3 bytes for altitude MSB first
 ***********************************************************/
void HP203B_ReadA(void) {
	I2CMasterBuffer[0] = READ_A; // Command
	write1byte(); // Write 1 byte
	read3byte(); // Read 3 bytes
	HP203B_Data.H = (int32_t)I2CSlaveBuffer[0]<<16 | (int32_t)I2CSlaveBuffer[1]<<8 | (int32_t)I2CSlaveBuffer[2]; // Save the altitude value to the data structure
	if(HP203B_Data.H & (1<<23)) // If altitude below 0
		HP203B_Data.H |= 0xFF<<24; // Rewrite 8 MSBs by 1
}

/************************************************************
 * Get temperature
 * - read the temperature value
 * - 3 bytes for temperature MSB first
 ***********************************************************/
void HP203B_ReadT(void) {
	I2CMasterBuffer[0] = READ_T; // Command
	write1byte(); // Write 1 byte
	read3byte(); // Read 3 bytes
	HP203B_Data.T = (int32_t)I2CSlaveBuffer[0]<<16 | (int32_t)I2CSlaveBuffer[1]<<8 | (int32_t)I2CSlaveBuffer[2]; // Save the temperature value to the data structure
	if(HP203B_Data.T & (1<<23)) // If temperature below 0
		HP203B_Data.T |= 0xFF<<24; // Rewrite 8 MSBs by 1
}

/************************************************************
 * Start internal calibration
 * - re-calibrate internal circuits
 * - use when environment rapidly changed
 * - this command allows increase the accurate
 * - no need execute it if environment is stable
 * - after finishes sensor enters to sleep mode
 ***********************************************************/
void HP203B_Calibration(HP203x_CR_TypeDef *HP203B_CR_Struct) {
	I2CMasterBuffer[0] = ANA_CAL; // Command
	write1byte(); // Write 1 byte
	while(1) { // Checking DEV_RDY bit of the INT_SRC register and wait until it will set to 1
	HP203B_ReadReg(HP203B_CR_Struct,INT_SRC); // Read INT_SRC register from HP203B
	if(HP203B_CR_Struct->INT_SRC & (1<<DEV_RDY)) // Check DEV_RDY bit
		break; // Break if HP203B is ready (DEV_RDY = 1)
	}
}

/************************************************************
 * Read register from HP203B to control registers structure by address
 * - read internal register of HP203B by address
 * - save it to structure by appropriate address
 ***********************************************************/
void HP203B_ReadReg(HP203x_CR_TypeDef *HP203B_CR_Struct, uint8_t address) {
	I2CMasterBuffer[0] = READ_REG | address; // Command
	write1byte(); // Write 1 byte
	read1byte(); // Read 1 byte
	*((uint8_t*)HP203B_CR_Struct + address) = I2CSlaveBuffer[0]; // Save data of register to the structure by appropriate address
}

/************************************************************
 * Write register to HP203B from control registers structure by address
 * - write data to internal register from the structure by address
 ***********************************************************/
void HP203B_WriteReg(HP203x_CR_TypeDef *HP203B_CR_Struct, uint8_t address) {
	I2CMasterBuffer[0] = WRITE_REG | address; // Command
	I2CMasterBuffer[1] = *((uint8_t*)HP203B_CR_Struct + address); // Data
	write2byte(); // Write 2 bytes
}

/************************************************************
 * Read all registers from HP203B to control registers structure
 ***********************************************************/
void HP203B_ReadAllReg(HP203x_CR_TypeDef *HP203B_CR_Struct) {
	uint8_t address;
	for(address = ALT_OFF_LSB; address <= PARA; address++) // For all registers
		HP203B_ReadReg(HP203B_CR_Struct, address); // Save their to the structure one by one
}

/************************************************************
 * Write all registers from the structure to HP203B
 ***********************************************************/
void HP203B_WriteAllReg(HP203x_CR_TypeDef *HP203B_CR_Struct) {
	uint8_t address;
	for(address = ALT_OFF_LSB; address <= INT_SRC; address++) // For addresses from LT_OFF_LSB to INT_SRC
		HP203B_WriteReg(HP203B_CR_Struct, address); // Write data from the structure to appropriate register by address
	HP203B_WriteReg(HP203B_CR_Struct, PARA); // Write data from the structure to PARA register
}

/* Initialization sequence for HP203B sensor */
void HP203B_Init(HP203x_CR_TypeDef *HP203B_CR_Struct, HP203x_TH_TypeDef *HP203B_TH_Struct) {
	CSB_GPIO->FIODIR |= 1<<CSB_PIN;							// Set CSB pin as output
#if CSB_LEVEL
	CSB_GPIO->FIOSET |= 1<<CSB_PIN;							// Pull down CSB pin
#else
	CSB_GPIO->FIOCLR |= 1<<CSB_PIN;							// Pull up CSB pin
#endif
	INT0_GPIO->IO0IntEnR |= 1<<INT0_PIN;					// Enable rise interrupt for INT0
//	INT1_GPIO->IO0IntEnR |= 1<<INT1_PIN;					// Enable rise interrupt for INT1
	while(1) {												// Checking DEV_RDY bit of the INT_SRC register and wait until it will set to 1
		HP203B_ReadReg(HP203B_CR_Struct,INT_SRC);			// Read INT_SRC register from HP203B
		if(HP203B_CR_Struct->INT_SRC & (1<<DEV_RDY))		// Check DEV_RDY bit
			break;											// Break if HP203B is ready (DEV_RDY = 1)
	}
	HP203B_CR_StructInit(HP203B_CR_Struct, HP203B_TH_Struct);// Configure the structure of HP203B registers
	HP203B_WriteAllReg(HP203B_CR_Struct);					// Copy this structure into internal registers of HP203B
}
