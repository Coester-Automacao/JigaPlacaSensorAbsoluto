/********************************************************************************
********************************************************************************
**
** Description
** -----------
** This module deals with the Board Support Package (BSP) 
** for Jiga Absolute Position Sensor.
**
********************************************************************************
********************************************************************************
**                                                                            **
** COPYRIGHT NOTIFICATION (c) 2020 Coester Automation                         **
**                                                                            **
** This code is the property of Coester Automation.                           **
** The source code may not be reproduced, distributed, or used without        **
** permission.                                                                **
**                                                                            **
********************************************************************************
********************************************************************************
**
** Company: Coester Automation 
**          jacy Porto, 1157
**          Vincetina
**          São Leopoldo-RS
**          Tel:     (+55) (51) 40094200
**          e-mail:  info@coester.com.br
**
********************************************************************************
********************************************************************************
**
** Author: Felipe Pinz
**
** Date: April of 2021
**
** Revision: 0
**
********************************************************************************
********************************************************************************
*/

#ifndef PlacaJiga_h
#define PlacaJiga_h

#include "Arduino.h"
#include <Wire.h>


class BspJigSensAbs{
	 
	public:	
	
/*******************************************************************************
**
** Public Services Definitions
**
********************************************************************************
*/

/*------------------------------------------------------------------------------
**
** BspJigSensAbs()
**
** Is the constructor of the class BspJigSensAbs.
**
**------------------------------------------------------------------------------
**
** Inputs:
**    none      - 
**
** Outputs:
**    none      - 
**
** Usage:
**    bsp_enab_5V_ext();
**
**------------------------------------------------------------------------------
*/
	
		BspJigSensAbs();
		
/*------------------------------------------------------------------------------
**
** bsp_i2c_start()
**
** Start the I2C channel.
**
**------------------------------------------------------------------------------
**
** Inputs:
**    none      - 
**
** Outputs:
**    none      - 
**
** Usage:
**    bsp_i2c_start();
**
**------------------------------------------------------------------------------
*/
		
		void bsp_i2c_start();	
		
		

/*------------------------------------------------------------------------------
**
** bsp_enab_BOOT0()
**
** Enable the BOOT0 of the STM32.
**
**------------------------------------------------------------------------------
**
** Inputs:
**    none      - 
**
** Outputs:
**    none      - 
**
** Usage:
**    bsp_enab_BOOT0();
**
**------------------------------------------------------------------------------
*/
		void bsp_enab_BOOT0();
		
/*------------------------------------------------------------------------------
**
** bsp_disab_BOOT0()
**
** Disable the BOOT0 of the STM32.
**
**------------------------------------------------------------------------------
**
** Inputs:
**    none      - 
**
** Outputs:
**    none      - 
**
** Usage:
**    bsp_disab_BOOT0();
**
**------------------------------------------------------------------------------
*/
		void bsp_disab_BOOT0();
	


/*------------------------------------------------------------------------------
**
** bsp_enab_5V_ext()
**
** Enable 5V for powering the DUT.
**
**------------------------------------------------------------------------------
**
** Inputs:
**    none      - 
**
** Outputs:
**    none      - 
**
** Usage:
**    bsp_enab_5V_ext();
**
**------------------------------------------------------------------------------
*/
		void bsp_enab_5V_ext();


/*------------------------------------------------------------------------------
**
** bsp_disab_5V_ext();
**
** Disable 5V for powering the DUT.
**
**------------------------------------------------------------------------------
**
** Inputs:
**    none      - 
**
** Outputs:
**    none      - 
**
** Usage:
**    bsp_disab_5V_ext();
**
**------------------------------------------------------------------------------
*/
		void bsp_disab_5V_ext();

		
/*------------------------------------------------------------------------------
**
** bsp_get_3V3()
**
** Get the analog value from the 3V3 measure.
**
**------------------------------------------------------------------------------
**
** Inputs:
**    none      - 
**
** Outputs:
**    return      - Analog value from the 3V3 measure.
**
** Usage:
**    int value = bsp_get_3V3();
**
**------------------------------------------------------------------------------
*/
		uint16_t bsp_get_3V3();   
		
		
/*------------------------------------------------------------------------------
**
** bsp_get_corrente()
** medir corrente tranmitida pela placa de teste (3.8mA a 24 mA)
**
**------------------------------------------------------------------------------
**
** Inputs:
**    none      - 
**
** Outputs:
**    return      - Analog value from the from the voltage measure of the R15 resistor.
**
** Usage:
**    int value = bsp_get_corrente();
**
**------------------------------------------------------------------------------
*/
		uint16_t bsp_get_corrente();
		
		
/*------------------------------------------------------------------------------
**
** bsp_enab_bootloader_ext()
**
** Enable the bootloader mode of the external microcontroller. 
**
**------------------------------------------------------------------------------
**
** Inputs:
**    none      - 
**
** Outputs:
**    none      - 
**
** Usage:
**    bsp_enab_bootloader_ext();
**
**------------------------------------------------------------------------------
*/
		void bsp_enab_bootloader_ext();
		
		
/*------------------------------------------------------------------------------
**
** bsp_disab_bootloader_ext()
**
** Disable the bootloader mode of external microcontroller. 
**
**------------------------------------------------------------------------------
**
** Inputs:
**    none      - 
**
** Outputs:
**    none      - 
**
** Usage:
**    bsp_disab_bootloader_ext();
**
**------------------------------------------------------------------------------
*/
		void bsp_disab_bootloader_ext();
		
		
				
/*------------------------------------------------------------------------------
**
** bsp_set_led_status_green()
**
** Turn on the green status LED. 
**
**------------------------------------------------------------------------------
**
** Inputs:
**    none      - 
**
** Outputs:
**    none      - 
**
** Usage:
**    bsp_set_led_status_green();
**
**------------------------------------------------------------------------------
*/
		void bsp_set_led_status_green();
		
		
/*------------------------------------------------------------------------------
**
** bsp_clear_led_status_green()
**
** Turn off the green status LED. 
**
**------------------------------------------------------------------------------
**
** Inputs:
**    none      - 
**
** Outputs:
**    none      - 
**
** Usage:
**    bsp_clear_led_status_green();
**
**------------------------------------------------------------------------------
*/
		void bsp_clear_led_status_green();
		
/*------------------------------------------------------------------------------
**
** bsp_toggle_led_status_green()
**
** Toggle the green status LED. 
**
**------------------------------------------------------------------------------
**
** Inputs:
**    none      - 
**
** Outputs:
**    none      - 
**
** Usage:
**    bsp_toggle_led_status_green();
**
**------------------------------------------------------------------------------
*/
		void bsp_toggle_led_status_green();
		
		
/*------------------------------------------------------------------------------
**
** bsp_set_led_status_red()
**
** Turn on the red status LED. 
**
**------------------------------------------------------------------------------
**
** Inputs:
**    none      - 
**
** Outputs:
**    none      - 
**
** Usage:
**    bsp_set_led_status_red();
**
**------------------------------------------------------------------------------
*/		
		void bsp_set_led_status_red();
		
		
/*------------------------------------------------------------------------------
**
** bsp_clear_led_status_red()
**
** Turn off the red status LED. 
**
**------------------------------------------------------------------------------
**
** Inputs:
**    none      - 
**
** Outputs:
**    none      - 
**
** Usage:
**    bsp_clear_led_status_red();
**
**------------------------------------------------------------------------------
*/		
		void bsp_clear_led_status_red();


/*------------------------------------------------------------------------------
**
** bsp_i2c_set_slave_addres(uint8_t addres)
**
** Set the I2C slave addres.
**
**------------------------------------------------------------------------------
**
** Inputs:
**    addres      - The addres of the slave.
**
** Outputs:
**    none      - 
**
** Usage:
**    uint8_t byteRecebido = bsp_i2c_read();
**
**------------------------------------------------------------------------------
*/
		void bsp_i2c_set_slave_addres(uint8_t addres);
		

/*------------------------------------------------------------------------------
**
** bsp_i2c_write(uint8_t data)
**
** I2C Master writes on the external device addressed by the function "bsp_i2c_set_slave_addres". 
**
**------------------------------------------------------------------------------
**
** Inputs:
**    data      - byte to be transmmitted
**
** Outputs:
**    none      - 
**
** Usage:
**    bsp_i2c_write(0x10);
**
**------------------------------------------------------------------------------
*/	
		uint8_t bsp_i2c_write(uint8_t dados);
		
		
	/*------------------------------------------------------------------------------
**
** bsp_i2c_write(uint8_t* data, int length)
**
** I2C Master writes the array data on the external device addressed by the function "bsp_i2c_set_slave_addres". 
**
**------------------------------------------------------------------------------
**
** Inputs:
**    data      - array of bytes to be transmmitted
**	  length	- number os bytes to be transmmitted
**
** Outputs:
**    none      - 
**
** Usage:
**    bsp_i2c_write(0x10);
**
**------------------------------------------------------------------------------
*/		
		uint8_t bsp_i2c_write(uint8_t* data, int length);
		
		
/*------------------------------------------------------------------------------
**
** bsp_i2c_read(uint8_t *buf, uint8_t quantity)
**
** I2C Master reads the Slave. 
**
**------------------------------------------------------------------------------
**
** Inputs:
**    	*buf  	    - pointer of buffer to storage the receiveds bytes
**		quantity	- quantity of the bytes to be requested
** Outputs:
**    	none		-
**
** Usage:
**    	uint8_t byteRecebido = bsp_i2c_read();
**
**------------------------------------------------------------------------------
*/			
		void bsp_i2c_read(uint8_t *buf, uint8_t quantity);
		
		
/*------------------------------------------------------------------------------
**
** bsp_i2c_read(char *buf, uint8_t quantity)
**
** I2C Master reads the Slave. 
**
**------------------------------------------------------------------------------
**
** Inputs:
**    	*buf  	    - pointer of buffer to storage the receiveds bytes
**		quantity	- quantity of the bytes to be requested
** Outputs:
**    	none		-
**
** Usage:
**    	uint8_t byteRecebido = bsp_i2c_read();
**
**------------------------------------------------------------------------------
*/			
		void bsp_i2c_read(char *buf, uint8_t quantity);
				
	
	private:
/*******************************************************************************
**
** Private Services Definitions
**
********************************************************************************
*/
	
		static uint8_t i2cSlaveAddres;
		static uint8_t *bufferReceived;
		
		const int PIN_3V3 = 34;						// Pino analogico medição 3V3
		//const int MIN_3V3 = 1948;          			// Valor mínimo da medição 3V3. Div. de tensão de 3,3V para 1,65V e considerando R4 e R5 com 5% de tolerancia, pior caso (R5 9,5K e R4 10,5k). Tensão mínima 1.57 V (1948)
		//const int MAX_3V3 = 2147;            		// Valor máximo da medição 3V3. Div. de tensão de 3,3V para 1,65V e considerando R4 e R5 com 5% de tolerancia, pior caso (R5 10,5K e R4 9,5k). Tensão máxima 1.73 V  (2147)
		
		const int PIN_CORRENTE = 35;				// Pino analogico medição Resistor Shunt Laço de Corrente
		//const int MIN_CORRENTE = 558;				// Valor mínimo da medição Resistor Shunt. Considerando R15 = 120R 5% de tolerancia (114R) e corrente de 4mA  = 0,456 V
		//const int MAX_CORRENTE = 3127;				// Valor máximo da medição Resistor Shunt. Considerando R15 = 120R 5% de tolerancia (126R) e corrente de 20mA = 2,52  V
		
		const int PIN_I2C_SDA = 19;		//27 wemos	//jiga 19			// Pino I2C - SDA
		const int PIN_I2C_SCL = 21;		//5 wemos	// jiga 21			// Pino I2C - SCL
		const int I2C_FREQ = 100000;				// I2C - Frequencia
		
		const int PIN_LED_GREEN = 22;
		const int PIN_LED_RED = 23;				
		
		const int PIN_U2_TX = 17;
		const int PIN_U2_RX = 16;
		const int PIN_CTRL_BL_SEL = 14;
		
		const int PIN_EB_TX = 1;
		const int PIN_EB_RX = 3;
		const int PIN_EB_SEL = 0;
		
		const int PIN_CTRL_5V = 13;
		
		
		
	
};

#endif