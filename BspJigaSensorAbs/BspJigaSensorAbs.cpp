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

#include "Arduino.h"
#include "BspJigaSensorAbs.h"


/*******************************************************************************
**
** Private globals variables
**
********************************************************************************
*/
uint8_t BspJigSensAbs::i2cSlaveAddres = 0x26; //0x49;


/*******************************************************************************
**
** Public Services Definitions
**
********************************************************************************
*/

/*------------------------------------------------------------------------------
** Constructor of the class
**------------------------------------------------------------------------------
*/
BspJigSensAbs::BspJigSensAbs(){
	pinMode(PIN_CTRL_BL_SEL, OUTPUT);
	digitalWrite(PIN_CTRL_BL_SEL, LOW);
	
	pinMode(PIN_CTRL_5V, OUTPUT);
	digitalWrite(PIN_CTRL_5V, LOW);
	
	pinMode(PIN_LED_GREEN, OUTPUT);
	digitalWrite(PIN_LED_GREEN, LOW);
	
	pinMode(PIN_LED_RED, OUTPUT);
	digitalWrite(PIN_LED_RED, LOW);
	
	//Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ);
	
}   

/*------------------------------------------------------------------------------
** bsp_i2c_start(uint8_t dados)
**------------------------------------------------------------------------------
*/
void 
BspJigSensAbs::bsp_i2c_start()					
{				
	Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL, I2C_FREQ);
}


/*------------------------------------------------------------------------------
** bsp_i2c_set_slave_addres(uint8_t dados)
**------------------------------------------------------------------------------
*/
void 
BspJigSensAbs::bsp_i2c_set_slave_addres(uint8_t addres)					// endereço default 0x10
{				
	i2cSlaveAddres = addres;
}


/*------------------------------------------------------------------------------
** bsp_i2c_write(uint8_t dados)
**------------------------------------------------------------------------------
*/
uint8_t 
BspJigSensAbs::bsp_i2c_write(uint8_t dados)
{	
	Wire.beginTransmission(i2cSlaveAddres);
	Wire.write(dados);
	return Wire.endTransmission();
}

/*------------------------------------------------------------------------------
** bsp_i2c_write(uint8_t dados, int length)
**------------------------------------------------------------------------------
*/
uint8_t 
BspJigSensAbs::bsp_i2c_write(uint8_t* data, int length)
{	
	Wire.beginTransmission(i2cSlaveAddres);
	Wire.write(data, length);
	return Wire.endTransmission();	
}


/*------------------------------------------------------------------------------
** bsp_i2c_read()
**------------------------------------------------------------------------------
*/
void 
BspJigSensAbs::bsp_i2c_read(uint8_t *buf, uint8_t quantity)
{	
	Wire.requestFrom(i2cSlaveAddres, quantity);
	bool isReceived = false;

	do{
		if(Wire.available()){
			isReceived = true;
			
			for(int i =0; i<quantity; i++){
				buf[i] = Wire.read();
			}
		}
	} while(!isReceived);
}



/*------------------------------------------------------------------------------
** bsp_i2c_read(char *buf, uint8_t quantity)
**------------------------------------------------------------------------------
*/
void 
BspJigSensAbs::bsp_i2c_read(char *buf, uint8_t quantity)
{	
	Wire.requestFrom(i2cSlaveAddres, quantity);
	bool isReceived = false;
	
	do{
		if(Wire.available()){
			isReceived = true;
			
			for(int i =0; i<quantity; i++){
				buf[i] = Wire.read();
			}
		}
	} while(!isReceived);
}

/*------------------------------------------------------------------------------
** bsp_enab_BOOT0()
**------------------------------------------------------------------------------
*/
void 
BspJigSensAbs::bsp_enab_BOOT0()
{
	digitalWrite(PIN_CTRL_BL_SEL, HIGH);   
}

/*------------------------------------------------------------------------------
** bsp_disab_BOOT0()
**------------------------------------------------------------------------------
*/
void 
BspJigSensAbs::bsp_disab_BOOT0()
{
	digitalWrite(PIN_CTRL_BL_SEL, LOW);
}


/*------------------------------------------------------------------------------
** bsp_enab_5V_ext()
**------------------------------------------------------------------------------
*/
void 
BspJigSensAbs::bsp_enab_5V_ext()
{
	digitalWrite(PIN_CTRL_5V, HIGH);  	
}


/*------------------------------------------------------------------------------
** bsp_disab_5V_ext()
**------------------------------------------------------------------------------
*/
void 
BspJigSensAbs::bsp_disab_5V_ext()
{
	digitalWrite(PIN_CTRL_5V, LOW);   
}


/*------------------------------------------------------------------------------
** bsp_get_3V3()
**------------------------------------------------------------------------------
*/
uint16_t 
BspJigSensAbs::bsp_get_3V3()
{
	return analogRead(PIN_3V3);
}


/*------------------------------------------------------------------------------
** bsp_get_corrente()
**------------------------------------------------------------------------------
*/
uint16_t 
BspJigSensAbs::bsp_get_corrente()
{
	return analogRead(PIN_CORRENTE);
}


/*------------------------------------------------------------------------------
** bsp_enab_bootloader_ext()
**------------------------------------------------------------------------------
*/
void 
BspJigSensAbs::bsp_enab_bootloader_ext()
{
	digitalWrite(PIN_CTRL_BL_SEL, HIGH); 
}


/*------------------------------------------------------------------------------
** bsp_disab_bootloader_ext()
**------------------------------------------------------------------------------
*/
void
BspJigSensAbs::bsp_disab_bootloader_ext()
{
	digitalWrite(PIN_CTRL_BL_SEL, LOW); 
}


/*------------------------------------------------------------------------------
** bsp_set_led_status_green()
**------------------------------------------------------------------------------
*/
void 
BspJigSensAbs::bsp_set_led_status_green()
{
	digitalWrite(PIN_LED_GREEN, HIGH);
}


/*------------------------------------------------------------------------------
** bsp_clear_led_status_green()
**------------------------------------------------------------------------------
*/
void 
BspJigSensAbs::bsp_clear_led_status_green()
{
	digitalWrite(PIN_LED_GREEN, LOW);
}

/*------------------------------------------------------------------------------
** bsp_toggle_led_status_green()
**------------------------------------------------------------------------------
*/
void 
BspJigSensAbs::bsp_toggle_led_status_green()
{
	digitalWrite(PIN_LED_GREEN, !digitalRead(PIN_LED_GREEN));
}


/*------------------------------------------------------------------------------
** bsp_set_led_status_red()
**------------------------------------------------------------------------------
*/
void 
BspJigSensAbs::bsp_set_led_status_red()
{
	digitalWrite(PIN_LED_RED, HIGH);
}


/*------------------------------------------------------------------------------
** bsp_clear_led_status_red()
**------------------------------------------------------------------------------
*/
void 
BspJigSensAbs::bsp_clear_led_status_red()
{
	digitalWrite(PIN_LED_RED, LOW);
}
