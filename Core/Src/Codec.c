/*
 * my_Codec.c
 *
 *  Created on: 17.06.2022
 *      Author: Davidlohner
 */
#include <Codec.h>
#include "stm32f7xx.h"

extern I2C_HandleTypeDef hi2c4;

void write_register(uint16_t register_pointer, uint16_t register_value)
{
    uint8_t data[4];

    data[0] = register_pointer>>8;  // MSB byte of 16bit register
    data[1] = register_pointer;  // LSB byte of 16bit register
    data[2] = register_value>>8;    // MSB byte of 16bit data
    data[3] = register_value;       // LSB byte of 16bit data

    HAL_I2C_Master_Transmit(&hi2c4, CODECI2CADDRESS, data, 4, 100);  // data is the start pointer of our array
}

uint16_t read_register(uint16_t register_pointer)
{
    uint8_t data[4];

    data[0] = register_pointer>>8;  // MSB byte of 16bit register
    data[1] = register_pointer;  // LSB byte of 16bit register
    data[2] = 0x00;    // MSB byte of 16bit data
    data[3] = 0x00;       // LSB byte of 16bit data

    //HAL_I2C_Master_Transmit(&hi2c4, CODECI2CADDRESS, data, 2, 100);  // data is the start pointer of our array
    HAL_I2C_Master_Receive(&hi2c4, CODECI2CADDRESS, data, 4, 100);

    return ((data[0] << 8) + data[1]);


    //HAL_I2C_Master_Transmit(&hi2c4, CODECI2CADDRESS, data, 4, 100);  // data is the start pointer of our array
}

//uint16_t read_register(uint16_t register_pointer)
//{
//	  uint16_t reg_address = register_pointer;  // Beispielregisteradresse
//	  uint8_t reg_value[2];  // 2 Byte für 16-Bit Registerwert
//
//	  // Lese ein 16-Bit Register des Audio-Codecs
//	  if(HAL_I2C_Mem_Read(&hi2c4, (uint16_t)(CODECI2CADDRESS), reg_address, I2C_MEMADD_SIZE_16BIT, reg_value, 2, HAL_MAX_DELAY) == HAL_OK)
//	  {
//	    // Erfolgreiches Lesen
//	    uint16_t value = (reg_value[0] << 8) | reg_value[1];  // 16-Bit Wert zusammensetzen
//	    return value;
//	  }
//	  else
//	  {
//	    return 0xFFFF;
//		  // Fehlerbehandlung
//	  }
//
//}


void Codec(void){

	/*Reset all Registers*/
	write_register(0x0000, 0x0000);

	volatile uint16_t temp = 0x0000;
	temp = read_register(0x0210);

	/* wm8994 Errata Work-Arounds */
	write_register(0x102, 0x0003);
	write_register(0x817, 0x0000);
	write_register(0x102, 0x0000);

	/* Enable VMID soft start (fast), Start-up Bias Current Enabled */
	write_register(0x39, 0x006C);


	/* Enable bias generator, Enable VMID */
	write_register(0x01, 0x0013);


	/* Add Delay */
	HAL_Delay(50);

	/*Headphones Out*/
    /* Disable DAC2 (Left), Disable DAC2 (Right),
    Enable DAC1 (Left), Enable DAC1 (Right)*/
	write_register(0x05, 0x0303);

    /* Enable the AIF1 Timeslot 0 (Left) to DAC 1 (Left) mixer path */
	write_register(0x601, 0x0001);

    /* Enable the AIF1 Timeslot 0 (Right) to DAC 1 (Right) mixer path */
	write_register(0x602, 0x0001);

    /* Disable the AIF1 Timeslot 1 (Left) to DAC 2 (Left) mixer path */
	write_register(0x604, 0x0000);

    /* Disable the AIF1 Timeslot 1 (Right) to DAC 2 (Right) mixer path */
	write_register(0x605, 0x0000);

	//Input Line In

    /* IN1LN_TO_IN1L, IN1LP_TO_VMID, IN1RN_TO_IN1R, IN1RP_TO_VMID */
	write_register(0x28, 0x0011);

    /* Disable mute on IN1L_TO_MIXINL and +0dB on IN1L PGA output */
	write_register(0x29, 0x0020);

    /* Disable mute on IN1R_TO_MIXINR +0dB on IN1R PGA output */

	write_register(0x2A, 0x0020);

    /* Enable AIF1ADC1 (Left), Enable AIF1ADC1 (Right)
     * Enable Left ADC, Enable Right ADC */
	write_register(0x04, 0x0303);

    /* Enable AIF1 DRC1 Signal Detect & DRC in AIF1ADC1 Left/Right Timeslot 0 */
    //counter += CODEC_IO_Write(DeviceAddr, 0x440, 0x00DB);
	write_register(0x440, 0x200);

    /* Enable IN1L and IN1R, Disable IN2L and IN2R, Enable Thermal sensor & shutdown */
	write_register(0x02, 0x6350);

    /* Enable the ADCL(Left) to AIF1 Timeslot 0 (Left) mixer path */
	write_register(0x606, 0x0002);

    /* Enable the ADCR(Right) to AIF1 Timeslot 0 (Right) mixer path */
	write_register(0x607, 0x0002);

    /* GPIO1 pin configuration GP1_DIR = output, GP1_FN = AIF1 DRC1 signal detect */
	write_register(0x700, 0x000D);

	/*Audio Frequency*/

    /* AIF1 Sample Rate = 48 (KHz), ratio=256 */
	write_register(0x210, 0x0083);

	/* AIF1 Word Length = 16-bits, AIF1 Format = I2S (Default Register Value) */
	write_register(0x300, 0x4010);


	/* slave mode */
	write_register(0x302, 0x0000);

	/* Enable the DSP processing clock for AIF1, Enable the core clock */
	write_register(0x208, 0x000A);

	/* Enable AIF1 Clock, AIF1 Clock Source = MCLK1 pin */
	write_register(0x200, 0x0001);


    /* Enable SPKRVOL PGA, Enable SPKMIXR, Enable SPKLVOL PGA, Enable SPKMIXL */
	write_register(0x03, 0x0300);

	/* Left Speaker Mixer Volume = 0dB */
	write_register(0x22, 0x0000);

	/* Speaker output mode = Class D, Right Speaker Mixer Volume = 0dB ((0x23, 0x0100) = class AB)*/
	write_register(0x23, 0x0000);

	/* Unmute DAC2 (Left) to Left Speaker Mixer (SPKMIXL) path,
	    Unmute DAC2 (Right) to Right Speaker Mixer (SPKMIXR) path */
	write_register(0x36, 0x0300);

	/* Enable bias generator, Enable VMID, Enable SPKOUTL, Enable SPKOUTR */
	write_register(0x01, 0x3003);


	 /* Enable Class W, Class W Envelope Tracking = AIF1 Timeslot 0 */
	write_register(0x51, 0x0005);

    /* Enable bias generator, Enable VMID, Enable HPOUT1 (Left) and Enable HPOUT1 (Right) input stages */
    /* idem for Speaker */
	write_register(0x01, 0x3303);

	 /* Enable HPOUT1 (Left) and HPOUT1 (Right) intermediate stages */
	write_register(0x60, 0x0022);

	/* Enable Charge Pump */
	write_register(0x4C, 0x9F25);

    /* Add Delay */
	HAL_Delay(15);

    /* Select DAC1 (Left) to Left Headphone Output PGA (HPOUT1LVOL) path */
    write_register(0x2D, 0x0001);

    /* Select DAC1 (Right) to Right Headphone Output PGA (HPOUT1RVOL) path */
    write_register(0x2E, 0x0001);

    /* Enable Left Output Mixer (MIXOUTL), Enable Right Output Mixer (MIXOUTR) */
       /* idem for SPKOUTL and SPKOUTR */
    write_register(0x03, 0x0030 | 0x0300);


    /* Enable DC Servo and trigger start-up mode on left and right channels */
    write_register(0x54, 0x0033);

    /* Add Delay */
    HAL_Delay(250);

    /* Enable HPOUT1 (Left) and HPOUT1 (Right) intermediate and output stages. Remove clamps */
    write_register(0x60, 0x00EE);

    /* Unmutes */

    /* Unmute DAC 1 (Left) */
    write_register(0x610, 0x00C0);

    /* Unmute DAC 1 (Right) */
    write_register(0x611, 0x00C0);

    /* Unmute the AIF1 Timeslot 0 DAC path */
    write_register(0x420, 0x0000);

    /* Unmute DAC 2 (Left) */
    write_register(0x612, 0x00C0);

    /* Unmute DAC 2 (Right) */
    write_register(0x613, 0x00C0);

    /* Unmute the AIF1 Timeslot 1 DAC2 path */
    write_register(0x422, 0x0000);


    /* Disable mute on IN1L, IN1L Volume = +0dB */
    write_register(0x18, 0x000B);

    /* Disable mute on IN1R, IN1R Volume = +0dB */
    write_register(0x1A, 0x000B);

    /* AIF ADC1 HPF enable, HPF cut = hifi mode fc=4Hz at fs=48kHz */
    write_register(0x410, 0x1800);



    /*Voulume Settings*/

    /* Left Headphone Volume */
    write_register(0x1C, 0x5B);

    /* Right Headphone Volume */
    write_register(0x1D, 0x5B);

    /* Left Speaker Volume */
    write_register(0x26, 0x5B);

    /* Right Speaker Volume */
    write_register(0x27, 0x5B);

    /* Left AIF1 ADC1 volume */
    write_register(0x400, 0x99);

    /* Right AIF1 ADC1 volume */
    write_register(0x401, 0x99);

    /* Left AIF1 ADC2 volume */
    write_register(0x404, 0x99);

    /* Right AIF1 ADC2 volume */
    write_register(0x405, 0x99);

    /*Unmute Codec*/

    /* Unmute the AIF1 Timeslot 0 DAC1 path L&R */
    write_register(0x420, 0x0000);

    /* Unmute the AIF1 Timeslot 1 DAC2 path L&R */
    write_register(0x422, 0x0000);


}
