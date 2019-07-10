//**********************************************************************//
//
//	Program for Enviro-Sens board
//	Author: Jacob Curtis
//	Date of last revision: 1/8/2018
//
//**********************************************************************//

#include "stm32f1xx.h"
#include "jacob_serial.h"
#include "jacob_LCD_drawing.h"
#include <stdint.h>

#define pressure 0xEC					// MS5637 pressure sensor i2c base address
#define pressure_reset 0x1E				// Reset command
#define pressure_adc_read 0x00			// ADC read command
#define pressure_prom_read 0xA2			// PROM read base value
#define pressure_convert_pres 0x48		// D1 (pressure) conversion, OSR = 4096
#define pressure_convert_temp 0x58		// D2 (temp) conversion, OSR = 4096

#define humidity 0x80					// si7021 humidity sensor i2c base address
#define humidity_reset 0xFE				// Reset command
#define humidity_read_hum 0xE5			// Read humidity (hold mode)
#define humidity_read_tmp 0xE3			// Read temperature (hold mode)
#define humidity_read_tmp_easy 0xE0		// Read temp from last humidity measurement

#define pressure_C1 0xA274
#define pressure_C2 0xA511
#define pressure_C3 0x5E5E
#define pressure_C4 0x6071
#define pressure_C5 0x78B8
#define pressure_C6 0x6541

#define pressure_TREF 0x78B800			// Factory-calibrated reference temp
#define pressure_TEMPSENS 324			// Factory-calibrated temp coefficient of temperature
#define pressure_OFF_T1 5538709504		// Pressure offset
#define pressure_SENS_T1 2725511168
#define pressure_TCO 386
#define pressure_TCS 189

extern uint8_t buffer[];				//gLCD data buffer
static uint16_t pressure_prom[5];		//prom values 1-6
static uint8_t *pressure_data;
static uint8_t *humidity_data;

static int pressure_temp;				//pressure sensor digital temp reading
static int pressure_pressure;			//pressure sensor pressure reading
static int dT;							//difference between actual and ref temp
static int64_t pressure_OFFSET;			//offset at actual temperature
static int64_t pressure_SENS;			//sensitivity at actual temperature
static int pressure_final;				//Pressure sensor calculated barometric pressure
static int ptemp_final;					//Pressure sensor calculated temperature

static int rh;			//Humidity sensor %Relative Humidity
static uint32_t htemp;		//Humidity sensor temperature in celsius
static int rh_reading;		//Calculated %relative humidity
static int htemp_reading;		//Humidity sensor calculated temperature
static uint32_t htemp_temp;		//temporary variable
static uint32_t temp_conv;		//variable for fahrenheit-converted temperature


uint8_t str1[] = {"Environmental data"};
uint8_t str2[] = {"Temperature: "};
uint8_t str3[] = {"Humidity: "};
uint8_t str4[] = {"Pressure: "};
uint8_t buf[5] = {'0','0','0','0','0','0'};

void pressure_init()
{
	//Prepare pressure sensor for interfacing
	static uint8_t *pressure_prom_reading;		//temp variable for prom read

	//Step 1. Send reset command.
	i2c_write(pressure,pressure_reset);

	//Step 2. Read PROM for calibration data. (A0 - AE)

	for(int i = 0; i < 6; i++) {
		pressure_prom_reading = i2c_read_16bit_register(pressure, (pressure_prom_read + (2*i)));
		pressure_prom[i] = ((*(pressure_prom_reading) << 8) | *(pressure_prom_reading + 1));
	}
}

void pressure_calculate_temperature()
{
	//Step 1. Read digital temperature value (D2, OSR = 4096)
	pressure_data = i2c_read_24bit_register(pressure,pressure_convert_temp,pressure_adc_read);

	//Step 2. Calculate difference between actual and reference temp
	// (dT = pressure_temp - pressure_TREF )
	pressure_temp = ((*pressure_data << 16) | (*(pressure_data+1) << 8) | *(pressure_data+2));
	//pressure_temp -= pressure_TREF;		// this is dT
	dT = pressure_temp - pressure_TREF;


	//Step 3. Calculate actual temperature
	ptemp_final = (2000 + (dT / pressure_TEMPSENS));	//hex value of temp

}

void pressure_calculate_pressure()
{
	//Step 0. Get pressure reading
	pressure_data = i2c_read_24bit_register(pressure,pressure_convert_pres,pressure_adc_read);
	pressure_pressure = ((*pressure_data << 16) | (*(pressure_data+1) << 8) | *(pressure_data+2));

	//Step 1. Calculate offset at actual temperature
	//	(OFF = OFF_T1 + TCO * dT)
	pressure_OFFSET = pressure_OFF_T1 + (pressure_TCO * dT);

	//Step 2. Calculate sensitivity at actual temperature
	pressure_SENS = pressure_SENS_T1 + (pressure_TCS * dT);
	pressure_SENS /= 2097152;

	//Step 3. Calculate temperature compensated pressure
	pressure_final = (pressure_pressure * pressure_SENS - pressure_OFFSET) / 32768;

}

void humidity_calculate_humidity()
{
	//Step 1. Get relative humidity reading from sensor
	humidity_data = i2c_read_16bit_register(humidity, humidity_read_hum);
	rh_reading = ((*(humidity_data) << 8) | *(humidity_data+1));

	//Step 2. Calculate %RH
	rh = ((125 * rh_reading) / 65536) - 6;

	//printstring(6,0,"rh = ");
	//printbyte(6,30, rh >> 8);
	//printbyte(6,42, rh);

}

void humidity_calculate_temperature()
{
	//Step 1. Get temperature (easy) reading from sensor
	humidity_data = i2c_read_16bit_register(humidity, humidity_read_tmp_easy);
	htemp_reading = ((*(humidity_data) << 8) | *(humidity_data+1));

	//Step 2. Calculate temperature
	//htemp = ((176 * htemp_reading) / 65536) - 47;
	htemp = ((17572 * htemp_reading) / 65536) - 4585;

	/*
	printbyte(3,0,htemp_reading >> 8);
	printbyte(3,12,htemp_reading);

	htemp_temp = ((17572 * htemp_reading) / 65536) - 4685;
	printbyte(3,30,htemp_temp >> 8);
	printbyte(3,42,htemp_temp);
	*/

}

uint32_t conv_f(uint32_t value)
{

	return ((value * 9 / 5) + 3200);

}



void disp_decimal(uint32_t value)
{
	//example: 0x2C = 44 = 0010 1100... 8 bits

	uint32_t count = value;

	//reset buf values
	for(int i = 0; i<6; i++) {
		buf[i]='0';
	}

	for(int i = 0; i < count; i++) {
		buf[0]++;

		if(buf[0] > '9') {
			buf[0] = '0';
			buf[1]++;
		}
		if(buf[1] > '9') {
			buf[1] = '0';
			buf[2]++;
		}
		if(buf[2] > '9') {
			buf[2] = '0';
			buf[3]++;
		}
		if(buf[3] > '9') {
			buf[3] = '0';
			buf[4]++;
		}
		if(buf[4] > '9') {
			buf[4] = '0';
			buf[5]++;
		}
	}
}




int main(void)
{
	peripheral_init(buffer);
	pressure_init();

	while(1) {

	pressure_calculate_temperature();
	pressure_calculate_pressure();
	humidity_calculate_humidity();
	humidity_calculate_temperature();



	printstring(0,0,"Pressure sensor:");

	printstring(1,0,"P: ");

	disp_decimal(pressure_final);
	if(buf[5] > '0'){
		printchar(1,18,buf[5]);
	}
	printchar(1,24,buf[4]);
	printchar(1,30,buf[3]);
	printchar(1,36,buf[2]);
	printchar(1,42,'.');
	printchar(1,48,buf[1]);
	printchar(1,54,buf[0]);
	printstring(1,66,"mbar");


	disp_decimal(ptemp_final);
	printstring(2,0,"T:    .   C /   .   F");

	printchar(2,24,buf[3]);
	printchar(2,30,buf[2]);
	printchar(2,42,buf[1]);
	printchar(2,48,buf[0]);

	temp_conv = conv_f(ptemp_final);
	disp_decimal(temp_conv);

	printchar(2,84,buf[3]);
	printchar(2,90,buf[2]);
	printchar(2,102,buf[1]);
	printchar(2,108,buf[0]);
	printchar(2,114,248);




	printstring(4,0,"Humidity Sensor:");

	disp_decimal(rh);

	printstring(5,0,"RH: ");
	printchar(5,24,buf[1]);
	printchar(5,30,buf[0]);
	printchar(5,36,'%');



	disp_decimal(htemp);
	printstring(6,0,"T:    .   C /   .   F");

	printchar(6,24,buf[3]);
	printchar(6,30,buf[2]);
	printchar(6,42,buf[1]);
	printchar(6,48,buf[0]);

	temp_conv = conv_f(htemp);
	disp_decimal(temp_conv);

	printchar(6,84,buf[3]);
	printchar(6,90,buf[2]);
	printchar(6,102,buf[1]);
	printchar(6,108,buf[0]);
	printchar(6,114,248);




	write_buffer(buffer);


	}


	/*
	pressure_data = i2c_read_24bit_register(pressure,0x50,pressure_adc_read);

	printstring(0,0,"D2(OSR=256):");
	printbyte(0,84,*(pressure_data+0));
	printbyte(0,98,*(pressure_data+1));
	printbyte(0,112,*(pressure_data+2));

	pressure_data = i2c_read_24bit_register(pressure,0x52,pressure_adc_read);

	printstring(1,0,"D2(OSR=512):");
	printbyte(1,84,*(pressure_data+0));
	printbyte(1,98,*(pressure_data+1));
	printbyte(1,112,*(pressure_data+2));

	pressure_data = i2c_read_24bit_register(pressure,0x54,pressure_adc_read);

	printstring(2,0,"D2(OSR=1024):");
	printbyte(2,84,*(pressure_data+0));
	printbyte(2,98,*(pressure_data+1));
	printbyte(2,112,*(pressure_data+2));

	pressure_data = i2c_read_24bit_register(pressure,0x56,pressure_adc_read);

	printstring(3,0,"D2(OSR=2048):");
	printbyte(3,84,*(pressure_data+0));
	printbyte(3,98,*(pressure_data+1));
	printbyte(3,112,*(pressure_data+2));




	pressure_data = i2c_read_24bit_register(pressure,0x5A,pressure_adc_read);

	printstring(5,0,"D2(OSR=8192):");
	printbyte(5,84,*(pressure_data+0));
	printbyte(5,98,*(pressure_data+1));
	printbyte(5,112,*(pressure_data+2));


	printstring(6,0,"Pres. offset = ");
	printbyte(6,90,(pressure_prom[1] >> 8));
	printbyte(6,104,pressure_prom[1]);
	*/


	//printstring(0,0,"made it here");




	//temp = humidity_firmware(humidity,0x84,0xB8);
	//printbyte(0,0,temp);



	/*

	i2c_write(pressure,pressure_prom_read);
	pressure_data = i2c_read_two_bytes(pressure);
	printstring(2,0,"Pressure reading:");
	printbyte(3,0,*(pressure_data+0));
	printbyte(3,14,*(pressure_data+1));
	write_buffer(buffer);
	 */





}
