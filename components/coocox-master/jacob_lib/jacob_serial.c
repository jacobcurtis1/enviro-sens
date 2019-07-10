#include "stm32f1xx.h"
#include "jacob_serial.h"


// Initialize GPIO, I2C, and SPI for interfacing
void peripheral_init(uint8_t *buff)
{
	// Enable GPIOB and I2C 2
	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	//RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	RCC->APB1ENR |= RCC_APB1ENR_I2C2EN;

	//Enable GPIOA and SPI 1
	RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

	i2c_init();
	spi_init();
	dog_initialize();
	clear_buffer(buff);
}


void i2c_init()
{
	GPIOB->CRH |= 0x0000EE00;	//PB10/11 alt output opendrain 2mhz
	// Setup I2C registers for 100khz sm
	I2C2->CR2 |= 0x08;		//Set system clk frequency to 8Mhz
	I2C2->CCR |= 0x28;		//Set 100khz scl frequency
	I2C2->TRISE |= 0x09;	//Set rise time
	I2C1->CR1 |= I2C_CR1_ACK;		//Enable ACK
	I2C2->CR1 |= I2C_CR1_PE;			// ENABLE I2C



	/*
	//Conf. GPIOB pins
	GPIOB->CRL |= 0xEE000000;			//Set PB6 and PB7 for alt function output 2MHz open drain
	
	// Setup I2C registers for 100khz sm
	I2C1->CR2 |= 0x08;		//Set system clk frequency to 8Mhz
	I2C1->CCR |= 0x28;		//Set 100khz scl frequency
	I2C1->TRISE |= 0x09;	//Set rise time
	//I2C1->CR1 |= I2C_CR1_ACK;	//Enable acknowledge
	//I2C1->CR1 |= I2C_CR1_PE;	// START I2C
	//I2C1->CR1 |= I2C_CR1_PE;			// ENABLE I2C
	*/

	/**
	// Setup I2C registers for 400khz fm
	I2C1->CR2 |= 0x08;	//8mhz system clock
	I2C1->CCR |= I2C_CCR_FS;	//select fast mode
	I2C1->CCR |= I2C_CCR_DUTY;	//select 16/9 duty cycle
	I2C1->CCR |= 0x01;	//lowest CCR for 400khz?
	I2C1->TRISE |= 0x09;
	I2C1->CR1 |= I2C_CR1_ACK;	//Enable acknowledge
	I2C1->CR1 |= I2C_CR1_PE;	// START I2C
	**/

}

//config spi parameters
void spi_init()
{

	//Conf. GPIOA pins
	GPIOA->CRL = 0xB0BB0000;	//Config SPI pins (PA4,5,7 alt output, push-pull, hi-speed)
	//GPIOA->CRL |= 0x00000030;	//PA1 = DOG_RESET = general purpose push-pull @ HI speed
	GPIOB->CRL |= 0x7;			//PB0 = DOG_RESET = general purpose push-pull @ HI speed
	GPIOA->CRL |= 0x00003000;	//PA3 = LCD_A0 = general purpose push-pull output @ HI speed

	//Config SPI registers
	SPI1->CR1 |= 0x4007;			//4mhz clock
	SPI1->CR2 |= SPI_CR2_SSOE;		//enable nss output
	SPI1->CR1 |= SPI_CR1_SPE;		//enable SPI

	// Prep dog lines for LCD interface
	GPIOB->BSRR |= GPIO_BSRR_BS0;		//dog reset hi pulse (PB0)
	GPIOA->BSRR |= GPIO_BSRR_BS3;		//dog_a0 hi pulse (PA3)
}

//Declare an i2c write to specified device (address) and register (data)
void i2c_write(uint8_t address, uint8_t data)
{

	//I2C1->CR1 &= ~(I2C_CR1_ACK);	//Disable ACKing
	I2C2->CR1 |= I2C_CR1_START;		//Generate start condition

	//Wait for SB = 1
	while(!(I2C2->SR1 & I2C_SR1_SB)) {
	}

	I2C2->DR = address;	//Send device address + write

	//Wait for ADDR = 1
	while(!(I2C2->SR1 & I2C_SR1_ADDR)) {
	}

	//Read SR2 to clear addr.
	while(!(I2C2->SR2 & I2C_SR2_MSL))
	{}

	I2C2->DR = data;		//Send register address or data

	I2C2->CR1 |= I2C_CR1_STOP;

}

//Send i2c write signal to slave plus two data bytes
void i2c_write_2(uint8_t address, uint8_t data1, uint8_t data2)
{

	I2C2->CR1 |= I2C_CR1_START;		//Generate start condition

	//Wait for SB = 1
	while(!(I2C2->SR1 & I2C_SR1_SB)) {
	}

	I2C2->DR = address;	//Send device address + write

	//Wait for ADDR = 1
	while(!(I2C2->SR1 & I2C_SR1_ADDR)) {
	}

	//Wait for transmit register empty
	//while(!(I2C1->SR1 & I2C_SR1_TXE))
	//{}

	//Read SR2 to clear addr.
	while(!(I2C2->SR2 & I2C_SR2_MSL))
	{}

	I2C2->DR = data1;		//Send first data

	//wait for transmit buffer empty
	while(!(I2C2->SR1 & I2C_SR1_BTF))
	{}

	I2C2->DR = data2;		//Send second data

}

//Reads a SINGLE BYTE from target device
uint8_t i2c_read_byte(uint8_t address)
{
	//I2C1->CR1 |= I2C_CR1_ACK;		//Enable ACK
	I2C2->CR1 |= I2C_CR1_START;		//Generate start condition

	//Wait for SB = 1
	while(!(I2C2->SR1 & I2C_SR1_SB))
	{}

	I2C2->DR = (address |= 0x1);	//Send device address + read

	//Wait for ADDR = 1
	while(!(I2C2->SR1 & I2C_SR1_ADDR))
	{}

	//Read SR2 to clear addr.
	while(!(I2C2->SR2 & I2C_SR2_MSL))
	{}

	I2C2->CR1 &= ~(I2C_CR1_ACK);	//Send nak
	I2C2->CR1 |= I2C_CR1_STOP;		//Send stop condition

	//Wait for data to arrive in DR
	while(!(I2C2->SR1 & I2C_SR1_RXNE))
	{}

	return(I2C2->DR);
}

//Reads MULTIPLE BYTES from target device
uint8_t *i2c_read_two_bytes(uint8_t address)
{
	static uint8_t bytes2[1];

	I2C2->CR1 |= I2C_CR1_POS;
	I2C2->CR1 |= I2C_CR1_ACK;
	I2C2->CR1 |= I2C_CR1_START;		//Generate start condition

	//Wait for SB = 1
	while(!(I2C2->SR1 & I2C_SR1_SB))
	{}

	I2C2->DR = (address |= 0x1);	//Send device address + read

	//Wait for ADDR = 1
	while(!(I2C2->SR1 & I2C_SR1_ADDR))
	{}

	//Read SR2 to clear addr.
	while(!(I2C2->SR2 & I2C_SR2_MSL))
	{}

	I2C2->CR1 &= ~(I2C_CR1_ACK);	//Send nak

	//Wait for LSB of data to arrive in DR
	while(!(I2C2->SR1 & I2C_SR1_BTF))
	{}

	I2C2->CR1 |= I2C_CR1_STOP;		//Send stop condition

	bytes2[0] = I2C2->DR;
	bytes2[1] = I2C2->DR;

	return bytes2;
}

uint8_t *i2c_read_three_bytes(uint8_t address)
{
	static uint8_t bytes3[2];

	I2C2->CR1 |= I2C_CR1_ACK;
	I2C2->CR1 |= I2C_CR1_START;		//Generate start condition

	//Wait for SB = 1
	while(!(I2C2->SR1 & I2C_SR1_SB))
	{}

	I2C2->DR = (address |= 0x1);	//Send device address + read

	//Wait for ADDR = 1
	while(!(I2C2->SR1 & I2C_SR1_ADDR))
	{}

	while(!(I2C2->SR2 & I2C_SR2_MSL))
	{}

	while(!(I2C2->SR1 & I2C_SR1_RXNE))
	{}

	I2C2->CR1 &= ~(I2C_CR1_ACK);	//Send nak

	//Wait for byte 1 and 2 to arrive in rx buffer and shift register
	while(!(I2C2->SR1 & I2C_SR1_BTF))
	{}

	bytes3[0] = I2C2->DR;			//Store data high byte

	I2C2->CR1 |= I2C_CR1_STOP;		//Send stop condition

	bytes3[1] = I2C2->DR;			//Store data middle byte

	//Wait for LSB to arrive
	while(!(I2C2->SR1 & I2C_SR1_RXNE))
	{}

	bytes3[2] = I2C2->DR;			//Store data low byte

	return bytes3;
}



uint8_t i2c_read_register(uint8_t device_address, uint8_t register_address)
{
	I2C2->CR1 |= I2C_CR1_PE;			// ENABLE I2C
	i2c_write(device_address, register_address);
	return i2c_read_byte(device_address);
	I2C2->CR1 &= ~(I2C_CR1_PE);			//END I2C
}

uint8_t *i2c_read_16bit_register(uint8_t device_address, uint8_t register_address)
{
	I2C2->CR1 |= I2C_CR1_PE;			// ENABLE I2C
	i2c_write(device_address, register_address);
	return i2c_read_two_bytes(device_address);
	I2C2->CR1 &= ~(I2C_CR1_PE);			//END I2C
}

uint8_t *i2c_read_24bit_register(uint8_t device_address, uint8_t first_register_address, uint8_t second_register_address)
{
	I2C2->CR1 |= I2C_CR1_PE;			// ENABLE I2C
	i2c_write(device_address, first_register_address);		//start conversion command

	// Wait for conversion to complete
	for(int i = 0; i < 3; i++) {
		for(int j = 0; j <5000; j++){
			;
		}
	}

	i2c_write(device_address, second_register_address);		//ADC read command
	return i2c_read_three_bytes(device_address);

	I2C2->CR1 &= ~(I2C_CR1_PE);			//END I2C
}

uint8_t humidity_firmware(uint8_t device_address, uint8_t add1, uint8_t add2)
{
	I2C2->CR1 |= I2C_CR1_PE;			// ENABLE I2C
	i2c_write_2(device_address,add1,add2);
	return i2c_read_byte(device_address);
	I2C2->CR1 &= ~(I2C_CR1_PE);			//END I2C
}

//function to easily send desired byte via SPI
void dog_data(uint8_t data)
{
	while(!(SPI1->SR & SPI_SR_TXE)) {
		//wait until tx buffer is empty
	}
	SPI1->DR = data;	//transmit command
}

//function to send lcd command
void dog_cmd(uint8_t data)
{
	while(!(SPI1->SR & SPI_SR_TXE)) {
		//wait until tx buffer is empty
	}
	SPI1->DR = data;					//transmit command
	GPIOA->BRR |= GPIO_BRR_BR3;			//a0 low pulse
	GPIOA->BSRR |= GPIO_BSRR_BS3;		//dog_a0 hi pulse

}

int pagemap[] = { 7, 6, 5, 4, 3, 2, 1, 0 };


// initialization routine for glcd
void dog_initialize()
{
	dog_cmd(CMD_DOG_RESET);		//reset display

	dog_cmd(0x40);		//display start line 0
	/******************* ADAFRUIT DISPLAY TWEAKS *************************/
	// set adc normal and reverse com0-com63 to display adafruit style
	/*********************************************************************/
	dog_cmd(0xA1);		//adc reverse
	//dog_cmd(0xA0);	//adc normal
	//dog_cmd(0xC0);	//normal com0-com63
	dog_cmd(0xC8);		//reverse com0-63
	dog_cmd(0xA6);		//display normal
	//dog_cmd(0xA7);	//display invert

	dog_cmd(0xA2);		//set bias 1/9
	dog_cmd(0x2F);		//booster, regulator, follower on
	dog_cmd(0xF8);		//set internal booster to 4x
	dog_cmd(0x00);
	dog_cmd(0x27);		//contrast set
	dog_cmd(0x81);
	dog_cmd(0x16);	//change this # to change contrast
	//dog_cmd(0x20);

	dog_cmd(0xAC);		//no indicator
	dog_cmd(0x00);
	dog_cmd(0xAF);		//display on
}

// print the buffer
void write_buffer(uint8_t *buff) {
  uint8_t c, p;

  //for page 0-7
  for(p = 0; p < 8; p++) {

    dog_cmd(CMD_SET_PAGE | pagemap[p]);					//set page=p
    dog_cmd(CMD_SET_COLUMN_LOWER);						//set low column = 0.
    dog_cmd(CMD_SET_COLUMN_UPPER);						//set high column = 0
    dog_cmd(CMD_RMW);

    for(c = 0; c < 128; c++) {
      dog_data(buff[(128*p)+c]);
    }
  }
}

// clear everything
void clear_buffer(uint8_t *buff) {
  //memset(buff, 0, 1024);
  for(int i=0; i<1024; i++) {
	  buff[i] = 0x00;
  }
}
