#include "stm32f1xx.h"

/***** LCD COMMANDS *****/
#define CMD_SET_COLUMN_UPPER  0x10
#define CMD_SET_COLUMN_LOWER  0x00
#define CMD_RMW  0xE0
#define CMD_SET_PAGE  0xB0
#define CMD_DOG_RESET 0xE2
#define CMD_CURSOR_BLINK_ON 0xAD

/***** SERIAL CONFIG *****/
void peripheral_init();
void i2c_init();
void spi_init();

/***** I2C FUNCTIONS *****/
void i2c_write(uint8_t address, uint8_t data);
uint8_t i2c_read_byte(uint8_t address);
uint8_t *i2c_read_two_bytes(uint8_t address);
uint8_t *i2c_read_three_bytes(uint8_t address);
uint8_t i2c_read_register(uint8_t device_address, uint8_t register_address);
void i2c_write_2(uint8_t address, uint8_t data1, uint8_t data2);
uint8_t humidity_firmware(uint8_t device_address, uint8_t add1, uint8_t add2);

uint8_t *i2c_read_16bit_register(uint8_t device_address, uint8_t register_address);
uint8_t *i2c_read_24bit_register(uint8_t device_address, uint8_t first_register_address, uint8_t second_register_address);

/****** LCD SPI FUNCTIONS *****/
void dog_initialize();
void dog_data(uint8_t data);
void dog_cmd(uint8_t data);
void write_buffer(uint8_t *buff);
void clear_buffer(uint8_t *buff);
