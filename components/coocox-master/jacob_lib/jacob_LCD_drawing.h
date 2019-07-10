#include "stm32f1xx.h"


/*****	TEXT *****/
//void printchar(uint8_t line, uint8_t x, uint8_t c, uint8_t *font, uint8_t *buff);
//void printstring(uint8_t line, uint8_t x, uint8_t *c, uint8_t *font, uint8_t *buff);
void printchar(uint8_t line, uint8_t x, uint8_t c);
void printstring(uint8_t line, uint8_t x, uint8_t *c);
uint8_t ascii_lower_nibble(uint8_t number);
uint8_t ascii_upper_nibble(uint8_t number);
//void printbyte(uint8_t line, uint8_t x, uint8_t number, uint8_t *font, uint8_t *buff);
void printbyte(uint8_t line, uint8_t x, uint8_t number);

/*****	SHAPES *****/
void drawline(uint8_t *buff, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1);
void drawcircle(uint8_t *buff,uint8_t x0, uint8_t y0, uint8_t r);
void fillcircle(uint8_t *buff, uint8_t x0, uint8_t y0, uint8_t r);
void draw_heart(uint8_t *font, uint8_t *buff);

/*****	MISC *****/
void setpixel(uint8_t *buff, uint8_t x, uint8_t y);


