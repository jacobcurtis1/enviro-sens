/*----------------------------------------------------------
* This file contains functions that are used for 
* editing the on-screen contents of the LCD.
* -----------------------------------------------------------*/

#include "stm32f1xx.h"
#include "jacob_LCD_drawing.h"
#include <stdlib.h>

uint8_t buffer[1024] = {
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,

0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x3, 0x7, 0xF, 0x1F, 0x1F, 0x3F, 0x3F, 0x3F, 0x3F, 0x7, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x7F, 0x3F, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x1F, 0x3F, 0x70, 0x70, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x6, 0x6, 0x0, 0x0, 0x0, 0x3, 0x3,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,

0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0xF, 0x7, 0x7,
0x7, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x3E, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xF, 0x3F,
0x70, 0x60, 0x60, 0x60, 0x60, 0x30, 0x7F, 0x3F, 0x0, 0x0, 0x1F, 0x3F, 0x70, 0x60, 0x60, 0x60,
0x60, 0x39, 0xFF, 0xFF, 0x0, 0x6, 0x1F, 0x39, 0x60, 0x60, 0x60, 0x60, 0x30, 0x3F, 0x7F, 0x0,
0x0, 0x60, 0xFF, 0xFF, 0x60, 0x60, 0x0, 0x7F, 0x7F, 0x70, 0x60, 0x60, 0x40, 0x0, 0x7F, 0x7F,
0x0, 0x0, 0x0, 0x0, 0x7F, 0x7F, 0x0, 0x0, 0x0, 0x7F, 0x7F, 0x0, 0x0, 0x60, 0xFF, 0xFF,
0x60, 0x60, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,

0x80, 0xF8, 0xFC, 0xFE, 0xFE, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xEF, 0xE7, 0xE7, 0xE3,
0xF3, 0xF9, 0xFF, 0xFF, 0xFF, 0xF7, 0x7, 0x1F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x7F, 0xFF,
0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x7F, 0x3F, 0x3F, 0x1F, 0xF, 0x7, 0x3, 0x0, 0x0, 0x0, 0xC0,
0xE0, 0x60, 0x20, 0x20, 0x60, 0xE0, 0xE0, 0xE0, 0x0, 0x0, 0x80, 0xC0, 0xE0, 0x60, 0x20, 0x60,
0x60, 0xE0, 0xE0, 0xE0, 0x0, 0x0, 0x80, 0xC0, 0x60, 0x60, 0x20, 0x60, 0x60, 0xE0, 0xE0, 0x0,
0x0, 0x0, 0xE0, 0xE0, 0x0, 0x0, 0x0, 0xE0, 0xE0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x80, 0xE0,
0x60, 0x60, 0x60, 0x60, 0xE0, 0x80, 0x0, 0x0, 0x0, 0xE0, 0xE0, 0x0, 0x0, 0x0, 0xE0, 0xE0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,

0x0, 0x0, 0x0, 0x3, 0x7, 0x1F, 0x9F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD, 0xF1, 0xE3,
0xE3, 0xCF, 0xFF, 0xFF, 0xFF, 0xFF, 0xF0, 0xFC, 0x7F, 0x3F, 0x3F, 0x3F, 0x3F, 0x7F, 0xFF, 0xFF,
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFC, 0xF0, 0xE0, 0x80, 0x0, 0x0, 0x0, 0xC,
0x1C, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x7F, 0x7F, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x7, 0x7, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x1C, 0xC, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,

0x0, 0x7, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE, 0xFE, 0xFE, 0xFE, 0xFC, 0xF8,
0xF8, 0xF0, 0xFE, 0xFF, 0xFF, 0xFF, 0x7F, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x1F,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xFF,
0xFF, 0x0, 0x0, 0x0, 0xFF, 0xFF, 0xE0, 0xC0, 0xC0, 0xC0, 0xFF, 0x7F, 0x0, 0x0, 0x1E, 0x7F,
0xE1, 0xC0, 0xC0, 0xC0, 0xC0, 0x61, 0xFF, 0xFF, 0x0, 0x0, 0xFE, 0xFF, 0x1, 0x0, 0x0, 0x0,
0xFF, 0xFF, 0x0, 0x0, 0x21, 0xF9, 0xF8, 0xDC, 0xCC, 0xCF, 0x7, 0x0, 0xC0, 0xFF, 0xFF, 0xC0,
0x80, 0x0, 0xFF, 0xFF, 0xC0, 0xC0, 0x80, 0x0, 0x0, 0xFF, 0xFF, 0x0, 0x0, 0x1F, 0x7F, 0xF9,
0xC8, 0xC8, 0xC8, 0xC8, 0x79, 0x39, 0x0, 0x0, 0x71, 0xF9, 0xD8, 0xCC, 0xCE, 0x47, 0x3, 0x0,

0x0, 0x0, 0x0, 0x0, 0x80, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xF8, 0xFC, 0xFC, 0xFC, 0xFC, 0xF8, 0xF0, 0xC0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0xC0,
0xC0, 0x0, 0x0, 0x0, 0xC0, 0xC0, 0x0, 0x0, 0x0, 0x0, 0xC0, 0xC0, 0x0, 0x0, 0x0, 0x80,
0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0x80, 0xC0, 0xC0, 0x0, 0x0, 0x0, 0x80, 0xC0, 0xC0, 0xC0, 0xC0,
0xC0, 0x80, 0x0, 0x0, 0x80, 0xC0, 0xC0, 0xC0, 0xC0, 0xC0, 0x0, 0x0, 0x0, 0xC0, 0xC0, 0x0,
0x0, 0x0, 0xC0, 0x80, 0x0, 0x0, 0x0, 0x0, 0x0, 0xC0, 0xC0, 0x0, 0x0, 0x0, 0x80, 0xC0,
0xC0, 0xC0, 0xC0, 0xC0, 0x80, 0x80, 0x0, 0x0, 0x80, 0xC0, 0xC0, 0xC0, 0xC0, 0x80, 0x0, 0x0,

0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,
0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0,

};

static uint8_t stdfont[] = {
		0x0, 0x0, 0x0, 0x0, 0x0,       // Ascii 0
		  0x7C, 0xDA, 0xF2, 0xDA, 0x7C,  //ASC(01)
		  0x7C, 0xD6, 0xF2, 0xD6, 0x7C,  //ASC(02)
		  0x38, 0x7C, 0x3E, 0x7C, 0x38,
		  0x18, 0x3C, 0x7E, 0x3C, 0x18,
		  0x38, 0xEA, 0xBE, 0xEA, 0x38,
		  0x38, 0x7A, 0xFE, 0x7A, 0x38,
		  0x0, 0x18, 0x3C, 0x18, 0x0,
		  0xFF, 0xE7, 0xC3, 0xE7, 0xFF,
		  0x0, 0x18, 0x24, 0x18, 0x0,
		  0xFF, 0xE7, 0xDB, 0xE7, 0xFF,
		  0xC, 0x12, 0x5C, 0x60, 0x70,
		  0x64, 0x94, 0x9E, 0x94, 0x64,
		  0x2, 0xFE, 0xA0, 0xA0, 0xE0,
		  0x2, 0xFE, 0xA0, 0xA4, 0xFC,
		  0x5A, 0x3C, 0xE7, 0x3C, 0x5A,
		  0xFE, 0x7C, 0x38, 0x38, 0x10,
		  0x10, 0x38, 0x38, 0x7C, 0xFE,
		  0x28, 0x44, 0xFE, 0x44, 0x28,
		  0xFA, 0xFA, 0x0, 0xFA, 0xFA,
		  0x60, 0x90, 0xFE, 0x80, 0xFE,
		  0x0, 0x66, 0x91, 0xA9, 0x56,
		  0x6, 0x6, 0x6, 0x6, 0x6,
		  0x29, 0x45, 0xFF, 0x45, 0x29,
		  0x10, 0x20, 0x7E, 0x20, 0x10,
		  0x8, 0x4, 0x7E, 0x4, 0x8,
		  0x10, 0x10, 0x54, 0x38, 0x10,
		  0x10, 0x38, 0x54, 0x10, 0x10,
		  0x78, 0x8, 0x8, 0x8, 0x8,
		  0x30, 0x78, 0x30, 0x78, 0x30,
		  0xC, 0x1C, 0x7C, 0x1C, 0xC,
		  0x60, 0x70, 0x7C, 0x70, 0x60,
		  0x0, 0x0, 0x0, 0x0, 0x0,
		  0x0, 0x0, 0xFA, 0x0, 0x0,
		  0x0, 0xE0, 0x0, 0xE0, 0x0,
		  0x28, 0xFE, 0x28, 0xFE, 0x28,
		  0x24, 0x54, 0xFE, 0x54, 0x48,
		  0xC4, 0xC8, 0x10, 0x26, 0x46,
		  0x6C, 0x92, 0x6A, 0x4, 0xA,
		  0x0, 0x10, 0xE0, 0xC0, 0x0,
		  0x0, 0x38, 0x44, 0x82, 0x0,
		  0x0, 0x82, 0x44, 0x38, 0x0,
		  0x54, 0x38, 0xFE, 0x38, 0x54,
		  0x10, 0x10, 0x7C, 0x10, 0x10,
		  0x0, 0x1, 0xE, 0xC, 0x0,
		  0x10, 0x10, 0x10, 0x10, 0x10,
		  0x0, 0x0, 0x6, 0x6, 0x0,
		  0x4, 0x8, 0x10, 0x20, 0x40,
		  0x7C, 0x8A, 0x92, 0xA2, 0x7C,
		  0x0, 0x42, 0xFE, 0x2, 0x0,
		  0x4E, 0x92, 0x92, 0x92, 0x62,
		  0x84, 0x82, 0x92, 0xB2, 0xCC,
		  0x18, 0x28, 0x48, 0xFE, 0x8,
		  0xE4, 0xA2, 0xA2, 0xA2, 0x9C,
		  0x3C, 0x52, 0x92, 0x92, 0x8C,
		  0x82, 0x84, 0x88, 0x90, 0xE0,
		  0x6C, 0x92, 0x92, 0x92, 0x6C,
		  0x62, 0x92, 0x92, 0x94, 0x78,
		  0x0, 0x0, 0x28, 0x0, 0x0,
		  0x0, 0x2, 0x2C, 0x0, 0x0,
		  0x0, 0x10, 0x28, 0x44, 0x82,
		  0x28, 0x28, 0x28, 0x28, 0x28,
		  0x0, 0x82, 0x44, 0x28, 0x10,
		  0x40, 0x80, 0x9A, 0x90, 0x60,
		  0x7C, 0x82, 0xBA, 0x9A, 0x72,
		  0x3E, 0x48, 0x88, 0x48, 0x3E,
		  0xFE, 0x92, 0x92, 0x92, 0x6C,
		  0x7C, 0x82, 0x82, 0x82, 0x44,
		  0xFE, 0x82, 0x82, 0x82, 0x7C,
		  0xFE, 0x92, 0x92, 0x92, 0x82,
		  0xFE, 0x90, 0x90, 0x90, 0x80,
		  0x7C, 0x82, 0x82, 0x8A, 0xCE,
		  0xFE, 0x10, 0x10, 0x10, 0xFE,
		  0x0, 0x82, 0xFE, 0x82, 0x0,
		  0x4, 0x2, 0x82, 0xFC, 0x80,
		  0xFE, 0x10, 0x28, 0x44, 0x82,
		  0xFE, 0x2, 0x2, 0x2, 0x2,
		  0xFE, 0x40, 0x38, 0x40, 0xFE,
		  0xFE, 0x20, 0x10, 0x8, 0xFE,
		  0x7C, 0x82, 0x82, 0x82, 0x7C,
		  0xFE, 0x90, 0x90, 0x90, 0x60,
		  0x7C, 0x82, 0x8A, 0x84, 0x7A,
		  0xFE, 0x90, 0x98, 0x94, 0x62,
		  0x64, 0x92, 0x92, 0x92, 0x4C,
		  0xC0, 0x80, 0xFE, 0x80, 0xC0,
		  0xFC, 0x2, 0x2, 0x2, 0xFC,
		  0xF8, 0x4, 0x2, 0x4, 0xF8,
		  0xFC, 0x2, 0x1C, 0x2, 0xFC,
		  0xC6, 0x28, 0x10, 0x28, 0xC6,
		  0xC0, 0x20, 0x1E, 0x20, 0xC0,
		  0x86, 0x9A, 0x92, 0xB2, 0xC2,
		  0x0, 0xFE, 0x82, 0x82, 0x82,
		  0x40, 0x20, 0x10, 0x8, 0x4,
		  0x0, 0x82, 0x82, 0x82, 0xFE,
		  0x20, 0x40, 0x80, 0x40, 0x20,
		  0x2, 0x2, 0x2, 0x2, 0x2,
		  0x0, 0xC0, 0xE0, 0x10, 0x0,
		  0x4, 0x2A, 0x2A, 0x1E, 0x2,
		  0xFE, 0x14, 0x22, 0x22, 0x1C,
		  0x1C, 0x22, 0x22, 0x22, 0x14,
		  0x1C, 0x22, 0x22, 0x14, 0xFE,
		  0x1C, 0x2A, 0x2A, 0x2A, 0x18,
		  0x0, 0x10, 0x7E, 0x90, 0x40,
		  0x18, 0x25, 0x25, 0x39, 0x1E,
		  0xFE, 0x10, 0x20, 0x20, 0x1E,
		  0x0, 0x22, 0xBE, 0x2, 0x0,
		  0x4, 0x2, 0x2, 0xBC, 0x0,
		  0xFE, 0x8, 0x14, 0x22, 0x0,
		  0x0, 0x82, 0xFE, 0x2, 0x0,
		  0x3E, 0x20, 0x1E, 0x20, 0x1E,
		  0x3E, 0x10, 0x20, 0x20, 0x1E,
		  0x1C, 0x22, 0x22, 0x22, 0x1C,
		  0x3F, 0x18, 0x24, 0x24, 0x18,
		  0x18, 0x24, 0x24, 0x18, 0x3F,
		  0x3E, 0x10, 0x20, 0x20, 0x10,
		  0x12, 0x2A, 0x2A, 0x2A, 0x24,
		  0x20, 0x20, 0xFC, 0x22, 0x24,
		  0x3C, 0x2, 0x2, 0x4, 0x3E,
		  0x38, 0x4, 0x2, 0x4, 0x38,
		  0x3C, 0x2, 0xC, 0x2, 0x3C,
		  0x22, 0x14, 0x8, 0x14, 0x22,
		  0x32, 0x9, 0x9, 0x9, 0x3E,
		  0x22, 0x26, 0x2A, 0x32, 0x22,
		  0x0, 0x10, 0x6C, 0x82, 0x0,
		  0x0, 0x0, 0xEE, 0x0, 0x0,
		  0x0, 0x82, 0x6C, 0x10, 0x0,
		  0x40, 0x80, 0x40, 0x20, 0x40,
		  0x3C, 0x64, 0xC4, 0x64, 0x3C,
		  0x78, 0x85, 0x85, 0x86, 0x48,
		  0x5C, 0x2, 0x2, 0x4, 0x5E,
		  0x1C, 0x2A, 0x2A, 0xAA, 0x9A,
		  0x84, 0xAA, 0xAA, 0x9E, 0x82,
		  0x84, 0x2A, 0x2A, 0x1E, 0x82,
		  0x84, 0xAA, 0x2A, 0x1E, 0x2,
		  0x4, 0x2A, 0xAA, 0x9E, 0x2,
		  0x30, 0x78, 0x4A, 0x4E, 0x48,
		  0x9C, 0xAA, 0xAA, 0xAA, 0x9A,
		  0x9C, 0x2A, 0x2A, 0x2A, 0x9A,
		  0x9C, 0xAA, 0x2A, 0x2A, 0x1A,
		  0x0, 0x0, 0xA2, 0x3E, 0x82,
		  0x0, 0x40, 0xA2, 0xBE, 0x42,
		  0x0, 0x80, 0xA2, 0x3E, 0x2,
		  0xF, 0x94, 0x24, 0x94, 0xF,
		  0xF, 0x14, 0xA4, 0x14, 0xF,
		  0x3E, 0x2A, 0xAA, 0xA2, 0x0,
		  0x4, 0x2A, 0x2A, 0x3E, 0x2A,
		  0x3E, 0x50, 0x90, 0xFE, 0x92,
		  0x4C, 0x92, 0x92, 0x92, 0x4C,
		  0x4C, 0x12, 0x12, 0x12, 0x4C,
		  0x4C, 0x52, 0x12, 0x12, 0xC,
		  0x5C, 0x82, 0x82, 0x84, 0x5E,
		  0x5C, 0x42, 0x2, 0x4, 0x1E,
		  0x0, 0xB9, 0x5, 0x5, 0xBE,
		  0x9C, 0x22, 0x22, 0x22, 0x9C,
		  0xBC, 0x2, 0x2, 0x2, 0xBC,
		  0x3C, 0x24, 0xFF, 0x24, 0x24,
		  0x12, 0x7E, 0x92, 0xC2, 0x66,
		  0xD4, 0xF4, 0x3F, 0xF4, 0xD4,
		  0xFF, 0x90, 0x94, 0x6F, 0x4,
		  0x3, 0x11, 0x7E, 0x90, 0xC0,
		  0x4, 0x2A, 0x2A, 0x9E, 0x82,
		  0x0, 0x0, 0x22, 0xBE, 0x82,
		  0xC, 0x12, 0x12, 0x52, 0x4C,
		  0x1C, 0x2, 0x2, 0x44, 0x5E,
		  0x0, 0x5E, 0x50, 0x50, 0x4E,
		  0xBE, 0xB0, 0x98, 0x8C, 0xBE,
		  0x64, 0x94, 0x94, 0xF4, 0x14,
		  0x64, 0x94, 0x94, 0x94, 0x64,
		  0xC, 0x12, 0xB2, 0x2, 0x4,
		  0x1C, 0x10, 0x10, 0x10, 0x10,
		  0x10, 0x10, 0x10, 0x10, 0x1C,
		  0xF4, 0x8, 0x13, 0x35, 0x5D,
		  0xF4, 0x8, 0x14, 0x2C, 0x5F,
		  0x0, 0x0, 0xDE, 0x0, 0x0,
		  0x10, 0x28, 0x54, 0x28, 0x44,
		  0x44, 0x28, 0x54, 0x28, 0x10,
		  0x55, 0x0, 0xAA, 0x0, 0x55,
		  0x55, 0xAA, 0x55, 0xAA, 0x55,
		  0xAA, 0x55, 0xAA, 0x55, 0xAA,
		  0x0, 0x0, 0x0, 0xFF, 0x0,
		  0x8, 0x8, 0x8, 0xFF, 0x0,
		  0x28, 0x28, 0x28, 0xFF, 0x0,
		  0x8, 0x8, 0xFF, 0x0, 0xFF,
		  0x8, 0x8, 0xF, 0x8, 0xF,
		  0x28, 0x28, 0x28, 0x3F, 0x0,
		  0x28, 0x28, 0xEF, 0x0, 0xFF,
		  0x0, 0x0, 0xFF, 0x0, 0xFF,
		  0x28, 0x28, 0x2F, 0x20, 0x3F,
		  0x28, 0x28, 0xE8, 0x8, 0xF8,
		  0x8, 0x8, 0xF8, 0x8, 0xF8,
		  0x28, 0x28, 0x28, 0xF8, 0x0,
		  0x8, 0x8, 0x8, 0xF, 0x0,
		  0x0, 0x0, 0x0, 0xF8, 0x8,
		  0x8, 0x8, 0x8, 0xF8, 0x8,
		  0x8, 0x8, 0x8, 0xF, 0x8,
		  0x0, 0x0, 0x0, 0xFF, 0x8,
		  0x8, 0x8, 0x8, 0x8, 0x8,
		  0x8, 0x8, 0x8, 0xFF, 0x8,
		  0x0, 0x0, 0x0, 0xFF, 0x28,
		  0x0, 0x0, 0xFF, 0x0, 0xFF,
		  0x0, 0x0, 0xF8, 0x8, 0xE8,
		  0x0, 0x0, 0x3F, 0x20, 0x2F,
		  0x28, 0x28, 0xE8, 0x8, 0xE8,
		  0x28, 0x28, 0x2F, 0x20, 0x2F,
		  0x0, 0x0, 0xFF, 0x0, 0xEF,
		  0x28, 0x28, 0x28, 0x28, 0x28,
		  0x28, 0x28, 0xEF, 0x0, 0xEF,
		  0x28, 0x28, 0x28, 0xE8, 0x28,
		  0x8, 0x8, 0xF8, 0x8, 0xF8,
		  0x28, 0x28, 0x28, 0x2F, 0x28,
		  0x8, 0x8, 0xF, 0x8, 0xF,
		  0x0, 0x0, 0xF8, 0x8, 0xF8,
		  0x0, 0x0, 0x0, 0xF8, 0x28,
		  0x0, 0x0, 0x0, 0x3F, 0x28,
		  0x0, 0x0, 0xF, 0x8, 0xF,
		  0x8, 0x8, 0xFF, 0x8, 0xFF,
		  0x28, 0x28, 0x28, 0xFF, 0x28,
		  0x8, 0x8, 0x8, 0xF8, 0x0,
		  0x0, 0x0, 0x0, 0xF, 0x8,
		  0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		  0xF, 0xF, 0xF, 0xF, 0xF,
		  0xFF, 0xFF, 0xFF, 0x0, 0x0,
		  0x0, 0x0, 0x0, 0xFF, 0xFF,
		  0xF0, 0xF0, 0xF0, 0xF0, 0xF0,
		  0x1C, 0x22, 0x22, 0x1C, 0x22,
		  0x3E, 0x54, 0x54, 0x7C, 0x28,
		  0x7E, 0x40, 0x40, 0x60, 0x60,
		  0x40, 0x7E, 0x40, 0x7E, 0x40,
		  0xC6, 0xAA, 0x92, 0x82, 0xC6,
		  0x1C, 0x22, 0x22, 0x3C, 0x20,
		  0x2, 0x7E, 0x4, 0x78, 0x4,
		  0x60, 0x40, 0x7E, 0x40, 0x40,
		  0x99, 0xA5, 0xE7, 0xA5, 0x99,
		  0x38, 0x54, 0x92, 0x54, 0x38,
		  0x32, 0x4E, 0x80, 0x4E, 0x32,
		  0xC, 0x52, 0xB2, 0xB2, 0xC,
		  0xC, 0x12, 0x1E, 0x12, 0xC,
		  0x3D, 0x46, 0x5A, 0x62, 0xBC,
		  0x7C, 0x92, 0x92, 0x92, 0x0,
		  0x7E, 0x80, 0x80, 0x80, 0x7E,
		  0x54, 0x54, 0x54, 0x54, 0x54,
		  0x22, 0x22, 0xFA, 0x22, 0x22,
		  0x2, 0x8A, 0x52, 0x22, 0x2,
		  0x2, 0x22, 0x52, 0x8A, 0x2,
		  0x0, 0x0, 0xFF, 0x80, 0xC0,
		  0x7, 0x1, 0xFF, 0x0, 0x0,
		  0x10, 0x10, 0xD6, 0xD6, 0x10,
		  0x6C, 0x48, 0x6C, 0x24, 0x6C,
		  0x00, 0x60, 0x90, 0x90, 0x60,		//#248 (degrees)
		  0x0, 0x0, 0x18, 0x18, 0x0,
		  0x0, 0x0, 0x8, 0x8, 0x0,
		  0xC, 0x2, 0xFF, 0x80, 0x80,
		  0x0, 0xF8, 0x80, 0x80, 0x78,
		  0x0, 0x98, 0xB8, 0xE8, 0x48,
		  0x0, 0x3C, 0x3C, 0x3C, 0x3C,
		  0x22, 0x72, 0xFE, 0x72, 0x22,		//#255... my tree
};


// Print a character on the LCD.
// x = column to start in
// line = line to start on
// c = position of font[] to print
void printchar(uint8_t line, uint8_t startColumn, uint8_t fontCode) {
  for (uint8_t i =0; i < 5; i++) {
	buffer[startColumn + (line*128) ] = stdfont[((c*5)+i)];
    	startColumn++;
  }
}



//void drawstring(uint8_t *buff, uint8_t x, uint8_t line, uint8_t *c) {
//void printstring(uint8_t line, uint8_t x, uint8_t *c, uint8_t *font, uint8_t *buff) {
void printstring(uint8_t line, uint8_t x, uint8_t *c) {
	int i = 0;

	while (c[i] != 0) {
	//printchar(line, x, c[i], stdfont, buffer);
	printchar(line, x, c[i]);
    i++;
    x += 6; // 6 pixels wide
    if (x + 6 >= 127) {
      x = 0;    // ran out of this line
      line ++;
    }
    if (line >= 7)
      return;        // ran out of space :(
  }
}

//returns the ascii value of the lower nibble of an 8 bit number
uint8_t ascii_lower_nibble(uint8_t number)
{
	if((number & 0x0F) > 0x09)
		return (number & 0x0F) + 0x37;	//return ascii A-F
	else
		return (number & 0x0F) + 0x30;	//return ascii 0-9
}

//returns the ascii value of the upper nibble of an 8 bit number
uint8_t ascii_upper_nibble(uint8_t number)
{
	if(number > 0x9F)	//if upper digit is A-F
		return (((number & 0xF0) >> 4) & 0x0F) + 0x37; //return ascii A-F
	else
		return (((number & 0xF0) >> 4) & 0x0F) + 0x30; //return ascii 0-9

}

//Print a hex byte in converted ascii format at specified line and position
//void printbyte(uint8_t line, uint8_t x, uint8_t number, uint8_t *font, uint8_t *buff)
void printbyte(uint8_t line, uint8_t x, uint8_t number)
{
	//printchar(line,x,ascii_upper_nibble(number),stdfont,buffer);
	//printchar(line,x+6,ascii_lower_nibble(number),stdfont,buffer);
	printchar(line,x,ascii_upper_nibble(number));
	printchar(line,x+6,ascii_lower_nibble(number));
}



// bresenham's algorithm - thx wikpedia
void drawline(uint8_t *buff, uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
  uint8_t steep = abs(y1 - y0) > abs(x1 - x0);
  int tmp = 0;

  if (steep) {
    //swap(x0, y0);
	tmp = x0;
	x0 = y0;
	y0 = tmp;
    //swap(x1, y1);
	tmp = x1;
	x1 = y1;
	y1 = tmp;
  }

  if (x0 > x1) {
    //swap(x0, x1);
	tmp = x0;
	x0 = x1;
	x1 = tmp;
    //swap(y0, y1);
	tmp = y0;
	y0 = y1;
	y1 = tmp;
  }

  uint8_t dx, dy;
  dx = x1 - x0;
  dy = abs(y1 - y0);

  int8_t err = dx / 2;
  int8_t ystep;

  if (y0 < y1) {
    ystep = 1;
  } else {
    ystep = -1;}

  for (; x0<x1; x0++) {
    if (steep) {
      setpixel(buff, y0, x0);
    } else {
      setpixel(buff, x0, y0);
    }
    err -= dy;
    if (err < 0) {
      y0 += ystep;
      err += dx;
    }
  }
}

// draw a circle
void drawcircle(uint8_t *buff,uint8_t x0, uint8_t y0, uint8_t r)
{
  int8_t f = 1 - r;
  int8_t ddF_x = 1;
  int8_t ddF_y = -2 * r;
  int8_t x = 0;
  int8_t y = r;

  setpixel(buff, x0, y0+r);
  setpixel(buff, x0, y0-r);
  setpixel(buff, x0+r, y0);
  setpixel(buff, x0-r, y0);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    setpixel(buff, x0 + x, y0 + y);
    setpixel(buff, x0 - x, y0 + y);
    setpixel(buff, x0 + x, y0 - y);
    setpixel(buff, x0 - x, y0 - y);

    setpixel(buff, x0 + y, y0 + x);
    setpixel(buff, x0 - y, y0 + x);
    setpixel(buff, x0 + y, y0 - x);
    setpixel(buff, x0 - y, y0 - x);

  }
}

// draw a circle
void fillcircle(uint8_t *buff, uint8_t x0, uint8_t y0, uint8_t r)
{
  int8_t f = 1 - r;
  int8_t ddF_x = 1;
  int8_t ddF_y = -2 * r;
  int8_t x = 0;
  int8_t y = r;

  for (uint8_t i=y0-r; i<=y0+r; i++) {
    setpixel(buff, x0, i);
  }

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    for (uint8_t i=y0-y; i<=y0+y; i++) {
      setpixel(buff, x0+x, i);
      setpixel(buff, x0-x, i);
    }
    for (uint8_t i=y0-x; i<=y0+x; i++) {
      setpixel(buff, x0+y, i);
      setpixel(buff, x0-y, i);
    }
  }
}



void draw_heart(uint8_t *font, uint8_t *buff)
{
	// LEFT SIDE
	drawline(buff,64,55,35,21);
	drawline(buff,35,21,35,11);
	drawline(buff,35,11,42,3);
	drawline(buff,42,3,52,3);
	drawline(buff,52,3,64,12);
	// RIGHT SIDE
	drawline(buff,64,55,93,21);
	drawline(buff,93,21,93,11);
	drawline(buff,93,11,86,3);
	drawline(buff,86,3,76,3);
	drawline(buff,76,3,64,12);
	//ODDS AND ENDS
	setpixel(buff,64,12);
	setpixel(buff,64,55);



	// NAMES

	printstring(2,59,jc);
	//printchar(3,62,'+',stdfont,buffer);
	printstring(4,59,az);
	//printstring(2,59,jc);
	printchar(3,62,'+');
	//printstring(4,59,az);



	//******* ARROWHEAD
	drawline(buff,93,15,110,7);
	//drawline(buffer,93,14,110,6);
	drawline(buff,93,16,110,8);
	drawline(buff,110,8,105,16);
	drawline(buff,110,7,101,7);
	drawline(buff,101,7,105,16);
	//******* ARROW SHAFT
	drawline(buff,49,37,33,45);
	drawline(buff,49,36,33,44);
	drawline(buff,33,45,33,50);
	drawline(buff,33,51,42,39);
	drawline(buff,33,44,29,40);
	drawline(buff,29,40,39,41);

}

// set a single pixel
void setpixel(uint8_t *buff, uint8_t x, uint8_t y)
{
	if((x >= 128) || (y >= 64))
		return;
	//buff[x + (y/8)*128] &= ~_BV(8-(y%8));
	//buff[x + (y/8)*128] &= ~(1<<(8-(y%8)));
	buff[x + (y/8)*128] |= (1<<(7-(y%8)));
}
