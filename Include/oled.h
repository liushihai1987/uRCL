#ifndef OLED_H
#define OLED_H
#include "main.h"




void Column_set(unsigned char column);
void Page_set(unsigned char page);
void OLED_clear(void);
void OLED_full(void);
void OLED_init(void);
void Picture_display(const unsigned char *ptr_pic);
void Picture_ReverseDisplay(const unsigned char *ptr_pic);
void Display_int(void);
void Display_scan(uint16_t Volage , uint16_t current);
void Display_SET_scan(uint16_t V , uint16_t c);

#endif


