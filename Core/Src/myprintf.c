/*
 * myprintf.c
 *
 *  Created on: May 7, 2025
 *      Author: macos
 */


#include "myprintf.h"
#include <string.h>
#include <stdarg.h>
#include "stm32f4xx_hal.h"
#include <stdio.h>

extern UART_HandleTypeDef huart2;
void myprintf(const char *fmt, ...) {
    static char buffer[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    int len = strlen(buffer);
    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, HAL_MAX_DELAY);
}
