#pragma once

#include "printf.h"

#define SER_EOL "\r\n"

void serial_writestr(const char *str);
void serial_putchar(char c);
void serial_printfln_ts(const char* format, ...);
#define serial_printf(...) printf(__VA_ARGS__)
#define serial_printfln(format, ...) printf(format SER_EOL, ##__VA_ARGS__)
#define serial_print(str) serial_writestr(str)
#define serial_println(str) (serial_writestr(str), serial_writestr(SER_EOL))
int serial_available();
char serial_getchar();
void start_receive();

#define LOG(fmt, ...) serial_printfln_ts(fmt, ##__VA_ARGS__)
