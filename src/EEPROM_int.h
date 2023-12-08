#ifndef EEPROM_int_h
#define EEPRIM_int_h  

#include <EEPROM.h>
#include <Arduino.h>

void writeIntIntoEEPROM(int address, int number);
int readIntFromEEPROM(int address);

#endif