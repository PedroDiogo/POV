#ifndef EEPROM_H
#define EEPROM_H

/* Function prototypes */
unsigned char crc(unsigned char crc, char crc_data);
unsigned char strcrc(char *str);

char readEEPROM(char addr);
BOOL writeEEPROM(char addr, char data);
void readStrEEPROM(char addr, unsigned char length, char *str);
BOOL writeStrEEPROM(char addr, char length, char *str);

#endif