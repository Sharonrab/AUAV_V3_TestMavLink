//
//  SIL-eeprom.c
//  MatrixPilot-SIL
//
//  Created by Ben Levitt on 2/4/13.
//  Copyright (c) 2013 MatrixPilot. All rights reserved.
//

#if (WIN == 1 || NIX == 1)


#include <stdio.h>
#include <string.h>
//#include "../../libUDB/libUDB.h"
#include "libUDB.h"

#define EEPROMFilePath "EEPROM.bin"

#define EE_PAGE_SIZE   32
#define EE_PAGE_COUNT  1024
#define EE_DATA_SIZE   (EE_PAGE_SIZE * EE_PAGE_COUNT)

uint8_t EEPROMbuffer[EE_DATA_SIZE];
boolean EEPROMloaded = 0;
boolean EEPROMdirty = 0;

void loadEEPROMFileIfNeeded(void)
{
	long n;
	FILE* fp;

	if (EEPROMloaded) return;
	
	EEPROMloaded = 1;
	fp = fopen(EEPROMFilePath, "r");
	if (!fp) {
		memset(EEPROMbuffer, 0, EE_DATA_SIZE);
		return;
	}
	n = fread(EEPROMbuffer, EE_PAGE_SIZE, EE_PAGE_COUNT, fp);
	fclose(fp);
	if (n < EE_PAGE_COUNT) {
		memset(EEPROMbuffer, 0, EE_DATA_SIZE);
	}
}

void writeEEPROMFileIfNeeded(void)
{
	FILE* fp;

	if (!EEPROMdirty) return;
	
	fp = fopen(EEPROMFilePath, "w");
	if (!fp) {
		return;
	}
	fwrite(EEPROMbuffer, EE_PAGE_SIZE, EE_PAGE_COUNT, fp);
	fclose(fp);
	EEPROMdirty = 0;
}

void eeprom_ByteWrite(uint16_t address, uint8_t data)
{
	loadEEPROMFileIfNeeded();
	EEPROMbuffer[address] = data;
	EEPROMdirty = 1;
}

void eeprom_ByteRead(uint16_t address, uint8_t *data)
{
	loadEEPROMFileIfNeeded();
	*data = EEPROMbuffer[address];
	EEPROMdirty = 1;
}

void eeprom_PageWrite(uint16_t address, uint8_t *data, uint8_t numbytes)
{
	loadEEPROMFileIfNeeded();
	memcpy(EEPROMbuffer+address, data, numbytes);
	EEPROMdirty = 1;
}

void eeprom_SequentialRead(uint16_t address, uint8_t *data, uint16_t numbytes)
{
	loadEEPROMFileIfNeeded();
	memcpy(data, EEPROMbuffer+address, numbytes);
	EEPROMdirty = 1;
}

#endif // (WIN == 1 || NIX == 1)
