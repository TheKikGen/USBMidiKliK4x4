#pragma once

// EEPROM parameters
// The signature is used to check if EEPROM is correctly initialized
// The parameters version is the version of the current structure.
// Changing the version will cause a new initialization in CheckEEPROM();.
// The following structure start at the first address of the EEPROM

#define EE_SIGNATURE "MDK"
#define EE_PRMVER 1
#define MIDI_PRODUCT_STRING_SIZE 30
#define MIDI_ROUTING_TARGET_MAX 4

typedef struct {
        uint8_t  signature[3];
        uint8_t  prmVer;
        uint8_t  midiCableRoutingTarget[MIDI_ROUTING_TARGET_MAX];
        uint8_t  midiSerialRoutingTarget[MIDI_ROUTING_TARGET_MAX];
        uint16_t vendorID;
        uint16_t productID;
        uint8_t  productString[MIDI_PRODUCT_STRING_SIZE+1];
} EEPROM_Params_t;

int EEPROM_writeBlock(uint16 ee, const uint8 *bloc, uint16 size );
int EEPROM_readBlock(uint16 ee,  uint8 *bloc, uint16 size );

