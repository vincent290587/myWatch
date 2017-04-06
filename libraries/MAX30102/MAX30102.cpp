/*
 * MAX30102.cpp
 *
 *  Created on: 27 févr. 2017
 *      Author: Vincent
 */

#include "I2C.h"
#include "Arduino.h"

#include "MAX30102.h"

#define NRF_LOG_MODULE_NAME "MAX"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"

// 7 bit address
#define MAX30102_ADDR 0x57


// Status Registers
static const uint8_t MAX30102_INTSTAT1 =		0x00;
static const uint8_t MAX30102_INTSTAT2 =		0x01;
static const uint8_t MAX30102_INTENABLE1 =		0x02;
static const uint8_t MAX30102_INTENABLE2 =		0x03;

// FIFO Registers
static const uint8_t MAX30102_FIFOWRITEPTR = 	0x04;
static const uint8_t MAX30102_FIFOOVERFLOW = 	0x05;
static const uint8_t MAX30102_FIFOREADPTR = 	0x06;
static const uint8_t MAX30102_FIFODATA =		0x07;

// Configuration Registers
static const uint8_t MAX30102_FIFOCONFIG = 		0x08;
static const uint8_t MAX30102_MODECONFIG = 		0x09;
static const uint8_t MAX30102_PARTICLECONFIG = 	0x0A;    // Note, sometimes listed as "SPO2" config in datasheet (pg. 11)
static const uint8_t MAX30102_LED1_PULSEAMP = 	0x0C;
static const uint8_t MAX30102_LED2_PULSEAMP = 	0x0D;
static const uint8_t MAX30102_LED3_PULSEAMP = 	0x0E;
static const uint8_t MAX30102_LED_PROX_AMP = 	0x10;
static const uint8_t MAX30102_MULTILEDCONFIG1 = 0x11;
static const uint8_t MAX30102_MULTILEDCONFIG2 = 0x12;

// Die Temperature Registers
static const uint8_t MAX30102_DIETEMPINT = 		0x1F;
static const uint8_t MAX30102_DIETEMPFRAC = 	0x20;
static const uint8_t MAX30102_DIETEMPCONFIG = 	0x21;

// Proximity Function Registers
static const uint8_t MAX30102_PROXINTTHRESH = 	0x30;

// Part ID Registers
static const uint8_t MAX30102_REVISIONID = 		0xFE;
static const uint8_t MAX30102_PARTID = 			0xFF;    // Should always be 0x15. Identical to MAX30102.

// MAX30102 Commands
// Interrupt configuration (pg 13, 14)
static const uint8_t MAX30102_INT_A_FULL_MASK =		(byte)~0b10000000;
static const uint8_t MAX30102_INT_A_FULL_ENABLE = 	0x80;
static const uint8_t MAX30102_INT_A_FULL_DISABLE = 	0x00;

static const uint8_t MAX30102_INT_DATA_RDY_MASK = (byte)~0b01000000;
static const uint8_t MAX30102_INT_DATA_RDY_ENABLE =	0x40;
static const uint8_t MAX30102_INT_DATA_RDY_DISABLE = 0x00;

static const uint8_t MAX30102_INT_ALC_OVF_MASK = (byte)~0b00100000;
static const uint8_t MAX30102_INT_ALC_OVF_ENABLE = 	0x20;
static const uint8_t MAX30102_INT_ALC_OVF_DISABLE = 0x00;

static const uint8_t MAX30102_INT_PROX_INT_MASK = (byte)~0b00010000;
static const uint8_t MAX30102_INT_PROX_INT_ENABLE = 0x10;
static const uint8_t MAX30102_INT_PROX_INT_DISABLE = 0x00;

static const uint8_t MAX30102_INT_DIE_TEMP_RDY_MASK = (byte)~0b00000010;
static const uint8_t MAX30102_INT_DIE_TEMP_RDY_ENABLE = 0x02;
static const uint8_t MAX30102_INT_DIE_TEMP_RDY_DISABLE = 0x00;

static const uint8_t MAX30102_SAMPLEAVG_MASK =	(byte)~0b11100000;
static const uint8_t MAX30102_SAMPLEAVG_1 = 	0x00;
static const uint8_t MAX30102_SAMPLEAVG_2 = 	0x20;
static const uint8_t MAX30102_SAMPLEAVG_4 = 	0x40;
static const uint8_t MAX30102_SAMPLEAVG_8 = 	0x60;
static const uint8_t MAX30102_SAMPLEAVG_16 = 	0x80;
static const uint8_t MAX30102_SAMPLEAVG_32 = 	0xA0;

static const uint8_t MAX30102_ROLLOVER_MASK = 	0xEF;
static const uint8_t MAX30102_ROLLOVER_ENABLE = 0x10;
static const uint8_t MAX30102_ROLLOVER_DISABLE = 0x00;

static const uint8_t MAX30102_A_FULL_MASK = 	0xF0;

// Mode configuration commands (page 19)
static const uint8_t MAX30102_SHUTDOWN_MASK = 	0x7F;
static const uint8_t MAX30102_SHUTDOWN = 		0x80;
static const uint8_t MAX30102_WAKEUP = 			0x00;

static const uint8_t MAX30102_RESET_MASK = 		0xBF;
static const uint8_t MAX30102_RESET = 			0x40;

static const uint8_t MAX30102_MODE_MASK = 		0xF8;
static const uint8_t MAX30102_MODE_REDONLY = 	0x02;
static const uint8_t MAX30102_MODE_REDIRONLY = 	0x03;
static const uint8_t MAX30102_MODE_MULTILED = 	0x07;

// Particle sensing configuration commands (pgs 19-20)
static const uint8_t MAX30102_ADCRANGE_MASK = 	0x9F;
static const uint8_t MAX30102_ADCRANGE_2048 = 	0x00;
static const uint8_t MAX30102_ADCRANGE_4096 = 	0x20;
static const uint8_t MAX30102_ADCRANGE_8192 = 	0x40;
static const uint8_t MAX30102_ADCRANGE_16384 = 	0x60;

static const uint8_t MAX30102_SAMPLERATE_MASK = 0xE3;
static const uint8_t MAX30102_SAMPLERATE_50 = 	0x00;
static const uint8_t MAX30102_SAMPLERATE_100 = 	0x04;
static const uint8_t MAX30102_SAMPLERATE_200 = 	0x08;
static const uint8_t MAX30102_SAMPLERATE_400 = 	0x0C;
static const uint8_t MAX30102_SAMPLERATE_800 = 	0x10;
static const uint8_t MAX30102_SAMPLERATE_1000 = 0x14;
static const uint8_t MAX30102_SAMPLERATE_1600 = 0x18;
static const uint8_t MAX30102_SAMPLERATE_3200 = 0x1C;

static const uint8_t MAX30102_PULSEWIDTH_MASK = 0xFC;
static const uint8_t MAX30102_PULSEWIDTH_69 = 	0x00;
static const uint8_t MAX30102_PULSEWIDTH_118 = 	0x01;
static const uint8_t MAX30102_PULSEWIDTH_215 = 	0x02;
static const uint8_t MAX30102_PULSEWIDTH_411 = 	0x03;

//Multi-LED Mode configuration (pg 22)
static const uint8_t MAX30102_SLOT1_MASK = 		0xF8;
static const uint8_t MAX30102_SLOT2_MASK = 		0x8F;
static const uint8_t MAX30102_SLOT3_MASK = 		0xF8;
static const uint8_t MAX30102_SLOT4_MASK = 		0x8F;

static const uint8_t SLOT_NONE = 				0x00;
static const uint8_t SLOT_RED_LED = 			0x01;
static const uint8_t SLOT_IR_LED = 				0x02;
static const uint8_t SLOT_GREEN_LED = 			0x03;
static const uint8_t SLOT_NONE_PILOT = 			0x04;
static const uint8_t SLOT_RED_PILOT =			0x05;
static const uint8_t SLOT_IR_PILOT = 			0x06;
static const uint8_t SLOT_GREEN_PILOT = 		0x07;

static const uint8_t MAX_30102_EXPECTEDPARTID = 0x15;

//The MAX30102 stores up to 32 samples on the IC
//This is additional local storage to the microcontroller
const int STORAGE_SIZE = 32;
struct Record
{
	uint32_t red[STORAGE_SIZE];
	uint32_t IR[STORAGE_SIZE];
	uint32_t green[STORAGE_SIZE];
	byte head;
	byte tail;
} sense; //This is our circular buffer of readings from the sensor

MAX30102::MAX30102() : I2C_Device () {
	// Constructor
	activeLEDs = 2;
}

bool MAX30102::init() {

	this->softReset();

	if (!this->begin()) {
		NRF_LOG_ERROR("Init problem\r\n");
		return false;
	}

	byte ledBrightness = 50; //Options: 0=Off to 255=50mA
	byte sampleAverage = 32; //Options: 1, 2, 4, 8, 16, 32
	byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
	int sampleRate = 1000; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
	int pulseWidth = 118; //Options: 69, 118, 215, 411
	int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

	this->setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings

	// This register sets the IR ADC count that will trigger the beginning of HR or SpO2 mode
	// 0x4 is a good value
	this->setProximityThreshold(0x04);
	this->enableAFULL();
	this->enablePROXINT();
	this->enableALCOVF();

	NRF_LOG_ERROR("INT1_STATUS register: 0x%x\r\n", this->getINT1());

	return true;
}

boolean MAX30102::begin() {

	_i2caddr = MAX30102_ADDR;

	uint8_t part_id = readPartID();
	// Step 1: Initial Communciation and Verification
	// Check that a MAX30102 is connected
	if (part_id != MAX_30102_EXPECTEDPARTID) {
		// Error -- Part ID read from MAX30102 does not match expected part ID.
		NRF_LOG_ERROR("Part ID read (0x%X) does not match\r\n", part_id);
		return false;
	}

	NRF_LOG_INFO("Device ID 0x%X\r\n", part_id);

	// Populate revision ID
	readRevisionID();

	return true;
}

//
// Configuration
//

//Begin Interrupt configuration
uint8_t MAX30102::getINT1(void) {
	return (readRegister8(_i2caddr, MAX30102_INTSTAT1));
}
uint8_t MAX30102::getINT2(void) {
	return (readRegister8(_i2caddr, MAX30102_INTSTAT2));
}

void MAX30102::enableAFULL(void) {
	bitMask(MAX30102_INTENABLE1, MAX30102_INT_A_FULL_MASK, MAX30102_INT_A_FULL_ENABLE);
}
void MAX30102::disableAFULL(void) {
	bitMask(MAX30102_INTENABLE1, MAX30102_INT_A_FULL_MASK, MAX30102_INT_A_FULL_DISABLE);
}

void MAX30102::enableDATARDY(void) {
	bitMask(MAX30102_INTENABLE1, MAX30102_INT_DATA_RDY_MASK, MAX30102_INT_DATA_RDY_ENABLE);
}
void MAX30102::disableDATARDY(void) {
	bitMask(MAX30102_INTENABLE1, MAX30102_INT_DATA_RDY_MASK, MAX30102_INT_DATA_RDY_DISABLE);
}

void MAX30102::enableALCOVF(void) {
	bitMask(MAX30102_INTENABLE1, MAX30102_INT_ALC_OVF_MASK, MAX30102_INT_ALC_OVF_ENABLE);
}
void MAX30102::disableALCOVF(void) {
	bitMask(MAX30102_INTENABLE1, MAX30102_INT_ALC_OVF_MASK, MAX30102_INT_ALC_OVF_DISABLE);
}

void MAX30102::enablePROXINT(void) {
	bitMask(MAX30102_INTENABLE1, MAX30102_INT_PROX_INT_MASK, MAX30102_INT_PROX_INT_ENABLE);
}
void MAX30102::disablePROXINT(void) {
	bitMask(MAX30102_INTENABLE1, MAX30102_INT_PROX_INT_MASK, MAX30102_INT_PROX_INT_DISABLE);
}

void MAX30102::enableDIETEMPRDY(void) {
	bitMask(MAX30102_INTENABLE2, MAX30102_INT_DIE_TEMP_RDY_MASK, MAX30102_INT_DIE_TEMP_RDY_ENABLE);
}
void MAX30102::disableDIETEMPRDY(void) {
	bitMask(MAX30102_INTENABLE2, MAX30102_INT_DIE_TEMP_RDY_MASK, MAX30102_INT_DIE_TEMP_RDY_DISABLE);
}

//End Interrupt configuration

void MAX30102::softReset(void) {
	bitMask(MAX30102_MODECONFIG, MAX30102_RESET_MASK, MAX30102_RESET);

	// Poll for bit to clear, reset is then complete
	// Timeout after 100ms
	unsigned long startTime = millis();
	while (millis() - startTime < 100)
	{
		uint8_t response = readRegister8(_i2caddr, MAX30102_MODECONFIG);
		if ((response & MAX30102_RESET) == 0) break; //We're done!
		delay(1); //Let's not over burden the I2C bus
	}
}

void MAX30102::shutDown(void) {
	// Put IC into low power mode (datasheet pg. 19)
	// During shutdown the IC will continue to respond to I2C commands but will
	// not update with or take new readings (such as temperature)
	bitMask(MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_SHUTDOWN);
}

void MAX30102::wakeUp(void) {
	// Pull IC out of low power mode (datasheet pg. 19)
	bitMask(MAX30102_MODECONFIG, MAX30102_SHUTDOWN_MASK, MAX30102_WAKEUP);
}

void MAX30102::setLEDMode(uint8_t mode) {
	// Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
	// See datasheet, page 19
	bitMask(MAX30102_MODECONFIG, MAX30102_MODE_MASK, mode);
}

void MAX30102::setADCRange(uint8_t adcRange) {
	// adcRange: one of MAX30102_ADCRANGE_2048, _4096, _8192, _16384
	bitMask(MAX30102_PARTICLECONFIG, MAX30102_ADCRANGE_MASK, adcRange);
}

void MAX30102::setSampleRate(uint8_t sampleRate) {
	// sampleRate: one of MAX30102_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
	bitMask(MAX30102_PARTICLECONFIG, MAX30102_SAMPLERATE_MASK, sampleRate);
}

void MAX30102::setPulseWidth(uint8_t pulseWidth) {
	// pulseWidth: one of MAX30102_PULSEWIDTH_69, _188, _215, _411
	bitMask(MAX30102_PARTICLECONFIG, MAX30102_PULSEWIDTH_MASK, pulseWidth);
}

// NOTE: Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
// See datasheet, page 21
void MAX30102::setPulseAmplitudeRed(uint8_t amplitude) {
	writeRegister8(_i2caddr, MAX30102_LED1_PULSEAMP, amplitude);
}

void MAX30102::setPulseAmplitudeIR(uint8_t amplitude) {
	writeRegister8(_i2caddr, MAX30102_LED2_PULSEAMP, amplitude);
}

void MAX30102::setPulseAmplitudeGreen(uint8_t amplitude) {
	writeRegister8(_i2caddr, MAX30102_LED3_PULSEAMP, amplitude);
}

void MAX30102::setPulseAmplitudeProximity(uint8_t amplitude) {
	writeRegister8(_i2caddr, MAX30102_LED_PROX_AMP, amplitude);
}

void MAX30102::setProximityThreshold(uint8_t threshMSB) {
	// Set the IR ADC count that will trigger the beginning of particle-sensing mode.
	// The threshMSB signifies only the 8 most significant-bits of the ADC count.
	// See datasheet, page 24.
	writeRegister8(_i2caddr, MAX30102_PROXINTTHRESH, threshMSB);
}

//Given a slot number assign a thing to it
//Devices are SLOT_RED_LED or SLOT_RED_PILOT (proximity)
//Assigning a SLOT_RED_LED will pulse LED
//Assigning a SLOT_RED_PILOT will ??
void MAX30102::enableSlot(uint8_t slotNumber, uint8_t device) {

	switch (slotNumber) {
	case (1):
    		  bitMask(MAX30102_MULTILEDCONFIG1, MAX30102_SLOT1_MASK, device);
	break;
	case (2):
    		  bitMask(MAX30102_MULTILEDCONFIG1, MAX30102_SLOT2_MASK, device << 4);
	break;
	case (3):
    		  bitMask(MAX30102_MULTILEDCONFIG2, MAX30102_SLOT3_MASK, device);
	break;
	case (4):
    		  bitMask(MAX30102_MULTILEDCONFIG2, MAX30102_SLOT4_MASK, device << 4);
	break;
	default:
		//Shouldn't be here!
		break;
	}
}

//Clears all slot assignments
void MAX30102::disableSlots(void) {
	writeRegister8(_i2caddr, MAX30102_MULTILEDCONFIG1, 0);
	writeRegister8(_i2caddr, MAX30102_MULTILEDCONFIG2, 0);
}

//
// FIFO Configuration
//

//Set sample average (Table 3, Page 18)
void MAX30102::setFIFOAverage(uint8_t numberOfSamples) {
	bitMask(MAX30102_FIFOCONFIG, MAX30102_SAMPLEAVG_MASK, numberOfSamples);
}

//Resets all points to start in a known state
//Page 15 recommends clearing FIFO before beginning a read
void MAX30102::clearFIFO(void) {
	writeRegister8(_i2caddr, MAX30102_FIFOWRITEPTR, 0);
	writeRegister8(_i2caddr, MAX30102_FIFOOVERFLOW, 0);
	writeRegister8(_i2caddr, MAX30102_FIFOREADPTR, 0);
}

//Enable roll over if FIFO over flows
void MAX30102::enableFIFORollover(void) {
	bitMask(MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_ENABLE);
}

//Disable roll over if FIFO over flows
void MAX30102::disableFIFORollover(void) {
	bitMask(MAX30102_FIFOCONFIG, MAX30102_ROLLOVER_MASK, MAX30102_ROLLOVER_DISABLE);
}

//Set number of samples to trigger the almost full interrupt (Page 18)
//Power on default is 32 samples
//Note it is reverse: 0x00 is 32 samples, 0x0F is 17 samples
void MAX30102::setFIFOAlmostFull(uint8_t numberOfSamples) {
	bitMask(MAX30102_FIFOCONFIG, MAX30102_A_FULL_MASK, numberOfSamples);
}

//Read the FIFO Write Pointer
uint8_t MAX30102::getWritePointer(void) {
	return (readRegister8(_i2caddr, MAX30102_FIFOWRITEPTR));
}

//Read the FIFO Read Pointer
uint8_t MAX30102::getReadPointer(void) {
	return (readRegister8(_i2caddr, MAX30102_FIFOREADPTR));
}


// Die Temperature
// Returns temp in C
float MAX30102::readTemperature() {
	// Step 1: Config die temperature register to take 1 temperature sample
	writeRegister8(_i2caddr, MAX30102_DIETEMPCONFIG, 0x01);

	// Poll for bit to clear, reading is then complete
	// Timeout after 100ms
	unsigned long startTime = millis();
	while (millis() - startTime < 100)
	{
		uint8_t response = readRegister8(_i2caddr, MAX30102_DIETEMPCONFIG);
		if ((response & 0x01) == 0) break; //We're done!
		delay(1); //Let's not over burden the I2C bus
	}
	// Step 2: Read die temperature register (integer)
	int8_t tempInt = readRegister8(_i2caddr, MAX30102_DIETEMPINT);
	uint8_t tempFrac = readRegister8(_i2caddr, MAX30102_DIETEMPFRAC);

	// Step 3: Calculate temperature (datasheet pg. 23)
	return (float)tempInt + ((float)tempFrac * 0.0625);
}

// Returns die temp in F
float MAX30102::readTemperatureF() {
	float temp = readTemperature();

	if (temp != -999.0) temp = temp * 1.8 + 32.0;

	return (temp);
}


//
// Device ID and Revision
//
uint8_t MAX30102::readPartID() {
	return readRegister8(_i2caddr, MAX30102_PARTID);
}

void MAX30102::readRevisionID() {
	revisionID = readRegister8(_i2caddr, MAX30102_REVISIONID);
}

uint8_t MAX30102::getRevisionID() {
	return revisionID;
}


//Setup the sensor
//The MAX30102 has many settings. By default we select:
// Sample Average = 4
// Mode = MultiLED
// ADC Range = 16384 (62.5pA per LSB)
// Sample rate = 50
//Use the default setup if you are just getting started with the MAX30102 sensor
void MAX30102::setup(byte powerLevel, byte sampleAverage, byte ledMode, int sampleRate, int pulseWidth, int adcRange) {
	softReset(); //Reset all configuration, threshold, and data registers to POR values

	//FIFO Configuration
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	//The chip will average multiple samples of same type together if you wish
	if (sampleAverage == 1) setFIFOAverage(MAX30102_SAMPLEAVG_1); //No averaging per FIFO record
	else if (sampleAverage == 2) setFIFOAverage(MAX30102_SAMPLEAVG_2);
	else if (sampleAverage == 4) setFIFOAverage(MAX30102_SAMPLEAVG_4);
	else if (sampleAverage == 8) setFIFOAverage(MAX30102_SAMPLEAVG_8);
	else if (sampleAverage == 16) setFIFOAverage(MAX30102_SAMPLEAVG_16);
	else if (sampleAverage == 32) setFIFOAverage(MAX30102_SAMPLEAVG_32);
	else setFIFOAverage(MAX30102_SAMPLEAVG_4);

	//setFIFOAlmostFull(0xF); //Set to 15 samples to trigger an 'Almost Full' interrupt
	enableFIFORollover(); //Allow FIFO to wrap/roll over
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

	//Mode Configuration
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	if (ledMode == 3) setLEDMode(MAX30102_MODE_MULTILED); //Watch all three LED channels
	else if (ledMode == 2) setLEDMode(MAX30102_MODE_REDIRONLY); //Red and IR
	else setLEDMode(MAX30102_MODE_REDONLY); //Red only
	activeLEDs = ledMode; //Used to control how many bytes to read from FIFO buffer
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

	//Particle Sensing Configuration
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	if(adcRange < 4096) setADCRange(MAX30102_ADCRANGE_2048); //7.81pA per LSB
	else if(adcRange < 8192) setADCRange(MAX30102_ADCRANGE_4096); //15.63pA per LSB
	else if(adcRange < 16384) setADCRange(MAX30102_ADCRANGE_8192); //31.25pA per LSB
	else if(adcRange == 16384) setADCRange(MAX30102_ADCRANGE_16384); //62.5pA per LSB
	else setADCRange(MAX30102_ADCRANGE_2048);

	if (sampleRate < 100) setSampleRate(MAX30102_SAMPLERATE_50); //Take 50 samples per second
	else if (sampleRate < 200) setSampleRate(MAX30102_SAMPLERATE_100);
	else if (sampleRate < 400) setSampleRate(MAX30102_SAMPLERATE_200);
	else if (sampleRate < 800) setSampleRate(MAX30102_SAMPLERATE_400);
	else if (sampleRate < 1000) setSampleRate(MAX30102_SAMPLERATE_800);
	else if (sampleRate < 1600) setSampleRate(MAX30102_SAMPLERATE_1000);
	else if (sampleRate < 3200) setSampleRate(MAX30102_SAMPLERATE_1600);
	else if (sampleRate == 3200) setSampleRate(MAX30102_SAMPLERATE_3200);
	else setSampleRate(MAX30102_SAMPLERATE_50);

	//The longer the pulse width the longer range of detection you'll have
	//At 69us and 0.4mA it's about 2 inches
	//At 411us and 0.4mA it's about 6 inches
	if (pulseWidth < 118) setPulseWidth(MAX30102_PULSEWIDTH_69); //Page 26, Gets us 15 bit resolution
	else if (pulseWidth < 215) setPulseWidth(MAX30102_PULSEWIDTH_118); //16 bit resolution
	else if (pulseWidth < 411) setPulseWidth(MAX30102_PULSEWIDTH_215); //17 bit resolution
	else if (pulseWidth == 411) setPulseWidth(MAX30102_PULSEWIDTH_411); //18 bit resolution
	else setPulseWidth(MAX30102_PULSEWIDTH_69);
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

	//LED Pulse Amplitude Configuration
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	//Default is 0x1F which gets us 6.4mA
	//powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
	//powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
	//powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
	//powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch

	setPulseAmplitudeRed(powerLevel);
	setPulseAmplitudeIR(powerLevel);
	//setPulseAmplitudeGreen(powerLevel);
	setPulseAmplitudeProximity(0x02);
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

	//Multi-LED Mode Configuration, Enable the reading of the three LEDs
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
	enableSlot(1, SLOT_RED_LED);
	if (ledMode > 1) enableSlot(2, SLOT_IR_LED);
	if (ledMode > 2) enableSlot(3, SLOT_GREEN_LED);
	//enableSlot(1, SLOT_RED_PILOT);
	//enableSlot(2, SLOT_IR_PILOT);
	//enableSlot(3, SLOT_GREEN_PILOT);
	//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

	clearFIFO(); //Reset the FIFO before we begin checking the sensor
}

//
// Data Collection
//

//Tell caller how many samples are available
uint8_t MAX30102::available(void)
{
	int numberOfSamples = sense.head - sense.tail;
	if (numberOfSamples < 0) numberOfSamples += STORAGE_SIZE;

	return (uint8_t) numberOfSamples;
}

void MAX30102::resetAvailable() {
	sense.head = 0;
	sense.tail = 0;
}

//Report the most recent red value
uint32_t MAX30102::getRed(void)
{
	//Check the sensor for new data for 250ms
	if(safeCheck(250))
		return (sense.red[sense.head]);
	else
		return(0); //Sensor failed to find new data
}

//Report the most recent IR value
uint32_t MAX30102::getIR(void)
{
	//Check the sensor for new data for 250ms
	if(safeCheck(250))
		return (sense.IR[sense.head]);
	else
		return(0); //Sensor failed to find new data
}

//Report the most recent Green value
uint32_t MAX30102::getGreen(void)
{
	//Check the sensor for new data for 250ms
	if(safeCheck(250))
		return (sense.green[sense.head]);
	else
		return(0); //Sensor failed to find new data
}

//Report the next Red value in the FIFO
uint32_t MAX30102::getFIFORed(void)
{
	return (sense.red[sense.tail]);
}

//Report the next IR value in the FIFO
uint32_t MAX30102::getFIFOIR(void)
{
	return (sense.IR[sense.tail]);
}

//Report the next Green value in the FIFO
uint32_t MAX30102::getFIFOGreen(void)
{
	return (sense.green[sense.tail]);
}

//Report the next Red value in the FIFO
uint32_t MAX30102::getFILORed(void)
{
  return (sense.red[sense.head]);
}

//Report the next IR value in the FIFO
uint32_t MAX30102::getFILOIR(void)
{
  return (sense.IR[sense.head]);
}

//Report the next Green value in the FIFO
uint32_t MAX30102::getFILOGreen(void)
{
  return (sense.green[sense.head]);
}


//Advance the tail
void MAX30102::nextSample(void)
{
	if(available()) //Only advance the tail if new data is available
	{
		sense.tail++;
		sense.tail %= STORAGE_SIZE; //Wrap condition
	}
}


int MAX30102::getFIFOStored() {

	int numberOfSamples = 0;

	byte readPointer = getReadPointer();
	byte writePointer = getWritePointer();

	//Calculate the number of readings we need to get from sensor
	numberOfSamples = writePointer - readPointer;
	if (numberOfSamples < 0) numberOfSamples += 32; //Wrap condition

	return numberOfSamples;
}

//Polls the sensor for new data
//Call regularly
//If new data is available, it updates the head and tail in the main struct
//Returns number of new samples obtained
int MAX30102::check(void)
{
	//Read register FIDO_DATA in (3-byte * number of active LED) chunks
	//Until FIFO_RD_PTR = FIFO_WR_PTR

	uint8_t data_buffer[I2C_BUFFER_LENGTH];

	byte readPointer = getReadPointer();
	byte writePointer = getWritePointer();

	int numberOfSamples = 0;

	//Do we have new data?
	if (readPointer != writePointer)
	{
		//Calculate the number of readings we need to get from sensor
		numberOfSamples = writePointer - readPointer;
		if (numberOfSamples < 0) numberOfSamples += 32; //Wrap condition

		//NRF_LOG_INFO("Samples available: %u\r\n", numberOfSamples);

		//We now have the number of readings, now calc bytes to read
		//For this example we are just doing Red and IR (3 bytes each)
		int bytesLeftToRead = numberOfSamples * activeLEDs * 3;

		//NRF_LOG_INFO("Bytes to read: %u\r\n", bytesLeftToRead);

		//We may need to read as many as 288 bytes so we read in blocks no larger than I2C_BUFFER_LENGTH
		//I2C_BUFFER_LENGTH changes based on the platform. 64 bytes for SAMD21, 32 bytes for Uno.
		//Wire.requestFrom() is limited to BUFFER_LENGTH which is 32 on the Uno
		while (bytesLeftToRead >= activeLEDs * 3)
		{
			int toGet = bytesLeftToRead;
			if (toGet > I2C_BUFFER_LENGTH)
			{
				//If toGet is 32 this is bad because we read 6 bytes (Red+IR * 3 = 6) at a time
				//32 % 6 = 2 left over. We don't want to request 32 bytes, we want to request 30.
				//32 % 9 (Red+IR+GREEN) = 5 left over. We want to request 27.

				toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (activeLEDs * 3)); //Trim toGet to be a multiple of the samples we need to read
			}

			bytesLeftToRead -= toGet;

			if (readDataBlock(MAX30102_FIFODATA, data_buffer, toGet) != toGet) {
				NRF_LOG_ERROR("Bloc read problem !\r\n");
				return 0;
			} else {
				//NRF_LOG_WARNING("Reading %u bytes\r\n", toGet);
			}

			while (toGet > 0)
			{
				sense.head++; //Advance the head of the storage struct
				sense.head %= STORAGE_SIZE; //Wrap condition

				byte temp[sizeof(uint32_t)]; //Array of 4 bytes that we will convert into long
				uint32_t tempLong;

				//Burst read three bytes - RED
				temp[3] = 0;
				temp[2] = data_buffer[0];
				temp[1] = data_buffer[1];
				temp[0] = data_buffer[2];

				//Convert array to long
				memcpy(&tempLong, temp, sizeof(tempLong));

				tempLong &= 0x3FFFF; //Zero out all but 18 bits

				sense.red[sense.head] = tempLong; //Store this reading into the sense array

				if (activeLEDs > 1)
				{
					//Burst read three more bytes - IR
					temp[3] = 0;
					temp[2] = data_buffer[3];
					temp[1] = data_buffer[4];
					temp[0] = data_buffer[5];

					//Convert array to long
					memcpy(&tempLong, temp, sizeof(tempLong));

					tempLong &= 0x3FFFF; //Zero out all but 18 bits

					sense.IR[sense.head] = tempLong;
				}

				if (activeLEDs > 2)
				{
					//Burst read three more bytes - Green
					temp[3] = 0;
					temp[2] = data_buffer[6];
					temp[1] = data_buffer[7];
					temp[0] = data_buffer[8];

					//Convert array to long
					memcpy(&tempLong, temp, sizeof(tempLong));

					tempLong &= 0x3FFFF; //Zero out all but 18 bits

					sense.green[sense.head] = tempLong;
				}

				toGet -= activeLEDs * 3;
			}

		} //End while (bytesLeftToRead > 0)

	} //End readPtr != writePtr


	return (numberOfSamples); //Let the world know how much new data we found
}

//Check for new data but give up after a certain amount of time
//Returns true if new data was found
//Returns false if new data was not found
bool MAX30102::safeCheck(uint8_t maxTimeToCheck)
{
	uint32_t markTime = millis();

	while(1)
	{
		if(millis() - markTime > maxTimeToCheck) return(false);

		if(check() == true) //We found new data!
			return(true);

		delay(1);
	}
}

//Given a register, read it, mask it, and then set the thing
void MAX30102::bitMask(uint8_t reg, uint8_t mask, uint8_t thing)
{
	// Grab current register context
	uint8_t originalContents = readRegister8(_i2caddr, reg);

	// Zero-out the portions of the register we're interested in
	originalContents = originalContents & mask;

	// Change contents
	writeRegister8(_i2caddr, reg, originalContents | thing);
}


/*******************************************************************************
 * I2C Reads and Writes
 ******************************************************************************/


bool MAX30102::writeRegister8(uint8_t address, uint8_t reg, uint8_t value) {

	return i2c_write_reg_8(MAX30102_ADDR, reg, value);

}


uint8_t MAX30102::readRegister8(uint8_t address, uint8_t reg) {


	uint8_t res = 0xFF;

	if (!i2c_write8(MAX30102_ADDR, reg)) {
		//return 0xFF;
	}

	// repeated start
	if (!i2c_read8(MAX30102_ADDR, &res)) {
		return 0xFF;
	}

	return res;
}


/**
 * @brief Reads a block (array) of bytes from the I2C device and register
 *
 * @param[in] reg the register to read from
 * @param[out] val pointer to the beginning of the data
 * @param[in] len number of bytes to read
 * @return Number of bytes read. -1 on read error.
 */
int MAX30102::readDataBlock(uint8_t reg, uint8_t *val, unsigned int len) {

	if (!i2c_write8(MAX30102_ADDR, reg)) {
		//return 0;
	}

	// repeated start
	if (!i2c_read_n(MAX30102_ADDR, val, len)) {
		return 0;
	}

	return len;
}


