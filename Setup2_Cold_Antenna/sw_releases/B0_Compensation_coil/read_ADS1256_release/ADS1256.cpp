/*
        ADS1256.h - Arduino Library for communication with Texas Instrument ADS1256 ADC
        Written by Adien Akhmad, August 2015
		Modfified  Jan 2019 by Axel Sepulveda for ATMEGA328
    Modified Oct 2023 by Hippolyte Monroe for SAM3X8E (Arduino Due MCU)
*/

#include "ADS1256.h"

ADS1256::ADS1256(float clockspdMhz, float vref, bool useResetPin) {
  // Set DRDY as input
  pinMode(pinDRDY, INPUT);      
  // Set CS as output
  pinMode(pinCS, OUTPUT);
  
  if (useResetPin) {
    // set RESETPIN as output
    pinMode(pinRST, OUTPUT );
    // pull RESETPIN high
    pinMode(pinRST, HIGH);
  }
  
  // Voltage Reference
  _VREF = vref;

  // Default conversion factor
  _conversionFactor = 1.0;
  
  // Start SPI on a quarter of ADC clock speed
  SPI.begin();
  SPI.beginTransaction(
      SPISettings(clockspdMhz * 1000000 / 4, MSBFIRST, SPI_MODE1));     // min (fSCLK) = fCLKIN/4 according to ADS1256 datasheet (p.6)
}

void ADS1256::writeRegister(unsigned char reg, unsigned char wdata) {
  CSON();
  // waitDRDY();   //added
  // this->sendCommand(ADS1256_CMD_SDATAC);//added
  SPI.transfer(ADS1256_CMD_WREG | reg); // opcode1 Write registers starting from reg
  SPI.transfer(0);  // opcode2 Write 1+0 registers
  SPI.transfer(wdata);  // write wdata
  delayMicroseconds(1);              
  CSOFF();
}

unsigned char ADS1256::readRegister(unsigned char reg) {
  unsigned char readValue;
  CSON();
  // waitDRDY();//added
  // this->sendCommand(ADS1256_CMD_SDATAC);//added
  SPI.transfer(ADS1256_CMD_RREG | reg); // opcode1 read registers starting from reg
  SPI.transfer(0);                  // opcode2 read 1+0 registers
  delayMicroseconds(7);              //  t6 delay (4*tCLKIN 50*0.13 = 6.5 us)    
  readValue = SPI.transfer(0);          // read registers
  delayMicroseconds(1);              //  t11 delay (4*tCLKIN 4*0.13 = 0.52 us)    
  CSOFF();
  return readValue;
}

void ADS1256::sendCommand(unsigned char reg) {
  CSON();
  waitDRDY();
  SPI.transfer(reg);
  delayMicroseconds(1);              //  t11 delay (4*tCLKIN = 4*0.13 = 0.52 us)    
  CSOFF();
}

void ADS1256::setConversionFactor(float val) { _conversionFactor = val; }

void ADS1256::readTest() {
  unsigned char _highByte, _midByte, _lowByte;
  CSON();
  SPI.transfer(ADS1256_CMD_RDATA);
  delayMicroseconds(7);              //  t6 delay (50*tCLKIN = 50*0.13 = 6.5 us)    

  _highByte = SPI.transfer(ADS1256_CMD_WAKEUP);
  _midByte = SPI.transfer(ADS1256_CMD_WAKEUP);
  _lowByte = SPI.transfer(ADS1256_CMD_WAKEUP);

  CSOFF();
}

float ADS1256::readCurrentChannel() {
  CSON();
  SPI.transfer(ADS1256_CMD_RDATA);
  delayMicroseconds(7);              //  t6 delay (50*tCLKIN = 50*0.13 = 6.5 us)              
  float adsCode = read_float32();
  CSOFF();
  return ((adsCode / 0x7FFFFF) * ((2 * _VREF) / (float)_pga)) *
         _conversionFactor;
}

// Reads raw ADC data, as 32bit int
long ADS1256::readCurrentChannelRaw() {
  CSON();
  SPI.transfer(ADS1256_CMD_RDATA);
  delayMicroseconds(7);              //  t6 delay (50*tCLKIN = 50*0.13 = 6.5 us)       
  long adsCode = read_int32();
  CSOFF();
  return adsCode;
}

float ADS1256::readCurrentChannelC() {
  float adsCode = read_float32();
  return ((adsCode / 0x7FFFFF) * ((2 * _VREF) / (float)_pga)) *
         _conversionFactor;
}

// void ADS1256::readCurrentChannelRawC(byte* output_buffer) {
//   output_buffer[0] = SPI.transfer(0);
//   output_buffer[1] = SPI.transfer(0);
//   output_buffer[2] = SPI.transfer(0);
// }

long ADS1256::readCurrentChannelRawC() {
  long adsCode = read_int32();
  return adsCode;
}

// Call this ONLY after ADS1256_CMD_RDATA command
unsigned long ADS1256::read_uint24() {
  unsigned char _highByte, _midByte, _lowByte;
  unsigned long value;

  _highByte = SPI.transfer(0);
  _midByte  = SPI.transfer(0);
  _lowByte  = SPI.transfer(0);

  // Combine all 3-bytes to 24-bit data using byte shifting.
  value = ((long)_highByte << 16) + ((long)_midByte << 8) + ((long)_lowByte);
  return value;
}

// Call this ONLY after ADS1256_CMD_RDATA command
// Convert the signed 24bit stored in an unsigned 32bit to a signed 32bit
long ADS1256::read_int32() {
  long value = read_uint24();

  if (value & 0x00800000) { // if the 24 bit value is negative reflect it to 32bit
    value |= 0xff000000;
  }

  return value;
}

// Call this ONLY after ADS1256_CMD_RDATA command
// Cast as a float
float ADS1256::read_float32() {
  long value = read_int32();
  return (float)value;
}

// Channel switching  
// Negative input channel are set to AINCOM by default (single ended mode)
void ADS1256::setChannel(unsigned char AIN_P, unsigned char AIN_N) {
  byte MUX_CHANNEL;

  MUX_CHANNEL = (AIN_P << 4) | AIN_N;

  CSON();
  sendCommand(ADS1256_CMD_SDATAC);
  writeRegister(ADS1256_RADD_MUX, MUX_CHANNEL);
  sendCommand(ADS1256_CMD_SYNC);
  sendCommand(ADS1256_CMD_WAKEUP);
  CSOFF();
}
/*
Init chip with set datarate and gain and perform self calibration
*/ 
void ADS1256::begin(unsigned char drate, unsigned char gain, bool buffenable, bool clkout_disable) {
  _pga = 1 << gain;
  waitDRDY();
  sendCommand (ADS1256_CMD_RESET);
  delay(1000);
  // sendCommand(ADS1256_CMD_SDATAC);  // send out ADS1256_CMD_SDATAC command to stop continous reading mode.
  writeRegister(ADS1256_RADD_DRATE, drate);  // write data rate register   
  uint8_t bytemask = clkout_disable ? B01100111 : B00000111;    //bits 0-2 = gain ; bits 5-6 = clkout rate (off if 00)
  uint8_t adcon = readRegister(ADS1256_RADD_ADCON);
  uint8_t byte2send = (adcon & ~bytemask) | gain;
  writeRegister(ADS1256_RADD_ADCON, byte2send);

  uint8_t status = readRegister(ADS1256_RADD_STATUS); 
  bitSet(status, 2);    //enable ACAL (auto calibration) (change)
  if (buffenable) {  
    bitSet(status, 1);
  }
  writeRegister(ADS1256_RADD_STATUS, status);

  sendCommand(ADS1256_CMD_SELFCAL);  // perform self calibration
  
  waitDRDY();
  // wait ADS1256 to settle after self calibration
}

/*
Init chip with default datarate and gain and perform self calibration
*/ 
void ADS1256::begin() {
  sendCommand(ADS1256_CMD_SDATAC);  // send out ADS1256_CMD_SDATAC command to stop continous reading mode.
  uint8_t status = readRegister(ADS1256_RADD_STATUS);      
  sendCommand(ADS1256_CMD_SELFCAL);  // perform self calibration  
  waitDRDY();   // wait ADS1256 to settle after self calibration
}

/*
Reads and returns STATUS register
*/ 
uint8_t ADS1256::getStatus() {
  sendCommand(ADS1256_CMD_SDATAC);  // send out ADS1256_CMD_SDATAC command to stop continous reading mode.
  return readRegister(ADS1256_RADD_STATUS); 
}



void ADS1256::CSON() {
  //PORT_CS &= ~(1 << PINDEX_CS);
  digitalWrite(pinCS, LOW);
}  // digitalWrite(_CS, LOW); }

void ADS1256::CSOFF() {
  digitalWrite(pinCS, HIGH);
  //PORT_CS |= (1 << PINDEX_CS);
}  // digitalWrite(_CS, HIGH); }

void ADS1256::waitDRDY() {
  //while (PIN_DRDY & (1 << PINDEX_DRDY));
  while (digitalRead(pinDRDY));
}

boolean ADS1256::isDRDY() {
  return !digitalRead(pinDRDY);
}	

void ADS1256::startRdatac (){
  SPI.beginTransaction(SPISettings(1920000, MSBFIRST, SPI_MODE1)); 
  CSON();
  waitDRDY();
  SPI.transfer (ADS1256_CMD_RDATAC);
  delayMicroseconds (7);
}
