
#include "MagneticSensorSPI.h"

//CRC table for poly = x^4 + x^3 + x^2 + 1 (0x1d or 0b11101 or 29)
const uint8_t crc4LookupTable[16] = {0, 13, 7, 10, 14, 3, 9, 4, 1, 12, 6, 11, 15, 2, 8, 5};

/** Typical configuration for the 14bit AMS AS5147 magnetic sensor over SPI interface */
MagneticSensorSPIConfig_s AS5147_SPI = {
  .spi_mode = SPI_MODE1,
  .clock_speed = 1000000,
  .bit_resolution = 14,
  .angle_register = 0x3FFF,
  .data_start_bit = 13,
  .command_rw_bit = 14,
  .command_parity_bit = 15,
  .crc_type = NO_CRC
};
// AS5048 and AS5047 are the same as AS5147
MagneticSensorSPIConfig_s AS5048_SPI = AS5147_SPI;
MagneticSensorSPIConfig_s AS5047_SPI = AS5147_SPI;

/** Typical configuration for the 14bit MonolithicPower MA730 magnetic sensor over SPI interface */
MagneticSensorSPIConfig_s MA730_SPI = {
  .spi_mode = SPI_MODE0,
  .clock_speed = 1000000,
  .bit_resolution = 14,
  .angle_register = 0x0000,
  .data_start_bit = 15,
  .command_rw_bit = 0,  // not required
  .command_parity_bit = 0, // parity not implemented
  .crc_type = NO_CRC
};

/** Typical configuration for the 14bit MonolithicPower MA600 magnetic sensor over SPI interface */
MagneticSensorSPIConfig_s MA600_SPI = {
  .spi_mode = SPI_MODE0,
  .clock_speed = 1000000,
  .bit_resolution = 14,
  .angle_register = 0x0000,
  .data_start_bit = 15,
  .command_rw_bit = 0,  // not required
  .command_parity_bit = 0, // parity not implemented
  .crc_type = NO_CRC
};

/** Typical configuration for the 16bit MonolithicPower MA900 magnetic sensor over SPI interface */
MagneticSensorSPIConfig_s MA900_SPI = {
  .spi_mode = SPI_MODE0,
  .clock_speed = 1000000,
  .bit_resolution = 16,
  .angle_register = 0x036B,
  .data_start_bit = 15,  // Included in angle_register
  .command_rw_bit = 0,  // Included in angle_register
  .command_parity_bit = 0, // parity not implemented
  .crc_type = MA900_CRC // CRC needed for this sensor
};

// MagneticSensorSPI(int cs, float _bit_resolution, int _angle_register)
//  cs              - SPI chip select pin
//  _bit_resolution   sensor resolution bit number
// _angle_register  - (optional) angle read register - default 0x3FFF
MagneticSensorSPI::MagneticSensorSPI(int cs, float _bit_resolution, int _angle_register){

  chip_select_pin = cs;
  // angle read register of the magnetic sensor
  angle_register = _angle_register ? _angle_register : DEF_ANGLE_REGISTER;
  // register maximum value (counts per revolution)
  cpr = pow(2,_bit_resolution);
  spi_mode = SPI_MODE1;
  clock_speed = 1000000;
  bit_resolution = _bit_resolution;

  command_parity_bit = 15; // for backwards compatibilty
  command_rw_bit = 14; // for backwards compatibilty
  data_start_bit = 13; // for backwards compatibilty
}

MagneticSensorSPI::MagneticSensorSPI(MagneticSensorSPIConfig_s config, int cs){
  chip_select_pin = cs;
  // angle read register of the magnetic sensor
  angle_register = config.angle_register ? config.angle_register : DEF_ANGLE_REGISTER;
  // register maximum value (counts per revolution)
  cpr = pow(2, config.bit_resolution);
  spi_mode = config.spi_mode;
  clock_speed = config.clock_speed;
  bit_resolution = config.bit_resolution;

  command_parity_bit = config.command_parity_bit; // for backwards compatibilty
  command_rw_bit = config.command_rw_bit; // for backwards compatibilty
  data_start_bit = config.data_start_bit; // for backwards compatibilty
  crc_type = config.crc_type;
}

void MagneticSensorSPI::init(SPIClass* _spi){
  spi = _spi;
	// 1MHz clock (AMS should be able to accept up to 10MHz)
	settings = SPISettings(clock_speed, MSBFIRST, spi_mode);
	//setup pins
	pinMode(chip_select_pin, OUTPUT);
	//SPI has an internal SPI-device counter, it is possible to call "begin()" from different devices
	spi->begin();
	// do any architectures need to set the clock divider for SPI? Why was this in the code?
  //spi->setClockDivider(SPI_CLOCK_DIV8);
	digitalWrite(chip_select_pin, HIGH);

  this->Sensor::init(); // call base class init
}

//  Shaft angle calculation
//  angle is in radians [rad]
float MagneticSensorSPI::getSensorAngle(){
  return (getRawCount() / (float)cpr) * _2PI;
}

// function reading the raw counter of the magnetic sensor
int MagneticSensorSPI::getRawCount(){
	return (int)MagneticSensorSPI::read(angle_register);
}

// SPI functions 
/**
 * Utility function used to calculate even parity of word
 */
byte MagneticSensorSPI::spiCalcEvenParity(word value){
	byte cnt = 0;
	byte i;

	for (i = 0; i < 16; i++)
	{
		if (value & 0x1) cnt++;
		value >>= 1;
	}
	return cnt & 0x1;
}

/**
 * Utility function used to add a computed crc at the end of a 12bit data
 * Takes the 12bit data as uint16_t
 * Returns the 16bit data  with a 4bit crc at the end (12bit data are shifted)
 */
uint16_t MagneticSensorSPI::appendCrc4(uint16_t data){
  uint8_t crc4;                                   // 4-bit CRC (0-15)
  uint16_t valueWithCrc;
  // Compute CRC4

  switch (this->crc_type)
  {
  case MA900_CRC:
    crc4 = 10;                                      // Initial Value for CRC4 (Seed)
    crc4 = (data>>8 & 0xF) ^ crc4LookupTable[crc4]; // 4 MSB first
    crc4 = (data>>4 & 0xF) ^ crc4LookupTable[crc4]; // 
    crc4 = (data>>0 & 0xF) ^ crc4LookupTable[crc4]; // 4 LSB
    // Concatenate 12-bit data with CRC4
    valueWithCrc = ((data<<4) | crc4);
    break;
  
  default:
    // No crc or unknown crc: not changing data
    valueWithCrc = data;
    break;
  }
  return valueWithCrc;   
}

  /*
  * Read a register from the sensor
  * Takes the address of the register as a 16 bit word
  * Returns the value of the register
  */
word MagneticSensorSPI::read(word angle_register){

  word command = angle_register;

  if (command_rw_bit > 0) {
    command = angle_register | (1 << command_rw_bit);
  }
  if (command_parity_bit > 0) {
   	//Add a parity bit on the the MSB
  	command |= ((word)spiCalcEvenParity(command) << command_parity_bit);
  }
  if (crc_type != NO_CRC){
    command = (word)appendCrc4(command);
  }

  //SPI - begin transaction
  spi->beginTransaction(settings);
  digitalWrite(chip_select_pin, LOW);

  //For the MA900 sensor, the angle is already sent at the start of the communication
  word register_value = spi->transfer16(command);
  
#if defined(ESP_H) && defined(ARDUINO_ARCH_ESP32) // if ESP32 board
  delayMicroseconds(50); // why do we need to delay 50us on ESP32? In my experience no extra delays are needed, on any of the architectures I've tested...
#else
  delayMicroseconds(1); // delay 1us, the minimum time possible in plain arduino. 350ns is the required time for AMS sensors, 80ns for MA730, MA702
#endif
  
  if (crc_type == MA900_CRC){
    //Read the remaining 4bit value by sending bach the data recieved from first communication
    word register_value2 = spi->transfer16(register_value);
    //Gather data in one place
    word angle_raw = (register_value&0xFFF0) + ((register_value2&0xF000)>>12);
    register_value = angle_raw;
  } else {
    // Other sensors -> Read the response
    register_value = spi->transfer16(0x00);
  }
  digitalWrite(chip_select_pin, HIGH);

  //SPI - end transaction
  spi->endTransaction();

  register_value = register_value >> (1 + data_start_bit - bit_resolution);  //this should shift data to the rightmost bits of the word

  const static word data_mask = 0xFFFF >> (16 - bit_resolution);

	return register_value & data_mask;  // Return the data, stripping the non data (e.g parity) bits
}

/**
 * Closes the SPI connection
 * SPI has an internal SPI-device counter, for each init()-call the close() function must be called exactly 1 time
 */
void MagneticSensorSPI::close(){
	spi->end();
}


