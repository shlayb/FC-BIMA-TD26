#ifndef I2C_H
#define I2C_H
//  include guards
#ifndef Arduino_h
    #include <Arduino.h>
#endif
#ifndef TwoWire_h
    #include <Wire.h>
#endif
// #ifndef
// #define
// #endif

void i2c_Init(uint8_t _i2c_clock=400000)   // initialize wire communication
{
    Wire.begin();
    Wire.setClock(_i2c_clock);
}

void SendByte(uint8_t address, uint8_t data)
{
    Wire.beginTransmission(address);
    Wire.write(data);
    Wire.endTransmission();
}

void  WriteByte(uint8_t address, uint8_t subAddress, uint8_t data)  // write to slave
{
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.write(data);
    Wire.endTransmission();
}

uint8_t  ReadByte(uint8_t address, uint8_t subAddress)  // read a byte from slave
{
    uint8_t data;
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.endTransmission(false);
    Wire.requestFrom(address, (uint8_t) 1);
    data = Wire.read();
    return data;
}

uint16_t  ReadBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest) // read bytes of data from slave
{
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.endTransmission(false);
    uint8_t i = 0;
    Wire.requestFrom(address, count);
    while(Wire.available()){
        dest[i++] = Wire.read();
    }
    return *dest;
}

uint8_t  ReadBytes16(uint8_t address, uint8_t subAddress)  // read 24 bit from slave
{
    uint8_t data;
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.endTransmission();
    Wire.beginTransmission(address);
    Wire.requestFrom(address, 2);
    while (!Wire.available()){};
    uint8_t a = Wire.read();    // MSB Meast Significant Bit
    uint8_t b = Wire.read();    // LSB (Least Significant Bit)
    Wire.endTransmission();
    data = a << 8 | b;
    return data;
}

uint8_t  ReadBytes24(uint8_t address, uint8_t subAddress)  // read 24 bit from slave
{
    uint8_t data;
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.endTransmission();
    Wire.beginTransmission(address);
    Wire.requestFrom(address, 3);
    while (!Wire.available()){};
    uint8_t a = Wire.read();    // XSB ?
    uint8_t b = Wire.read();    // MSB Meast Significant Bit
    uint8_t c = Wire.read();    // LSB (Least Significant Bit)
    Wire.endTransmission();
    data = ((int32_t)a << 16) | ((int32_t)b << 8 | c);
    return data;
}
#endif