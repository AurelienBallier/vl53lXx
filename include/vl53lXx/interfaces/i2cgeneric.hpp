// I2Cdev library collection - Main I2C device class header file
// Abstracts bit and byte I2C R/W functions into a convenient class
// 6/9/2012 by Jeff Rowberg <jeff@rowberg.net>
//
// Changelog:
//     2012-06-09 - fix major issue with reading > 32 bytes at a time with Arduino Wire
//                - add compiler warnings when using outdated or IDE or limited I2Cdev implementation
//     2011-11-01 - fix write*Bits mask calculation (thanks sasquatch @ Arduino forums)
//     2011-10-03 - added automatic Arduino version detection for ease of use
//     2011-10-02 - added Gene Knight's NBWire TwoWire class implementation with small modifications
//     2011-08-31 - added support for Arduino 1.0 Wire library (methods are different from 0.x)
//     2011-08-03 - added optional timeout parameter to read* methods to easily change from default
//     2011-08-02 - added support for 16-bit registers
//                - fixed incorrect Doxygen comments on some methods
//                - added timeout value for read operations (thanks mem @ Arduino forums)
//     2011-07-30 - changed read/write function structures to return success or byte counts
//                - made all methods for multi-device memory savings
//     2011-07-28 - initial release

/* ============================================
I2Cgeneric device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

#ifndef _I2CGENERIC_H_
#define _I2CGENERIC_H_

#include <cstdint>
#include <stdio.h>

class I2Cgeneric {
    public:
        I2Cgeneric(uint8_t port, uint8_t address){};

        virtual void setAddress(uint8_t address) = 0;
        virtual uint8_t getAddress() = 0;

        //8-bits addresses
        virtual int8_t readBit(uint8_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout=0) = 0;
        virtual int8_t readBitW(uint8_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout=0) = 0;
        virtual int8_t readBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout=0) = 0;
        virtual int8_t readBitsW(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout=0) = 0;
        virtual int8_t readByte(uint8_t regAddr, uint8_t *data, uint16_t timeout=0);
        virtual int8_t readWord(uint8_t regAddr, uint16_t *data, uint16_t timeout=0) = 0;
        virtual int8_t readBytes(uint8_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout=0) = 0;
        virtual int8_t readWords(uint8_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout=0) = 0;

        //16-bits addresses
        virtual int8_t readBit(uint16_t regAddr, uint8_t bitNum, uint8_t *data, uint16_t timeout=0) = 0;
        virtual int8_t readBitW(uint16_t regAddr, uint8_t bitNum, uint16_t *data, uint16_t timeout=0) = 0;
        virtual int8_t readBits(uint16_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data, uint16_t timeout=0) = 0;
        virtual int8_t readBitsW(uint16_t regAddr, uint8_t bitStart, uint8_t length, uint16_t *data, uint16_t timeout=0) = 0;
        virtual int8_t readByte(uint16_t regAddr, uint8_t *data, uint16_t timeout=0) = 0;
        virtual int8_t readWord(uint16_t regAddr, uint16_t *data, uint16_t timeout=0) = 0;
        virtual int8_t readBytes(uint16_t regAddr, uint8_t length, uint8_t *data, uint16_t timeout=0) = 0;
        virtual int8_t readWords(uint16_t regAddr, uint8_t length, uint16_t *data, uint16_t timeout=0) = 0;

        //8-bits addresses
        virtual bool writeBit(uint8_t regAddr, uint8_t bitNum, uint8_t data) = 0;
        virtual bool writeBitW(uint8_t regAddr, uint8_t bitNum, uint16_t data) = 0;
        virtual bool writeBits(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) = 0;
        virtual bool writeBitsW(uint8_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data) = 0;
        virtual bool writeByte(uint8_t regAddr, uint8_t data){ printf("virtual 8bit\n"); return false; }
        virtual bool writeWord(uint8_t regAddr, uint16_t data) = 0;
        virtual bool writeBytes(uint8_t regAddr, uint8_t length, uint8_t *data) = 0;
        virtual bool writeWords(uint8_t regAddr, uint8_t length, uint16_t *data) = 0;

        //16-bits addresses
        virtual bool writeBit(uint16_t regAddr, uint8_t bitNum, uint8_t data) = 0;
        virtual bool writeBitW(uint16_t regAddr, uint8_t bitNum, uint16_t data) = 0;
        virtual bool writeBits(uint16_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data) = 0;
        virtual bool writeBitsW(uint16_t regAddr, uint8_t bitStart, uint8_t length, uint16_t data) = 0;
        virtual bool writeByte(uint16_t regAddr, uint8_t data){ printf("virtual 16bit\n"); return false; }
        virtual bool writeWord(uint16_t regAddr, uint16_t data) = 0;
        virtual bool writeBytes(uint16_t regAddr, uint8_t length, uint8_t *data) = 0;
        virtual bool writeWords(uint16_t regAddr, uint8_t length, uint16_t *data) = 0;

        uint16_t readTimeout;
};

#endif //_I2CGENERIC_H_
