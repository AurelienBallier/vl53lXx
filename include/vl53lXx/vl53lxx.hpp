#ifndef _VL53LXX_H
#define _VL53LXX_H

#include <unistd.h>

#include <vl53lXx/interfaces/i2cgeneric.hpp>

#ifdef BUILD_I2C_DEV
#include <vl53lXx/interfaces/i2cdev.hpp>
#endif


class VL53LXX
{
  public:
    VL53LXX(uint8_t port, const uint8_t address){
      #ifdef BUILD_I2C_DEV
      this->i2c = new I2Cdev(port, address);
      #endif
    };
    ~VL53LXX(){
      delete this->i2c;
    }

    virtual bool init(bool unused = true) = 0;

    virtual void setAddress(uint8_t new_addr) = 0;
    uint8_t getAddress() { return this->i2c->getAddress(); }

    virtual uint16_t readRangeSingleMillimeters(bool blocking = true) = 0;
    virtual uint16_t readRangeContinuousMillimeters(bool blocking = true) = 0;

    virtual void startContinuous(uint32_t period_ms) = 0;
    virtual void stopContinuous() = 0;

  protected:
		I2Cgeneric* i2c;
};

#endif
