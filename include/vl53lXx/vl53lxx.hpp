#ifndef _VL53LXX_H
#define _VL53LXX_H

#include <unistd.h>
#include <fstream>
#include <mutex>

#include <vl53lXx/interfaces/i2cgeneric.hpp>

#ifdef BUILD_I2C_DEV
#include <vl53lXx/interfaces/i2cdev.hpp>
#endif

#define CALIB_SIZE 2
static float DEFAULT_CALIB[CALIB_SIZE] = {0.0f, 1.0f};

class VL53LXX
{
  public:
    VL53LXX(uint8_t port, const uint8_t address, const int16_t xshutGPIOPin = -1, bool ioMode2v8 = true, float *calib = DEFAULT_CALIB);

    ~VL53LXX();

    virtual bool init() = 0;

    virtual void setAddress(uint8_t new_addr) = 0;
    uint8_t getAddress() { return this->i2c->getAddress(); }

    virtual uint16_t readRangeSingleMillimeters(bool blocking = true) = 0;
    virtual uint16_t readRangeContinuousMillimeters(bool blocking = true) = 0;

    virtual void startContinuous(uint32_t period_ms = 0) = 0;
    virtual void stopContinuous() = 0;

		/**
		 * Power on the sensor by setting its XSHUT pin to high via host's GPIO.
		 */
		void powerOn();

		/**
		 * Power off the sensor by setting its XSHUT pin to low via host's GPIO.
		 */
		void powerOff();

  protected:
		I2Cgeneric* i2c;
    float calib[CALIB_SIZE];
    int16_t xshutGPIOPin;
    bool ioMode2v8;

    //GPIO
		std::string gpioFilename;
		std::mutex fileAccessMutex;
		bool gpioInitialized = false;

		void initGPIO();
};

#endif
