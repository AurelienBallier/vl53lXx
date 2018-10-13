# VL53L0X library for Linux
Version: 0.2.0<br>
Release date: 05.05.2018<br>
Changelog: see git log

## Summary
This is a library for GNU/Linux that helps to interface with ST's [VL53L0X time-of-flight distance sensor](https://www.pololu.com/product/2490).
This library makes it simple to configure the sensor and read range data from it via I&sup2;C.

Additionally it provides support for managing multiple sensors connected to the same bus by managing their hardware standby state via their XSHUT pins, see [multiple sensors section](#multiple-sensors)

It uses `i2cdev` library for I2C access and `/sys/class/gpio/*` files for GPIO.

### Feedback, bugs, feature requests...
Pull requests and issue reports are welcome!

## Table of Contents
<!-- MarkdownTOC depth=0 autolink="true" bracket="round" -->

- [Development status](#development-status)
- [Supported platforms](#supported-platforms)
- [Usage](#usage)
	- [Hardware](#hardware)
		- [Connections](#connections)
			- [Odroid C2](#odroid-c2)
			- [Raspberry Pi](#raspberry-pi)
			- [BeagleBone Green Wireless](#beaglebone-green-wireless)
	- [Software](#software)
		- [Installation](#installation)
		- [Using the library](#using-the-library)
	- [Multiple sensors](#multiple-sensors)
- [Examples](#examples)
	- [Single ranging mode, single sensor](#single-ranging-mode-single-sensor)
	- [Single ranging mode, multiple sensors](#single-ranging-mode-multiple-sensors)
	- [Continuous ranging mode, multiple sensors](#continuous-ranging-mode-multiple-sensors)
- [ST's VL53L0X API and this library](#sts-vl53l0x-api-and-this-library)
- [Library reference](#library-reference)
- [Special thanks](#special-thanks)
- [License](#license)

<!-- /MarkdownTOC -->

---

## Development status
* __WORKING__
* `Wire.h` replaced with `i2cdev` and `/sys/class/gpio` access
* Multiple sensors support added
* Hardware standby (XSHUT) management support added (part of multiple sensors support)
* Single sensor: tested and working (`examples/single`)
* Continuous measurement, multiple sensors: tested and working (`examples/continuousMultipleSensors`)
* Code style consistency improved
* Using std::exception for errors
* Documentation moved to header (why would you want them with source?)
* Other minor improvements

TODO:
* Improve/add missing documentation
* Update/modify/merge-in(?) I2Cdev to accept i2c device path as argument

---

## Supported platforms
The library should work on all platforms supporting Linux I2Cdev access (`/dev/i2c-*` device files) and GPIO access via `/sys/class/gpio/*`.

It was tested on a BeagleBone Green Wireless.

---

## Usage
### Hardware
A [VL53L0X carrier](https://www.pololu.com/product/2490) can be purchased e.g. from Pololu's website.  Before continuing, careful reading of the [product page](https://www.pololu.com/product/2490) as well as the VL53L0X datasheet is recommended.

Important notes on hardware:
* XSHUT is connected via pull-up resistor to VIN, therefore the sensor can be powered from GPIO even if VIN is disconnected
* Even putting the sensor to hardware standby (XSHUT low, VIN connected) will reset its address! (see [Multiple sensors secton](#multiple-sensors) on why that's important)

#### Connections
##### Odroid C2
Example connection:
```
           Odroid C2   VL53L0X board
--------------------   -------------
 3.3V Power (Pin #1) - VIN
   I2CA_SDA (Pin #3) - SDA
   I2CA_SCL (Pin #5) - SCL
GPIOX.BIT21 (Pin #7) - XSHUT
     Ground (Pin #9) - GND
```

Of course, you can also use `I2CB_SDA/SCL` and any other `GPIOX` and `Ground` pins, see [Hardkernel's page for pins' description](http://odroid.com/dokuwiki/doku.php?id=en:c2_hardware#expansion_connectors).

##### Raspberry Pi
```
      Raspberry Pi 3   VL53L0X board
--------------------   -------------
 3.3V Power (Pin #1) - VIN
       SDA1 (Pin #3) - SDA
       SCL1 (Pin #5) - SCL
  GPIO_GCLK (Pin #7) - XSHUT
     Ground (Pin #9) - GND
```

##### BeagleBone Green Wireless
_NOTE: you have to configure BeagleBone to enable I2C and GPIO on the specific pin you want to use via DTBs._

_NOTE2: GPIOs are organised in blocks of 32 and the resulting GPIO number in `/sys/class/gpio/` is (block number * 32 + gpio number), e.g. `GPIO3_19` -> `/sys/class/gpio/gpio115`_

```
               BBGW   VL53L0X board
-------------------   -------------
 3.3V Power (P9_03) - VIN
       SDA2 (P9_20) - SDA
       SCL2 (P9_19) - SCL
   GPIO3_19 (P9_27) - XSHUT
     Ground (P9_01) - GND
```

### Software
#### Installation
Simply add all the `.cpp` and `.hpp` files to your project and modify your build script to include them.

See included `CMakeLists.txt` and [examples section](#examples) for an example on how to do it using CMake.

_NOTE: If you intend to use other I2C bus than `/dev/i2c-1`, modify the `#define I2C_DEV_PATH` in `I2Cdev.hpp` file._

#### Using the library
* Include `"VL53L0X.hpp"`
* Create an object of `VL53L0X` class
* Call `.init()`
* (Optional) set timing budget etc
* For single range reading:
	* Call `.readRangeSingleMillimeters()`
	* Check for possible timeout using `.timeoutOccurred()` (returned range value will be at maximum of 65535)
* For continuous ranging:
	* Call `.startContinuous()`
	* Read rangings using `.readRangeContinuousMillimeters()`
	* Check for timeout (like in single ranging)
	* End with `.stopContinuous()`

See [examples section](#examples) for reference and [Multiple sensors section](#multiple-sensors) for instructions how to use multiple sensors at once.

### Multiple sensors
Multiple sensors can be used easily by connecting them all to the same I&sup2;C bus and connecting their XSHUT pins to free GPIO pins of your board.

Note that even putting the sensor to hardware standby (XSHUT low) will reset its address! Thus, the workflow is as such:
* disable (power off) __all__ sensors
* enable (power on) first sensor
* set its address
* enable second sensor, set its address
* ...and so on

That translates to following steps within your code:
* pass XSHUT GPIO pin number to sensor object constructor (`VL53L0X(pin)`)
* disable all sensors either by calling their `.powerOff()` method
* initialize sensors one-by-one and set different address for each one before initializing the next one

After that, reading range values from sensors is just like with single one.
See `examples/continuousMultipleSensors.cpp` for reference.

## Examples
Build examples with:
```sh
cd build
cmake ..
make
```
and run with
```sh
./examples/single
./examples/singleMinimal
./examples/continuousMultipleSensors
```

### Single ranging mode, single sensor
`examples/singleMinimal.cpp` shows the minimal working example.

`examples/single.cpp` shows how to use a single sensor in single ranging mode in more detail and with proper commentary on what's going on.

### Continuous ranging mode, multiple sensors
`examples/continuousMultipleSensors.cpp` shows how to use continuous ranging mode (with back-to-back measurements) while using multiple sensors at once.
This is the most complex and thorough example and (after minor modifications) it was used in a real-life application on a mobile robot.

Note: in my experiments on Odroid C2 board with Linux-RT kernel, I've managed to run 6 sensors at once with stable frequency of more than 50Hz!

---

## ST's VL53L0X API and this library
Most of the functionality of this library is based on the [VL53L0X API](http://www.st.com/content/st_com/en/products/embedded-software/proximity-sensors-software/stsw-img005.html) provided by ST (STSW-IMG005), and some of the explanatory comments in the code are quoted or paraphrased from the API source code, API user manual (UM2039), and the VL53L0X datasheet. For more explanation about the library code and how it was derived from the API, see the block comments in VL53L0X.hpp and in-code comments in VL53L0X.cpp.

This library is intended to provide a quicker and easier way to get started using the VL53L0X with a GNU/Linux-driven single-board computer (like BeagleBone Black or Raspberry Pi), in contrast to customizing and compiling ST's API for that platform. The library has a more streamlined, object-oriented interface. However, it does not implement some of the more advanced functionality available in the API (for example, calibrating the sensor to work well under a cover glass), and it has less robust error checking. For advanced applications consider using the VL53L0X API directly (or opening PR/issue to add that functionality!).

---

## Library reference
__This section was not yet updated to match Linux version of the library - see header file for updated descriptions!__

* `uint8_t last_status`<br>
  The status of the last I&sup2;C write transmission. 0 for success, errno for errors (you can use strerrno from <cstring> to see error description)
* `VL53L0X()`<br>
  Constructor.
* `void setAddress(uint8_t new_addr)`<br>
  Changes the I&sup2;C slave device address of the VL53L0X to the given value (7-bit).
* `uint8_t getAddress()`<br>
  Returns the current I&sup2;C address.
* `bool init(bool io_2v8 = true)`<br>
  Iniitializes and configures the sensor. If the optional argument `io_2v8` is true (the default if not specified), the sensor is configured for 2V8 mode (2.8 V I/O); if false, the sensor is left in 1V8 mode. The return value is a boolean indicating whether the initialization completed successfully.
* `void writeReg(uint8_t reg, uint8_t value)`<br>
  Writes an 8-bit sensor register with the given value.
  Register address constants are defined by the REGISTER_ADDRESSES enumeration type in VL53L0X.h.<br>
  Example use: `sensor.writeReg(VL53L0X::SYSRANGE_START, 0x01);`
* `void writeReg16Bit(uint8_t reg, uint16_t value)`<br>
  Writes a 16-bit sensor register with the given value.
* `void writeReg32Bit(uint8_t reg, uint32_t value)`<br>
  Writes a 32-bit sensor register with the given value.
* `uint8_t readReg(uint8_t reg)`<br>
  Reads an 8-bit sensor register and returns the value read.
* `uint16_t readReg16Bit(uint8_t reg)`<br>
  Reads a 16-bit sensor register and returns the value read.
* `uint32_t readReg32Bit(uint8_t reg)`<br>
  Reads a 32-bit sensor register and returns the value read.
* `void writeMulti(uint8_t reg, uint8_t const * src, uint8_t count)`<br>
  Writes an arbitrary number of bytes from the given array to the sensor, starting at the given register.
* `void readMulti(uint8_t reg, uint8_t * dst, uint8_t count)`<br>
  Reads an arbitrary number of bytes from the sensor, starting at the given register, into the given array.
* `bool setSignalRateLimit(float limit_Mcps)`<br>
  Sets the return signal rate limit to the given value in units of MCPS (mega counts per second). This is the minimum amplitude of the signal reflected from the target and received by the sensor necessary for it to report a valid reading. Setting a lower limit increases the potential range of the sensor but also increases the likelihood of getting an inaccurate reading because of reflections from objects other than the intended target. This limit is initialized to 0.25 MCPS by default. The return value is a boolean indicating whether the requested limit was valid.
* `float getSignalRateLimit()`<br>
  Returns the current return signal rate limit in MCPS.
* `bool setMeasurementTimingBudget(uint32_t budget_us)`<br>
  Sets the measurement timing budget to the given value in microseconds. This is the time allowed for one range measurement; a longer timing budget allows for more accurate measurements. The default budget is about 33000 microseconds, or 33 ms; the minimum is 20 ms. The return value is a boolean indicating whether the requested budget was valid.
* `uint32_t getMeasurementTimingBudget()`<br>
  Returns the current measurement timing budget in microseconds.
* `bool setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks)`
  Sets the VCSEL (vertical cavity surface emitting laser) pulse period for the given period type (`VL53L0X::VcselPeriodPreRange` or `VL53L0X::VcselPeriodFinalRange`) to the given value (in PCLKs). Longer periods increase the potential range of the sensor. Valid values are (even numbers only):

  Pre: 12 to 18 (initialized to 14 by default)<br>
  Final: 8 to 14 (initialized to 10 by default)

  The return value is a boolean indicating whether the requested period was valid.
* `uint8_t getVcselPulsePeriod(vcselPeriodType type)`<br>
  Returns the current VCSEL pulse period for the given period type.
* `void startContinuous(uint32_t period_ms = 0)`<br>
  Starts continuous ranging measurements. If the optional argument `period_ms` is 0 (the default if not specified), continuous back-to-back mode is used (the sensor takes measurements as often as possible); if it is nonzero, continuous timed mode is used, with the specified inter-measurement period in milliseconds determining how often the sensor takes a measurement.
* `void stopContinuous()`<br>
  Stops continuous mode.
* `uint16_t readRangeContinuousMillimeters()`<br>
  Returns a range reading in millimeters when continuous mode is active.
* `uint16_t readRangeSingleMillimeters()`<br>
  Performs a single-shot ranging measurement and returns the reading in millimeters.
* `void setTimeout(uint16_t timeout)`<br>
  Sets a timeout period in milliseconds after which read operations will abort if the sensor is not ready. A value of 0 disables the timeout.
* `uint16_t getTimeout()`<br>
  Returns the current timeout period setting.
* `bool timeoutOccurred()`<br>
  Indicates whether a read timeout has occurred since the last call to `timeoutOccurred()`.

---

## Special thanks
* Pololu for both the sensor breakout board and their [Arduino library](https://github.com/pololu/vl53l0x-arduino) this project bases upon
* Drogon for his awesome [WiringPi library](http://wiringpi.com/), see notes in [my fork](https://github.com/mjbogusz/wiringPi)
* ST for making this great sensor!

## License
This library is licensed under MIT license, see `LICENSE.txt` file for full license text.
