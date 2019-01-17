#include <vl53lXx/vl53lxx.hpp>

VL53LXX::VL53LXX(uint8_t port, const uint8_t address, const int16_t xshutGPIOPin, bool ioMode2v8, float *calib):
    xshutGPIOPin(xshutGPIOPin),
    ioMode2v8(ioMode2v8)
{
    #ifdef BUILD_I2C_DEV
    this->i2c = new I2Cdev(port, address);
    #endif

    for(unsigned int i = 0; i < CALIB_SIZE; i++){
    this->calib[i] = calib[i];
    }

    if(xshutGPIOPin != -1){
        this->powerOn();
    }
}

VL53LXX::~VL53LXX(){
    if(xshutGPIOPin != -1){
        this->powerOff();
    }

    delete this->i2c;
}

void VL53LXX::initGPIO() {
	if (this->gpioInitialized) {
		return;
	}

	// Set XSHUT pin mode (if pin set)
	if (this->xshutGPIOPin >= 0) {
		std::string gpioDirectionFilename = std::string("/sys/class/gpio/gpio") + std::to_string(this->xshutGPIOPin) + std::string("/direction");
		this->gpioFilename = std::string("/sys/class/gpio/gpio") + std::to_string(this->xshutGPIOPin) + std::string("/value");

		std::lock_guard<std::mutex> guard(this->fileAccessMutex);

		std::ofstream file;
		file.open("/sys/class/gpio/export", std::ofstream::out);
		if (!file.is_open() || !file.good()) {
			file.close();
			throw(std::runtime_error("Failed opening file: /sys/class/gpio/export"));
		}
		file << this->xshutGPIOPin;
		file.close();

		file.open(gpioDirectionFilename.c_str(), std::ofstream::out);
		if (!file.is_open() || !file.good()) {
			file.close();
			throw(std::runtime_error(std::string("Failed opening file: ") + gpioDirectionFilename));
		}
		file << "out";
		file.close();
	}

	this->gpioInitialized = true;
}

void VL53LXX::powerOn() {
	this->initGPIO();

	if (this->xshutGPIOPin >= 0) {
		std::lock_guard<std::mutex> guard(this->fileAccessMutex);
		std::ofstream file;

		file.open(this->gpioFilename.c_str(), std::ofstream::out);
		if (!file.is_open() || !file.good()) {
			file.close();
			throw(std::runtime_error(std::string("Failed opening file: ") + this->gpioFilename));
		}
		file << "1";
		file.close();

		// t_boot is 1.2ms max, wait 2ms just to be sure
		usleep(2000);
	}
}

void VL53LXX::powerOff() {
	this->initGPIO();

	if (this->xshutGPIOPin >= 0) {
		std::lock_guard<std::mutex> guard(this->fileAccessMutex);
		std::ofstream file;

		file.open(this->gpioFilename.c_str(), std::ofstream::out);
		if (!file.is_open() || !file.good()) {
			file.close();
			throw(std::runtime_error(std::string("Failed opening file: ") + this->gpioFilename));
		}
		file << "0";
		file.close();
	}
}