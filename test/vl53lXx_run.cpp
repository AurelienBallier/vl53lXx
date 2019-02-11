#include <stdio.h>
#include <iostream>
#include <unistd.h>

#include <boost/program_options.hpp>

#include <vl53lXx/vl53lxx.hpp>
#include <vl53lXx/vl53l0x.hpp>
#include <vl53lXx/vl53l1x.hpp>

namespace po = boost::program_options;

int main(int argc, char** argv){
    std::cout << "vl53lXx test program." << std::endl;

    po::options_description desc("Allowed options");
    desc.add_options()
        ("help", "vl53lXx lib testing program")
        ("model", po::value<unsigned int>()->default_value(0), "Choose 0 for vl53l0x or 1 for vl53l1x.")
        ("bus", po::value<unsigned int>()->default_value(1), "Choose I2C bus to use.")
        ("address", po::value<unsigned int>()->default_value(0), "I2C addrss of the slave device.")
        ("single", po::bool_switch()->default_value(false), "Start a single distance measurement instead of continuous.")
        ("period", po::value<unsigned int>()->default_value(1000), "Refresh period in milliseconds.")
        ("timing-budget", po::value<unsigned int>()->default_value(50000), "Timing budget for range measurement.")
        ("range-mode", po::value<unsigned int>()->default_value(0), "Range mode (ie. Short (0) / Medium (1) /Long (2)).")
        ("gpio", po::value<int>()->default_value(-1), "GPIO number for chip select, -1 for disactivated.")
        ("mean", po::value<unsigned int>()->default_value(1), "Number of value to average.")
        ("power", po::value<bool>()->default_value(true), "Power state of the device.")
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    if (vm.count("help")){
        std::cout << desc << '\n';
        return 0;
    }

    unsigned int bus = vm["bus"].as<unsigned int>();
    unsigned int address = vm["address"].as<unsigned int>();
    unsigned int timing_budget = vm["timing-budget"].as<unsigned int>();
    unsigned int range_mode = vm["range-mode"].as<unsigned int>();
    unsigned int mean = vm["mean"].as<unsigned int>();
    bool power = vm["power"].as<bool>();
    int gpio_num = vm["gpio"].as<int>();

    float calib[2] = {0.0f, 1.0f};

    VL53LXX *range_sensor;

    switch(vm["model"].as<unsigned int>()){
        case 0:
            std::cout << "Initializing VL53L0X@" << address << " on bus " << bus << "..." << std::endl;
            range_sensor = (address != 0) ? new VL53L0X(bus, address, gpio_num, true, calib) : new VL53L0X(bus, VL53L0X_ADDRESS_DEFAULT, gpio_num, true, calib);
            break;
        case 1:
            std::cout << "Initializing VL53L1X@" << address << " on bus " << bus << "..." << std::endl;
            range_sensor = (address != 0) ? new VL53L1X(bus, address, gpio_num, true, calib, timing_budget, range_mode) : new VL53L1X(bus, VL53L1X_ADDRESS_DEFAULT, gpio_num, true, calib, timing_budget, range_mode);
            break;
        default:
            std::cout << "Unknown model, exiting..." << std::endl;
            return -1;
    }

    if(!power){
        range_sensor->powerOff();
    }

    if(address != 0){
        range_sensor->setAddress(address);
    }

    int timeout_counter = 0;
    while((!range_sensor->init()) && timeout_counter < 10){
        timeout_counter++;
        usleep(200000);
    }

    if(timeout_counter < 10){
        std::cout << "Initialization OK!" << std::endl;
        bool single = vm["single"].as<bool>();

        unsigned int period = vm["period"].as<unsigned int>();
        std::cout << "Period: " << period << " ms" << std::endl;

        range_sensor->startContinuous(period);

        if(single){
            std::cout << "Distance: " << range_sensor->readRangeSingleMillimeters() << " mm" << std::endl;
        }else{
            while(true){
                unsigned int i;
                unsigned long int range = 0;
                for(i=0; i < mean; i++){
                    range += range_sensor->readRangeContinuousMillimeters();
                }
                std::cout << "Distance: " << (range / i) << " mm" << std::endl;
            }
        }
        
        range_sensor->stopContinuous();
    }else{
        std::cerr << "Initialization timed out!!" << std::endl;
    }

    delete range_sensor;

    return 0;
}
