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
        ("period", po::value<unsigned int>()->default_value(100), "Refresh period in milliseconds.")
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

    VL53LXX *range_sensor;

    switch(vm["model"].as<unsigned int>()){
        case 0:
            range_sensor = (address != 0) ? new VL53L0X(bus, address) : new VL53L0X(bus);
            break;
        case 1:
            range_sensor = (address != 0) ? new VL53L1X(bus, address) : new VL53L1X(bus);
            break;
        default:
            std::cout << "Unknown model, exiting..." << std::endl;
            return -1;
    }

    bool single = vm["single"].as<bool>();

    if(single){
        std::cout << "Distance: " << range_sensor->readRangeSingleMillimeters() << "mm." << std::endl;
    }else{
        unsigned int period = vm["period"].as<unsigned int>();

        range_sensor->startContinuous(period);

        while(true){
            std::cout << "Distance: " << range_sensor->readRangeContinuousMillimeters() << "mm." << std::endl;
            usleep(period*1000);
        }

        range_sensor->stopContinuous();
    }

    return 0;
}
