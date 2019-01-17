#ifndef _VL53L1X_H
#define _VL53L1X_H

#include <stdio.h>
#include <string.h>
#include <time.h>

#include <vl53lXx/vl53lxx.hpp>
#include <vl53lXx/vl53l1x_defines.hpp>

class VL53L1X : public VL53LXX
{
  public:

    struct RangingData
    {
      uint16_t range_mm;
      VL53L1X_DEFINITIONS::RangeStatus range_status;
      float peak_signal_count_rate_MCPS;
      float ambient_count_rate_MCPS;
    };

    RangingData ranging_data;

    VL53L1X(uint8_t port, const uint8_t address = VL53L1X_ADDRESS_DEFAULT, const int16_t xshutGPIOPin = -1, bool ioMode2v8 = true, float *calib = DEFAULT_CALIB, unsigned int timing_budget = 50000, unsigned int range_mode = VL53L1X_DEFINITIONS::Long);
    ~VL53L1X();

    void setAddress(uint8_t new_addr);

    bool init();

    void writeReg(uint16_t reg, uint8_t value);
    void writeReg16Bit(uint16_t reg, uint16_t value);
    void writeReg32Bit(uint16_t reg, uint32_t value);
    uint8_t readReg(uint16_t reg);
    uint16_t readReg16Bit(uint16_t reg);
    uint32_t readReg32Bit(uint16_t reg);

    bool setDistanceMode(VL53L1X_DEFINITIONS::DistanceMode mode);
    VL53L1X_DEFINITIONS::DistanceMode getDistanceMode() { return distance_mode; }

    bool setMeasurementTimingBudget(uint32_t budget_us);
    uint32_t getMeasurementTimingBudget();

    void startContinuous(uint32_t period_ms = 0);
    void stopContinuous();
    uint16_t readRangeSingleMillimeters(bool blocking = true);
    uint16_t readRangeContinuousMillimeters(bool blocking = true) { return readRangeSingleMillimeters(blocking); } // alias of read()

    // check if sensor has new reading available
    // assumes interrupt is active low (GPIO_HV_MUX__CTRL bit 4 is 1)
    bool dataReady() { return (readReg(VL53L1X_DEFINITIONS::GPIO__TIO_HV_STATUS) & 0x01) == 0; }

    static const char * rangeStatusToString(VL53L1X_DEFINITIONS::RangeStatus status);

    void setTimeout(uint16_t timeout) { io_timeout = timeout; }
    uint16_t getTimeout() { return io_timeout; }
    bool timeoutOccurred();

  private:
    // value used in measurement timing budget calculations
    // assumes PresetMode is LOWPOWER_AUTONOMOUS
    //
    // vhv = LOWPOWER_AUTO_VHV_LOOP_DURATION_US + LOWPOWERAUTO_VHV_LOOP_BOUND
    //       (tuning parm default) * LOWPOWER_AUTO_VHV_LOOP_DURATION_US
    //     = 245 + 3 * 245 = 980
    // TimingGuard = LOWPOWER_AUTO_OVERHEAD_BEFORE_A_RANGING +
    //               LOWPOWER_AUTO_OVERHEAD_BETWEEN_A_B_RANGING + vhv
    //             = 1448 + 2100 + 980 = 4528
    static const uint32_t TimingGuard = 4528;

    // value in DSS_CONFIG__TARGET_TOTAL_RATE_MCPS register, used in DSS
    // calculations
    static const uint16_t TargetRate = 0x0A00;

    //Timing budget and mode for ranging
    unsigned int timing_budget, range_mode;

    // for storing values read from RESULT__RANGE_STATUS (0x0089)
    // through RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0_LOW
    // (0x0099)
    struct ResultBuffer
    {
      uint8_t range_status;
    // uint8_t report_status: not used
      uint8_t stream_count;
      uint16_t dss_actual_effective_spads_sd0;
   // uint16_t peak_signal_count_rate_mcps_sd0: not used
      uint16_t ambient_count_rate_mcps_sd0;
   // uint16_t sigma_sd0: not used
   // uint16_t phase_sd0: not used
      uint16_t final_crosstalk_corrected_range_mm_sd0;
      uint16_t peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
    };

    // making this static would save RAM for multiple instances as long as there
    // aren't multiple sensors being read at the same time (e.g. on separate
    // I2C buses)
    ResultBuffer results;

    uint16_t io_timeout;
    bool did_timeout;
    time_t timeout_start;

    uint16_t fast_osc_frequency;
    uint16_t osc_calibrate_val;

    bool calibrated;
    uint8_t saved_vhv_init;
    uint8_t saved_vhv_timeout;

    VL53L1X_DEFINITIONS::DistanceMode distance_mode;

    // Record the current time to check an upcoming timeout against
    void startTimeout() { timeout_start = time(0); }

    // Check if timeout is enabled (set to nonzero value) and has expired
    bool checkTimeoutExpired() {return ((io_timeout > 0) && ((difftime(time(0), timeout_start)*1000) > io_timeout)); }

    void setupManualCalibration();
    void readResults();
    void updateDSS();
    void getRangingData();

    int tmp_data[1024];
    int tmp_data_old[1024];

    static uint32_t decodeTimeout(uint16_t reg_val);
    static uint16_t encodeTimeout(uint32_t timeout_mclks);
    static uint32_t timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us);
    static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us);
    uint32_t calcMacroPeriod(uint8_t vcsel_period);

    // Convert count rate from fixed point 9.7 format to float
    float countRateFixedToFloat(uint16_t count_rate_fixed) { return (float)count_rate_fixed / (1 << 7); }
};

#endif
