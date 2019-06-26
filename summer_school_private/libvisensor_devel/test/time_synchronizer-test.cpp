// Bring in my package's API, which is what I'm testing
#include <cmath>
#include <fstream>
#include <iostream>
#include <sys/time.h>
#include <vector>

#include "visensor/visensor_constants.hpp"
#include "synchronization/time_synchronizer.hpp"

// Bring in gtest
#include <gtest/gtest.h>

static const uint64_t FPGA_TIME_MULTIPLIER = visensor::SECOND_IN_NANOSECOND/static_cast<uint64_t>(FPGA_TIME_COUNTER_FREQUENCY);


/**
 * \class FpgaClockModell

 * \brief Builds a clock model to simulate random transmission times and different clock frequencies
 *
 * The model use a equal distributed transmission time between FPGA and PC.
 * To model the different clock frequencies a linear model first order is used.
 *
 * PCTime = alpha * FpgaTime + FpgaOffset
 *
 * alpha is the skew (constant random variable between 9900- 10100)
 * FpgaOffset is the PCTime when the FPGA had the zero wrap around
 *
 * \author Lukas Schmid
 *
 * \version $Revision: 1.0 $
 *
 * \date $Date: 2015/05/8 $
 *
 * Contact: lukas.schmid@skybotix.com
 *
 */
class FpgaClockModell
{
 public:

  FpgaClockModell(uint64_t fpgaZeroPoint)
  {

    /* init model*/
    fpgaOffsetSim = fpgaZeroPoint;
    randDelayMultiplicator = 100 * visensor::MILLISECOND_IN_NANOSECOND;
    slopeSim = 10000*(0.1*(getEqualRand(1) - 0.5)+1);
    minDelaySim = getEqualRand(1e4);
    max_delay_ = 0;
    min_delay_ = visensor::SECOND_IN_NANOSECOND;

    std::clog << "Timestamps; fpgaOffsetSim; " << fpgaOffsetSim << "; randDelayMultiplicator; "
              << randDelayMultiplicator << "; slopeSim; " << slopeSim << "; minDelaySim; "
              << minDelaySim << "; " << std::endl;
  }

  /*
   * Returns a equal distributed random variable between multiplier and zero
   *
   * \param multiplier to scale the range of random variables
   *
   * \return
   *
   */
  double getEqualRand(double multiplier)
  {
    return ((static_cast<double>(rand()) * multiplier) / static_cast<double>(RAND_MAX));
  }

  /*
   * Returns a exponential distributed random variable
   *
   * exp(R) * multiplier
   * where R is equal distributed from - 15 to 0
   * gives a total range from 3.05902320502e-07 .. 1 * multiplier
   *
   * \param multiplier to scale the range of random variables
   *
   * \return
   *
   */
  double getExponentialRand(double multiplier)
  {
    return exp(getEqualRand(-15)) * multiplier;
  }

  /*
   * Returns a equal distributed random delay for the transmission
   *
   * \return
   *
   */
  uint64_t getRandDelay()
  {
    uint64_t delay = static_cast<uint64_t>(getExponentialRand(randDelayMultiplicator));
    if (delay > max_delay_)
      max_delay_= delay;
    if (delay < min_delay_)
      min_delay_ = delay;
    return delay;
  }

  /*
   * Returns the PcTime without any additional random variable according to the simulated model
   *
   * fpgaTime FPGA time to convert
   *
   * \return
   *
   */
  uint64_t fpgaTime2PcTime(const uint64_t fpgaTime)
  {
    return static_cast<uint64_t>(static_cast<double>(fpgaTime) * slopeSim) + fpgaOffsetSim;
  }

  /*
   * Returns the FPGA time without any additional random variable according to the simulated model
   *
   * pcTime PC time to convert
   *
   * \return
   *
   */
  uint64_t pcTime2fpgaTime(const uint64_t pcTime)
  {
    return static_cast<uint64_t>((pcTime - fpgaOffsetSim) / slopeSim);
  }

  /*
   * Calculates the FPGA time and the PC responding time given the request time
   * and two random transmission delays. It also returns to the FPGA time corresponding PC time.
   *
   * requestTime  PC request time as a start point for the Ping Pong.
   * fpgaTime     time on the FPGA of the receiving the request = time of sending the response (in FPGA ticks)
   * pcTimeTrue   time on the PC of the FPGA receiving and sending
   * responseTime PC receiving time of the Ping Pong response
   *
   * \return
   *
   */
  void getPingPongTimes(const uint64_t requestTime, uint64_t& fpgaTime, uint64_t& pcTimeTrue,
                        uint64_t& responseTime)
  {
    fpgaTime = pcTime2fpgaTime(requestTime)
        + static_cast<uint64_t>((minDelaySim + getRandDelay()) / slopeSim);
    pcTimeTrue = fpgaTime2PcTime(fpgaTime);
    responseTime = pcTimeTrue + minDelaySim + getRandDelay();
  }

  /*
   * Calculates the PC receiving time of a FPGA frame with one random transmission delays.
   *
   * fpgaTime     time on the FPGA of sending the frame (in FPGA ticks)
   * pcTimeTrue   time on the PC at the FPGA sending moment
   * responseTime PC receiving time of the Ping Pong response
   *
   * \return
   *
   */
  void getPcTimestamp(const uint64_t fpgaTime, uint64_t& responseTime, uint64_t& pcTimeTrue)
  {
    pcTimeTrue = fpgaTime2PcTime(fpgaTime);
    responseTime = pcTimeTrue + minDelaySim + getRandDelay();
  }


  double max_delay_;
  double min_delay_;

 private:

  double randDelayMultiplicator;
// skew of the both clocks
  double slopeSim;
//probagation delay in ns
  double minDelaySim;
//value of the PC time to FPGA time 0
  double fpgaOffsetSim;

};


/**
 * Helper function to split a string into elements
 */
std::vector<std::string> &split(const std::string &s, char delim, std::vector<std::string> &elems) {
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, delim)) {
        elems.push_back(item);
    }
    return elems;
}

/**
 * Helper function to split a string into elements
 */
std::vector<std::string> split(const std::string &s, char delim) {
    std::vector<std::string> elems;
    split(s, delim, elems);
    return elems;
}


class Test_time_synchronizer : public ::testing::Test {
 protected:
  virtual void SetUp()
  {
    timeSynchronizer_ = new timesync::TimeSynchronizer(0, (uint64_t)1e9/(uint64_t)100000);
    timestamp_log_filename_ = "timestamps.log";
    results_log_filename_ = "timesync.log";

    logfile.open(timestamp_log_filename_, std::ios::in);
    if (logfile.good()) {
      logfile.close();
      std::remove(timestamp_log_filename_.c_str());
    }
    logfile.open (timestamp_log_filename_, std::ios::out | std::ios::app);

    resfile.open(results_log_filename_, std::ios::in);
    if (resfile.good()) {
      resfile.close();
      std::remove(results_log_filename_.c_str());
    }
    resfile.open (results_log_filename_, std::ios::out | std::ios::app);


    log_timestamps("requestTime; pcTime at FPGA receiving; fpga_time; responseTime; ");
    log_results("fpgaTime; true pcTime at FPGA receiving; synchronizedTime; diff; ");
  }
  virtual void TearDown() {
    logfile.close();
    resfile.close();

    if (file_exists(timestamp_log_filename_))
      std::remove(timestamp_log_filename_.c_str());

    if (file_exists(results_log_filename_))
      std::remove(results_log_filename_.c_str());
  }

  inline bool file_exists(const std::string& name)
  {
    if (FILE *file = fopen(name.c_str(), "r")) {
      fclose(file);
      return true;
    } else {
      return false;
    }
  }

  timesync::TimeSynchronizer * timeSynchronizer_;
  std::fstream logfile;
  std::fstream resfile;
  std::string timestamp_log_filename_;
  std::string results_log_filename_;
  uint64_t requestTime;
  uint64_t fpgaTime;
  uint64_t responseTime;
  uint64_t synchronizedTime;
  uint64_t synchronizedTimeTic;
  uint64_t fpga_time_pintPong;
  uint64_t pcTimeTrue;
  uint64_t responseTime_old;

void log_timestamps(std::string str) {
  logfile << str << std::endl;
}
void log_results(std::string str) {
  resfile << str << std::endl;
}

};


/**
 * Test the time synchronization between FPGA and PC.
 *
 * It uses a linear model with equal distributed transmission delays.
 * After a initialization face with multiple Ping Pongs, the time synchronization
 * is updated every frame (like IMU frames) and the accuracy tested to the ground truth.
 * At the moment the tolerance is 5ms.
 */
TEST_F(Test_time_synchronizer, TestTimeSynchronizerSimulatedData)
{

  /* initialize random seed: */
//  srand(timeSynchronizer_->getSystemTime());
  srand(100);

  // Test setup
  const int TOLERANCE = 1 * visensor::MILLISECOND_IN_NANOSECOND;
  const int TOLERANCE_STEP = 10 * visensor::MUSECOND_IN_NANOSECOND;


  const int N_INIT_TIMESYNC = 10;
  const int TIMESTAMP_RATE = 20;
  const int PING_PONG_RATE = 1;
  const uint64_t TEST_TIME = 0x100100000; // Test time in FPGA Ticks (a bit more then the 32bit clock overflow)
  uint64_t error_old = -1; //maximal error


  // init test variables
  requestTime = 123456789012;
  FpgaClockModell fpgaClockModel(requestTime);

  for (int i = 0; i < N_INIT_TIMESYNC; ++i) {
    fpgaClockModel.getPingPongTimes(requestTime, fpgaTime, pcTimeTrue, responseTime);
//    std::clog << "Init: requestTime: " << requestTime << " pcTime at FPGA receiving " << pcTimeTrue
//              << " fpga_time: " << fpgaTime << " responseTime: " << responseTime << std::endl;
    timeSynchronizer_->initialUpdate(requestTime, static_cast<uint32_t>(fpgaTime), responseTime);

    requestTime = responseTime + visensor::SECOND_IN_NANOSECOND/(TIMESTAMP_RATE/10);
  }

  responseTime_old = responseTime;
  fpgaTime = fpgaClockModel.pcTime2fpgaTime(responseTime);
  uint64_t fpgaTimeStart = fpgaTime;
  while (fpgaTime < fpgaTimeStart + TEST_TIME) {
    for (int j = 0; j < TIMESTAMP_RATE/PING_PONG_RATE; ++j) {
      fpgaTime = fpgaTime + visensor::SECOND_IN_NANOSECOND/(TIMESTAMP_RATE * FPGA_TIME_MULTIPLIER);
      fpgaClockModel.getPcTimestamp(fpgaTime, responseTime, pcTimeTrue);

     log_timestamps("0; " + std::to_string(pcTimeTrue)
       + "; " + std::to_string(fpgaTime)
       + "; " + std::to_string(responseTime) + "; ");
      if (responseTime > responseTime_old) {

        timeSynchronizer_->updateTime(static_cast<uint32_t>(fpgaTime), responseTime);
        synchronizedTime = timeSynchronizer_->getSynchronizedTime(static_cast<uint32_t>(fpgaTime));

        log_results(std::to_string(static_cast<uint32_t>(fpgaTime)) + "; " + std::to_string(pcTimeTrue)
          + "; " + std::to_string(synchronizedTime)
          + "; " + std::to_string(abs(synchronizedTime - pcTimeTrue)) + "; " );

        //check if there are any bigger steps of the error/ offset
        if (error_old != static_cast<uint64_t>(-1))
          EXPECT_GE(TOLERANCE_STEP, abs(error_old - (synchronizedTime - pcTimeTrue)));

        //check if the synchronized time is in accurate enough
        EXPECT_GE(TOLERANCE, abs(synchronizedTime - pcTimeTrue));

        error_old = synchronizedTime - pcTimeTrue;
        responseTime_old = responseTime;
      }
    }
    requestTime = fpgaClockModel.fpgaTime2PcTime(fpgaTime -10*visensor::MILLISECOND_IN_NANOSECOND/FPGA_TIME_MULTIPLIER) ;
    fpgaClockModel.getPingPongTimes(requestTime, fpga_time_pintPong, pcTimeTrue, responseTime);

    log_timestamps( std::to_string(requestTime)
      + "; " + std::to_string(pcTimeTrue)
      + "; " + std::to_string(fpgaTime)
      + "; " + std::to_string(responseTime) + "; ");

    synchronizedTime = timeSynchronizer_->getSynchronizedTime(static_cast<uint32_t>(fpgaTime));
    responseTime_old = responseTime;

  }

  std::cout << "Max delay was: " << fpgaClockModel.max_delay_ << std::endl;
  std::cout << "Min delay was: " << fpgaClockModel.min_delay_ << std::endl;

}

// Run all the tests that were declared with TEST()
int main(int argc, char **argv)
{
  try {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
  } catch (std::exception &e) {
    std::cerr << "Unhandled Exception: " << e.what() << std::endl;
  }

}
