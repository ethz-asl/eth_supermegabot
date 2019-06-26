/*
 * SignalLoggerExample.cpp
 *
 *  Created on: Feb 29, 2016
 *      Author: Gabriel Hottiger
 */

#include <signal_logger_example/SignalLoggerExample.hpp>

namespace signal_logger_example {

SignalLoggerExample::SignalLoggerExample(NodeHandlePtr nh):
  any_node::Node(nh),
  publishThread_(),
  shouldPublish_(true),
  shouldRead_(true),
  logVar_(0.0),
  time_(ros::TIME_MIN),
  string_{"Hello World"},
  vec_{1, 2, 3},
  arr_{4, 5, 6},
  list_{7, 8, 9},
  set_{1, 1, 2, 3, 5, 8},
  deque_{13, 21, 34, 55},
  pair_{"pair", 2.0},
  map_{{"one", 1}, {"zwei", 2}},
  umap_{{1, 1.1}, {2, 2.22}, {3, 3.333}}
{
  ros::Time::setNow(time_);
}

bool SignalLoggerExample::init()
{
  // The signal logger is created as a SignalLoggerNone, this resets it to be a ros logger
  signal_logger::setSignalLoggerRos(&getNodeHandle());

  //! Initialize the logger
  signal_logger::SignalLoggerOptions siloOptions;
  siloOptions.updateFrequency_ = 10u; // Update is executed at 10 Hz
  siloOptions.maxLoggingTime_ = 12.5; // Log data for 12.5 seconds
  siloOptions.collectScriptFileName_ = "silo_example_script.yaml"; // Name of the collection script, will be created if non-existing
  siloOptions.loggerPrefix_ = "/log/examples"; // This prefix is added to all element names
  signal_logger::logger->initLogger(siloOptions);

  // Add different logger variables and default settings
  signal_logger::add(logVar_, "logVar1", "ns1", "[m]", 1, signal_logger::LogElementAction::SAVE,
   100, signal_logger::BufferType::FIXED_SIZE);
  signal_logger::add(logVar_, "logVar2", "ns1", "[sec]", 2, signal_logger::LogElementAction::SAVE_AND_PUBLISH,
   200, signal_logger::BufferType::FIXED_SIZE);
  signal_logger::add(logVar_, "logVar3", "ns1", "[N]", 1, signal_logger::LogElementAction::PUBLISH,
   100, signal_logger::BufferType::LOOPING);
  signal_logger::add(logVar_, "logVar4", "ns1", "[kg]", 3, signal_logger::LogElementAction::SAVE_AND_PUBLISH ,
   10, signal_logger::BufferType::EXPONENTIALLY_GROWING);

  // Add stl container types
  signal_logger::add(string_, "string", "nsSTL");
  signal_logger::add(vec_, "vec", "nsSTL");
  signal_logger::add(arr_, "arr", "nsSTL");
  signal_logger::add(list_, "list", "nsSTL");
  signal_logger::add(set_, "set", "nsSTL");
  signal_logger::add(deque_, "deque", "nsSTL");
  signal_logger::add(pair_, "pair", "nsSTL");
  signal_logger::add(map_, "map", "nsSTL");
  signal_logger::add(umap_, "unorderedMap", "nsSTL");

  // Call update logger, this loads the variables from the siloOptions.collectScriptFileName_ file.
  signal_logger::logger->updateLogger();

  // Add a fifth element in a different namespace ns2
  signal_logger::add(logVar_, "logVar5", "ns2", "[m]", 5, signal_logger::LogElementAction::SAVE ,
                     1000, signal_logger::BufferType::LOOPING);
  signal_logger::add(logVar_, "logVar6", "ns2", "[deg]", 1, signal_logger::LogElementAction::SAVE_AND_PUBLISH ,
                     50, signal_logger::BufferType::EXPONENTIALLY_GROWING);

  // Call update logger without loading any script to keep the default settings of logVar5
  signal_logger::logger->updateLogger(false);

  // Overwrite the (presumably empty) script containing the configuration of all logger vars
  signal_logger::logger->saveLoggerScript();

  // After updating the logger with the new variables, those can also be modified in code.
  // Get the fourth logger element (this function can throw an out_of_range exception
  try {
    const signal_logger::LogElementInterface & logElement4 = signal_logger::logger->getElement("/log/examples/ns1/logVar4");
    // Set the buffer size of the second element to twice the size of the fourth element
    signal_logger::logger->setElementBufferSize("/log/examples/ns1/logVar2", 2*logElement4.getBuffer().getBufferSize());
  } catch( std::out_of_range & e) {
    std::cout << "No element with name " << "/log/examples/ns1/logVar4" << std::endl;
  }
  // Set the buffer type of the first element to looping
  signal_logger::logger->setElementBufferType("/log/examples/ns1/logVar1", signal_logger::BufferType::LOOPING);

  // Set the divider of the fifth element to 10
  signal_logger::logger->setElementDivider("/log/examples/ns2/logVar5", 10);
  // Set the action of the fourth element to save
  signal_logger::logger->setElementAction("/log/examples/ns1/logVar4", signal_logger::LogElementAction::SAVE);

  // Now lets disable all elements in the second namespace
  signal_logger::logger->disableNamespace("ns2");

  // Disable logVar3
  signal_logger::logger->disableElement("/log/examples/ns1/logVar3");

  // We can also reset the max logging time (has only on the next logger start)
  signal_logger::logger->setMaxLoggingTime(35.0);

  // The ros logger publishes the elements in a seperate thread that you should provide externally
  publishThread_ = std::thread(&SignalLoggerExample::publishWorker, this);

  // A second thread reads values from element logVar1 at some rate
  readThread_ = std::thread(&SignalLoggerExample::readWorker, this);

  // Finally let's start the logger
  signal_logger::logger->startLogger();

  addWorker("SignalLoggerExample::updateWorker", 0.1, &SignalLoggerExample::update, this);

  return true;
}

void SignalLoggerExample::cleanup()
{
  // The logger can save various log file types (such as .csv, .bag and a minimal binary file for parsing with matlab)
  signal_logger::logger->saveLoggerData( { signal_logger::LogFileType::CSV,
                                           signal_logger::LogFileType::BAG,
                                           signal_logger::LogFileType::BINARY } );
  // Stop the logger, this can be done either before or after the saving
  signal_logger::logger->stopLogger();

  // Join other threads
  shouldPublish_.store(false);
  shouldRead_.store(false);
  publishThread_.join();
  readThread_.join();

  // Cleanup
  signal_logger::logger->cleanup();
}

bool SignalLoggerExample::update(const any_worker::WorkerEvent& event) {

  // Static counter for execution handling
  static unsigned int counter = 0;
  counter++;

  // Collect data
  signal_logger::logger->collectLoggerData();

  // Update logger vars and time


  logVar_ += 0.1;
  time_ += ros::Duration(1.0/10);
  ros::Time::setNow(time_);

  // Enable names
  if(counter ==  5) {
    // Set a property of a disabled element --> perfectly fine to do this
    signal_logger::logger->setElementDivider("/log/examples/ns1/logVar3", 2);
    string_ = "Hello";
  } else if(counter ==20) {
    string_ = "Hello Extended World";
  } else if(counter == 50) {
    signal_logger::logger->stopLogger();
    signal_logger::logger->disableNamespace("ns1");
    signal_logger::logger->enableElement("/log/examples/ns2/logVar5");
    signal_logger::logger->enableElement("/log/examples/ns1/logVar3");
    signal_logger::logger->startLogger();
  } else if(counter == 100) {
    signal_logger::logger->enableElement("/log/examples/ns1/logVar1");

  }

  return true;
}

void SignalLoggerExample::publishWorker() {
  while(shouldPublish_) {
    signal_logger::logger->publishData();
    usleep(100000);
  }
}

void SignalLoggerExample::readWorker() {
  static unsigned int counter = 0;

  while(shouldRead_) {
    if( (++counter % 20) == 0 ) {
      signal_logger::vector_type<double> v;
      try {
        v = signal_logger::logger->readNewValues<double>("/log/examples/ns1/logVar1");
      } catch( std::out_of_range & e) {
        std::cout << "No element with name " << "/log/examples/ns1/logVar1" << std::endl;
      }
      if(!v.empty()) {
        std::cout << "Got new values of /log/examples/ns1/logVar1: " << std::endl;
        for(unsigned int i = 0; i < v.size(); ++i) { std::cout << v[i] << ", "; }
        std::cout << std::endl;
      }
    }
    usleep(100000);
  }
}

} /* namespace signal_logger_example */
