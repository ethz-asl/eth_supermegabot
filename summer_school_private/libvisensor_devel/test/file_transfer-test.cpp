#include <fstream>
#include <iostream>
#include <string>

// Bring in gtest
#include <gtest/gtest.h>

#include "communication_layers/ssh_connections.hpp"
#include "networking/file_transfer.hpp"

class Visensor_file_transfer : public ::testing::Test
{
 protected:
  virtual void SetUp()
  {
    ssh_connection_ = boost::make_shared<SshConnection>();
    file_transfer_ = boost::make_shared<visensor::FileTransfer>(ssh_connection_);

    EXPECT_NE(ssh_connection_->sshConnect(SENSOR_IP_, "root", ""), NULL);
  }

  std::string random_string(size_t length)
  {
    auto randchar = []() -> char
    {
      const char charset[] =
      "0123456789"
      "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
      "abcdefghijklmnopqrstuvwxyz";
      const size_t max_index = (sizeof(charset) - 1);
      return charset[ rand() % max_index ];
    };
    std::string str(length, 0);
    std::generate_n(str.begin(), length, randchar);
    return str;
  }

  const std::string LOCAL_TEST_FILE_PATH = "testfile.txt";
  const std::string REMOTE_TEST_PATH = "/home/root/test/testfile.cfg";
  const std::string REMOTE_TEMP_TEST_PATH = "/media/ram/test/testfile.cfg";
  visensor::FileTransfer::Ptr file_transfer_;
  SshConnection::Ptr ssh_connection_;
  const std::string SENSOR_IP_ = "10.0.0.1";
};

TEST_F(Visensor_file_transfer, TestVisensorFileTransferWriteAndCopyBackRam)
{
  std::cout << "TestVisensorFileTransferWriteAndCopyBackRam test " << std::endl;
  std::string test_string = random_string(1000000);
  std::string read_back_sting;
  std::ifstream local_file;

  file_transfer_->writeRemoteFile(REMOTE_TEMP_TEST_PATH, test_string);
  file_transfer_->downloadFile(LOCAL_TEST_FILE_PATH, REMOTE_TEMP_TEST_PATH);

  local_file.open(LOCAL_TEST_FILE_PATH.c_str());
  local_file >> read_back_sting;
  local_file.close();
  std::remove(LOCAL_TEST_FILE_PATH.c_str());
  file_transfer_->deleteRemoteFile(REMOTE_TEMP_TEST_PATH);
  EXPECT_EQ(read_back_sting, test_string);
}

TEST_F(Visensor_file_transfer, TestVisensorFileTransferWriteAndCopyBack)
{
  std::cout << "TestVisensorFileTransferWriteAndCopyBack test " << std::endl;
  std::string test_string = random_string(1000000);
  std::string read_back_sting;
  std::ifstream local_file;

  file_transfer_->writeRemoteFile(REMOTE_TEST_PATH, test_string, true);
  file_transfer_->downloadFile(LOCAL_TEST_FILE_PATH, REMOTE_TEST_PATH);

  local_file.open(LOCAL_TEST_FILE_PATH.c_str());
  local_file >> read_back_sting;
  local_file.close();
  std::remove(LOCAL_TEST_FILE_PATH.c_str());
  file_transfer_->deleteRemoteFile(REMOTE_TEST_PATH);
  EXPECT_EQ(read_back_sting, test_string);
}

TEST_F(Visensor_file_transfer, TestVisensorFileTransferCopyAndReadBackRam)
{
  std::cout << "TestVisensorFileTransferCopyAndReadBackRam test " << std::endl;
  std::string test_string = random_string(1000000);
  std::string read_back_sting;
  std::ofstream local_file;

  local_file.open(LOCAL_TEST_FILE_PATH.c_str());
  local_file << test_string;
  local_file.close();

  file_transfer_->uploadFile(LOCAL_TEST_FILE_PATH, REMOTE_TEMP_TEST_PATH);
  file_transfer_->readRemoteFile(REMOTE_TEMP_TEST_PATH, &read_back_sting);

  std::remove(LOCAL_TEST_FILE_PATH.c_str());
  file_transfer_->deleteRemoteFile(REMOTE_TEMP_TEST_PATH);
  EXPECT_EQ(read_back_sting, test_string);
}

TEST_F(Visensor_file_transfer, TestVisensorFileTransferCopyAndReadBack)
{
  std::cout << "TestVisensorFileTransferCopyAndReadBack test " << std::endl;
  std::string test_string = random_string(10);
  std::string read_back_sting;
  std::ofstream local_file;

  local_file.open(LOCAL_TEST_FILE_PATH.c_str());
  local_file << test_string;
  local_file.close();

  file_transfer_->uploadFile(LOCAL_TEST_FILE_PATH, REMOTE_TEST_PATH, true);
  file_transfer_->readRemoteFile(REMOTE_TEST_PATH, &read_back_sting);

  std::remove(LOCAL_TEST_FILE_PATH.c_str());
  file_transfer_->deleteRemoteFile(REMOTE_TEST_PATH);
  EXPECT_EQ(test_string, read_back_sting);
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
