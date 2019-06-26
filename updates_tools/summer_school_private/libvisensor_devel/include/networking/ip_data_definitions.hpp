/*
 * Copyright (c) 2014, Skybotix AG, Switzerland (info@skybotix.com)
 * Copyright (c) 2014, Autonomous Systems Lab, ETH Zurich, Switzerland
 *
 * All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef IP_DATA_DEFINITIONS_HPP_
#define IP_DATA_DEFINITIONS_HPP_

//as this is a shared header, only include the header below if we are building the libvisensor
#ifdef VISENSOR_EXPORT
# include <config/config.hpp>
#endif

#ifdef __APPLE__
  #include "networking/endian.h" // compatibility header for endian.h
#endif	/* __APPLE__ */

#include <netinet/in.h> //for ntohl() and htonl()
#include <boost/array.hpp>
#include <boost/foreach.hpp>


namespace IpComm {

enum HeaderType{
  //Config server
  REQUEST_FPGA_INFO,
  FPGA_INFO,
  REQUEST_SENSOR_INFO,
  HOST_INITIALIZED,
  SENSOR_INFO,
  READ_UBI_REGISTER,
  WRITE_UBI_REGISTER,
  READ_FPGA_REGISTER,
  WRITE_FPGA_REGISTER,
  START_SENSOR,
  STOP_SENSOR,
  SEND_FILE,
  REQUEST_FILE,
  READ_CAMERA_CALIBRATION,
  WRITE_CAMERA_CALIBRATION,

  //Serial server (requests)
  SERIAL_SET_BAUDRATE,      //one way + ack
  SERIAL_SET_DELIMITER,     //one way + ack
  SERIAL_DATA,              //serial data (both ways)

  //GENERAL PACKETS
  ACK,
  NACK
};

typedef boost::array<uint32_t, 3> HeaderPayload;
struct Header
{
  uint32_t timestamp;
  uint32_t data_size;
  uint32_t data_id;

  Header(){
    timestamp=0;
    data_size=0;
    data_id=0;
  }

  Header(HeaderPayload header_payload){
    setSerialized(header_payload);
  }

  HeaderPayload getSerialized() {
    HeaderPayload header_payload;
    header_payload[0]=htonl(timestamp);
    header_payload[1]=htonl(data_size);
    header_payload[2]=htonl(data_id);
    return header_payload;
  }

  void setSerialized(HeaderPayload header_payload) {
    timestamp=ntohl(header_payload[0]);
    data_size=ntohl(header_payload[1]);
    data_id=ntohl(header_payload[2]);
  }

  void print()
  {
  }
};

typedef boost::array<uint32_t, 6> FpgaInfoPayload;
struct FpgaInfo
{
  uint32_t timestamp;
  uint32_t fpgaId;
  uint32_t firmwareVersionMajor;
  uint32_t firmwareVersionMinor;
  uint32_t firmwareVersionPatch;
  uint32_t numOfSensors;

  FpgaInfo() {
    timestamp = 0;
    fpgaId = 0;
    firmwareVersionMajor = 0;
    firmwareVersionMinor = 0;
    firmwareVersionPatch = 0;
    numOfSensors = 0;
  }

  FpgaInfo(FpgaInfoPayload fpgainfo_payload) {
    setSerialized(fpgainfo_payload);
  }

  FpgaInfoPayload getSerialized() {
    FpgaInfoPayload fpgainfo_payload;
    fpgainfo_payload[0] = htonl(timestamp);
    fpgainfo_payload[1] = htonl(fpgaId);
    fpgainfo_payload[2] = htonl(firmwareVersionMajor);
    fpgainfo_payload[3] = htonl(firmwareVersionMinor);
    fpgainfo_payload[4] = htonl(firmwareVersionPatch);
    fpgainfo_payload[5] = htonl(numOfSensors);
    return fpgainfo_payload;
  }

  void setSerialized(FpgaInfoPayload fpgainfo_payload) {
    timestamp = ntohl(fpgainfo_payload[0]);
    fpgaId = ntohl(fpgainfo_payload[1]);
    firmwareVersionMajor = ntohl(fpgainfo_payload[2]);
    firmwareVersionMinor = ntohl(fpgainfo_payload[3]);
    firmwareVersionPatch = ntohl(fpgainfo_payload[4]);
    numOfSensors = ntohl(fpgainfo_payload[5]);
  }
};

typedef boost::array<uint32_t, 2> StartSensorPayload;
struct StartSensor
{
  uint32_t id;
  uint32_t rate;

  StartSensor() : id(0), rate(0) {};

  StartSensor(StartSensorPayload startPackagePayload){
    setSerialized(startPackagePayload);
  }

  StartSensorPayload getSerialized() {
    StartSensorPayload startPackagePayload;
    startPackagePayload[0]=htonl(id);
    startPackagePayload[1]=htonl(rate);
    return startPackagePayload;
  }

  void setSerialized(StartSensorPayload startPackagePayload) {
    id=ntohl(startPackagePayload[0]);
    rate=ntohl(startPackagePayload[1]);
  }
};

typedef boost::array<uint32_t, 3> SensorInfoPayload;
struct SensorInfo
{
  uint32_t sensor_type;
  uint32_t sensor_id;

  SensorInfo(){
    sensor_type = 0;
    sensor_id = 0;
  }

  SensorInfo(SensorInfoPayload sensor_info_payload){
    setSerialized(sensor_info_payload);
  }

  SensorInfoPayload getSerialized() {
    SensorInfoPayload sensor_info_payload;
    sensor_info_payload[0]=htonl(sensor_type);
    sensor_info_payload[1]=htonl(sensor_id);
    return sensor_info_payload;
  }

  void setSerialized(SensorInfoPayload sensor_info_payload) {
    sensor_type=ntohl(sensor_info_payload[0]);
    sensor_id=ntohl(sensor_info_payload[1]);
  }
};

typedef boost::array<uint32_t, 4> BusPackagePayload;
struct BusPackage
{
  uint32_t sensor_id;
  uint32_t NumBits; // 8 or 16 Bit
  uint32_t registerAddress;  // always 8 Bit
  uint32_t value;

  BusPackage(){
    sensor_id = 0;
    NumBits = 0;
    registerAddress = 0;
    value = 0;
  }

  BusPackage(BusPackagePayload busPackagePayload){
    setSerialized(busPackagePayload);
  }

  BusPackagePayload getSerialized() {
    BusPackagePayload busPackagePayload;
    busPackagePayload[0]=htonl(sensor_id);
    busPackagePayload[1]=htonl(NumBits);
    busPackagePayload[2]=htonl(registerAddress);
    busPackagePayload[3]=htonl(value);
    return busPackagePayload;
  }

  void setSerialized(BusPackagePayload sendor_info_payload) {
    sensor_id=ntohl(sendor_info_payload[0]);
    NumBits=ntohl(sendor_info_payload[1]);
    registerAddress=ntohl(sendor_info_payload[2]);
    value=ntohl(sendor_info_payload[3]);
  }
};

typedef boost::array<uint64_t, 2> TimeSyncPayload;
struct TimeSync
{
  uint64_t host_time;
  uint64_t fpga_time;

  TimeSync(){
    host_time = 0;
    fpga_time = 0;
  }

  TimeSync(TimeSyncPayload time_sync_payload){
    setSerialized(time_sync_payload);
  }

  TimeSyncPayload getSerialized() {
    TimeSyncPayload time_sync_payload;
    time_sync_payload[0]=htobe64(host_time); //TODO(gohlp): find OSX compatible equivalent for htobe64
    time_sync_payload[1]=htobe64(fpga_time);
    return time_sync_payload;
  }

  void setSerialized(TimeSyncPayload time_sync_payload) {
    host_time=be64toh(time_sync_payload[0]);
    fpga_time=be64toh(time_sync_payload[1]);
  }
};

typedef boost::array<uint64_t, 2> CalibrationIdPayload;
struct CalibrationId {
  unsigned int port_id;
  unsigned int slot_id;

  CalibrationId() : port_id(0), slot_id(0) {};

  CalibrationId(CalibrationIdPayload calib_id_payload)
  {
    setSerialized(calib_id_payload);
  }

  CalibrationIdPayload getSerialized() {
    CalibrationIdPayload calib_id_payload;
    calib_id_payload[0] = port_id;
    calib_id_payload[1] = slot_id;

    return calib_id_payload;
  }

  void setSerialized(CalibrationIdPayload calib_id_payload) {
    port_id = calib_id_payload[0];
    slot_id = calib_id_payload[1];
  }
};

typedef boost::array<uint64_t, 22> CameraCalibrationPayload;
struct CameraCalibration {

  //intrinsics
  double focal_point[2];
  double principal_point[2];

  //distortion
  double distortion[5];

  //extrinsics
  double R[9];
  double t[3];

  //valid flag
  uint32_t valid;

  CameraCalibration()
  {
    BOOST_FOREACH( double &i, focal_point ) {i = 0.0;}
    BOOST_FOREACH( double &i, principal_point ) {i = 0.0;}
    BOOST_FOREACH( double &i, distortion ) {i = 0.0;}
    BOOST_FOREACH( double &i, R ) {i = 0.0;}
    BOOST_FOREACH( double &i, t ) {i = 0.0;}
    valid = 0;
  }

  CameraCalibration(CameraCalibrationPayload cam_calib_payload)
  {
    setSerialized(cam_calib_payload);
  }

  CameraCalibrationPayload getSerialized() {
    CameraCalibrationPayload cam_calib_payload;

    int i=0;
    for(int j=0; j<2; j++)
      memcpy(&cam_calib_payload[i++], &focal_point[j], sizeof(double));

    for(int j=0; j<2; j++)
      memcpy(&cam_calib_payload[i++], &principal_point[j], sizeof(double));

    for(int j=0; j<5; j++)
      memcpy(&cam_calib_payload[i++], &distortion[j], sizeof(double));

    for(int j=0; j<9; j++)
      memcpy(&cam_calib_payload[i++], &R[j], sizeof(double));

    for(int j=0; j<3; j++)
      memcpy(&cam_calib_payload[i++], &t[j], sizeof(double));

    cam_calib_payload[i++] = valid;

    return cam_calib_payload;
  }

  void setSerialized(CameraCalibrationPayload cam_calib_payload) {

    int i=0;
    for(int j=0; j<2; j++)
      memcpy(&focal_point[j], &cam_calib_payload[i++], sizeof(double));

    for(int j=0; j<2; j++)
      memcpy(&principal_point[j], &cam_calib_payload[i++], sizeof(double));

    for(int j=0; j<5; j++)
      memcpy(&distortion[j], &cam_calib_payload[i++], sizeof(double));

    for(int j=0; j<9; j++)
      memcpy(&R[j], &cam_calib_payload[i++], sizeof(double));

    for(int j=0; j<3; j++)
      memcpy(&t[j], &cam_calib_payload[i++], sizeof(double));

    valid = cam_calib_payload[i++];
  }

};

}

#endif /* IP_DATA_DEFINITIONS_HPP_ */
