/*
 * Copyright (c) 2015, Skybotix AG, Switzerland (info@skybotix.com)
 * Copyright (c) 2015, Autonomous Systems Lab, ETH Zurich, Switzerland
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

#ifndef VISENSOR_DATATYPES_HPP_
#define VISENSOR_DATATYPES_HPP_

#include <memory>
#include <stdint.h> // for uint32_t definitions
#include <string>
#include <vector>

#include <boost/shared_ptr.hpp>

#include "visensor/visensor_config.hpp"
#include "visensor/visensor_constants.hpp"

namespace visensor {

  #define BRISK_DESCRIPTOR_SIZE 48

  typedef enum {
    NOT_SET = 0,
    MONO8 = 2,
    COLOR = 4,
    MONO16 = 5
  } ViImageType;

  struct DSO_EXPORT ViConfigMsg {
    SensorId::SensorId sensorId;    ///< id of the sensor
    uint8_t devAdress;              ///< i2c address of the device
    uint32_t reg;                   ///< register to change
    uint32_t val;                   ///< new register value
    uint8_t comType;                ///< type of communication to the sensor
    bool valChanged;                ///< true, if value changed

    typedef boost::shared_ptr<ViConfigMsg> Ptr;
  };

  class DSO_EXPORT ViConfigParam {

   public:
    ViConfigParam()
        : _register(0),
          _default(0),
          _min(0),
          _max(0),
          _scaling(0),
          _value(0) {};

    ViConfigParam(uint8_t reg, uint16_t def, uint16_t min, uint16_t max,
                      float scaling)
        : _register(reg),
          _default(def),
          _min(min),
          _max(max),
          _scaling(scaling),
          _value(0) {};

   public:
    uint8_t _register;
    uint16_t _default;
    uint16_t _min;
    uint16_t _max;
    float _scaling;
    float _value;

    typedef boost::shared_ptr<ViConfigParam> Ptr;
  };

  struct DSO_EXPORT ViImuConfig {
    uint32_t rate;            ///< the imu data rate

    typedef boost::shared_ptr<ViImuConfig> Ptr;
  };

  struct DSO_EXPORT ViExternalTriggerConfig {
    enum direction_e {
      TRIGGER_OUTPUT = 0,
      TRIGGER_INPUT = 1
    };

    enum polarity_e {
      TRIGGER_ACTIVE_HIGH = 0,
      TRIGGER_ACTIVE_LOW = 1
    };

    direction_e direction;
    polarity_e polarity;

    unsigned int pulse_ms; //ms

    typedef boost::shared_ptr<ViExternalTriggerConfig> Ptr;
  };

  struct DSO_EXPORT ViImuMsg {
    double gyro[3];           ///< 3D Gyro values (rad/s)
    double acc[3];            ///< 3D acceleration values (m^2/s)
    double mag[3];            ///< 3D magnetometer values (gauss)
    double baro;              ///< barometer value (hPa)
    double temperature;       ///< temperature value
    uint64_t timestamp;       ///< FPGA time of imu message in nano seconds and corrected with an initial offset
    uint64_t timestamp_synchronized; ///< FPGA time on sensor in nano seconds and corrected with a internal time synchronization model
    uint32_t timestamp_fpga_counter; ///< raw fpga counter timestamp (1e-5 s)
    uint64_t timestamp_host;  ///< time when imu message was received on host*/
    uint32_t imu_id;          ///< id of the imu

    typedef boost::shared_ptr<ViImuMsg> Ptr;
  };

  struct DSO_EXPORT ViExternalTriggerMsg {
    uint64_t timestamp;              ///< FPGA time of trigger message in nano seconds and corrected with an initial offset
    uint64_t timestamp_synchronized; ///< FPGA time of trigger message on sensor in nano seconds and corrected with a internal time synchronization model
    uint32_t timestamp_fpga_counter; ///< raw fpga counter timestamp (1e-5 s)
    uint64_t timestamp_host;         ///< time when trigger message was received on host
    uint32_t trigger_id;             ///< id of the trigger core
    uint32_t event_id;               ///< id of the trigger event

    typedef boost::shared_ptr<ViExternalTriggerMsg> Ptr;
  };

  struct DSO_EXPORT ViCornerElem {
    ViCornerElem()
        : x(0),
          y(0),
          score(0),
          descriptor_size(BRISK_DESCRIPTOR_SIZE) {
    };

    uint16_t x;
    uint16_t y;
    uint32_t score;
    uint8_t descriptor_size;
    uint8_t descriptor[BRISK_DESCRIPTOR_SIZE];

    typedef boost::shared_ptr<ViCornerElem> Ptr;
  };

  struct DSO_EXPORT ViCorner {
    std::vector<ViCornerElem> corners;
    uint64_t timestamp; 		///< FPGA time when the image was captured in nano seconds and corrected with an initial offset
    uint64_t timestamp_synchronized; 	///< FPGA time when the image was captured on sensor in nano seconds and corrected with a internal time synchronization model
    uint32_t timestamp_fpga_counter; 	///< raw fpga counter timestamp (1e-5 s)
    uint64_t timestamp_host; 		///< time when data was received on host
    uint32_t camera_id;                 ///< the id of the camera

    typedef boost::shared_ptr<ViCorner> Ptr;
  };

  /**
   * \class ViFrame
   *
   *
   */
  class DSO_EXPORT ViFrame {
   public:
    ViFrame();
    ~ViFrame();

    int getBufferSize();
    void setImageRawPtr(boost::shared_ptr<uint8_t> buffer, uint32_t buffer_size);
    uint8_t* getImageRawPtr();

    uint32_t width;               ///< the image width
    uint32_t height;              ///< the image height
    uint64_t timestamp; 	  ///< FPGA time when the image was captured in nano seconds and corrected with an initial offset
    uint64_t timestamp_synchronized; ///< FPGA time when the image was captured on sensor in nano seconds and corrected with a internal time synchronization model
    uint32_t timestamp_fpga_counter; ///< raw fpga counter timestamp (1e-5 s)
    uint64_t timestamp_host; 	  ///< time when the image was received on host
    uint32_t id;                  ///< the frame id
    uint32_t allocated_image_bytes; ///< amount of memory allocated for the *image field.
    uint32_t camera_id;           ///< the id of the camera
    ViImageType image_type;       ///< type of the image
    bool useCorners;              ///<
    ViCorner::Ptr corner;

    typedef boost::shared_ptr<ViFrame> Ptr;

   private:
    boost::shared_ptr<uint8_t> image; ///< the image
  };

  class ViCameraProjectionModel
  {
   public:
    typedef std::shared_ptr<ViCameraProjectionModel> Ptr;
    enum class ProjectionModelTypes
    {
      UNKNOWN,
      PINHOLE,
      OMNIDIRECTIONAL
    };

    ViCameraProjectionModel(const ProjectionModelTypes type, const std::string type_name)
        : type_(type),
          type_name_(type_name)
    {
    }

    virtual ~ViCameraProjectionModel()
    {
    }

    virtual std::vector<double> getCoefficients()
    {
      std::vector<double> coefficients;
      return coefficients;
    }

    ProjectionModelTypes type_;
    std::string type_name_;
  };

  class ViCameraProjectionModelPinhole : public ViCameraProjectionModel
  {
   public:
    typedef std::shared_ptr<ViCameraProjectionModelPinhole> Ptr;
    enum class PinholeCoefficients
    {
      FU,
      FV,
      PU,
      PV
    };
    ViCameraProjectionModelPinhole()
        : ViCameraProjectionModel(ProjectionModelTypes::PINHOLE, "pinhole"),
          focal_length_u_(0.0),
          focal_length_v_(0.0),
          principal_point_u_(0.0),
          principal_point_v_(0.0)
    {
    }

    ViCameraProjectionModelPinhole(double fu, double fv, double pu, double pv)
        : ViCameraProjectionModel(ProjectionModelTypes::PINHOLE, "pinhole"),
          focal_length_u_(fu),
          focal_length_v_(fv),
          principal_point_u_(pu),
          principal_point_v_(pv)
    {
    }

    std::vector<double> getCoefficients()
    {
      std::vector<double> coefficients;
      coefficients.push_back(focal_length_u_);
      coefficients.push_back(focal_length_v_);
      coefficients.push_back(principal_point_u_);
      coefficients.push_back(principal_point_v_);
      return coefficients;
    }

    double focal_length_u_;
    double focal_length_v_;
    double principal_point_u_;
    double principal_point_v_;
  };

  class ViCameraProjectionModelOmnidirectional : public ViCameraProjectionModel
  {
   public:
    typedef std::shared_ptr<ViCameraProjectionModelOmnidirectional> Ptr;
    enum class OmnidirectionalCoefficients
    {
      FU,
      FV,
      PU,
      PV,
      XI
    };

    ViCameraProjectionModelOmnidirectional()
        : ViCameraProjectionModel(ProjectionModelTypes::OMNIDIRECTIONAL, "omnidirectional"),
          focal_length_u_(0.0),
          focal_length_v_(0.0),
          principal_point_u_(0.0),
          principal_point_v_(0.0),
          mirror_xi_(0.0)
    {
    }

    ViCameraProjectionModelOmnidirectional(double fu, double fv, double pu, double pv, double xi)
        : ViCameraProjectionModel(ProjectionModelTypes::OMNIDIRECTIONAL, "omnidirectional"),
          focal_length_u_(fu),
          focal_length_v_(fv),
          principal_point_u_(pu),
          principal_point_v_(pv),
          mirror_xi_(xi)
    {
    }

    std::vector<double> getCoefficients()
    {
      std::vector<double> coefficients;
      coefficients.push_back(mirror_xi_);
      coefficients.push_back(focal_length_u_);
      coefficients.push_back(focal_length_v_);
      coefficients.push_back(principal_point_u_);
      coefficients.push_back(principal_point_v_);
      return coefficients;
    }

    double focal_length_u_;
    double focal_length_v_;
    double principal_point_u_;
    double principal_point_v_;
    double mirror_xi_;
  };

  class ViCameraLensModel
  {
   public:
    typedef std::shared_ptr<ViCameraLensModel> Ptr;
    enum class LensModelTypes
    {
      UNKNOWN,
      RADTAN,
      EQUIDISTANT
    };

    ViCameraLensModel(LensModelTypes type, const std::string type_name)
        : type_(type),
          type_name_(type_name)
    {
    }

    virtual ~ViCameraLensModel()
    {
    }

    virtual std::vector<double> getCoefficients()
    {
      std::vector<double> coefficients;
      return coefficients;
    }

    LensModelTypes type_;
    std::string type_name_;
  };

  class ViCameraLensModelEquidistant : public ViCameraLensModel
  {
   public:
    typedef std::shared_ptr<ViCameraLensModelEquidistant> Ptr;
    enum class EquidistantsCoefficients
    {
      K1,
      K2,
      K3,
      K4
    };

    ViCameraLensModelEquidistant()
        : ViCameraLensModel(LensModelTypes::EQUIDISTANT, "equidistant"),
          k1_(0.0),
          k2_(0.0),
          k3_(0.0),
          k4_(0.0)
    {
    }

    ViCameraLensModelEquidistant(double k1, double k2, double k3, double k4)
        : ViCameraLensModel(LensModelTypes::EQUIDISTANT, "equidistant"),
          k1_(k1),
          k2_(k2),
          k3_(k3),
          k4_(k4)
    {
    }

    std::vector<double> getCoefficients()
    {
      std::vector<double> coefficients;
      coefficients.push_back(k1_);
      coefficients.push_back(k2_);
      coefficients.push_back(k3_);
      coefficients.push_back(k4_);
      return coefficients;
    }

    double k1_;
    double k2_;
    double k3_;
    double k4_;
  };

  class ViCameraLensModelRadtan : public ViCameraLensModel
  {
   public:
    typedef std::shared_ptr<ViCameraLensModelRadtan> Ptr;
    enum class RadtanCoefficients
    {
      K1,
      K2,
      R1,
      R2
    };

    ViCameraLensModelRadtan()
        : ViCameraLensModel(LensModelTypes::RADTAN, "radtan"),
          k1_(0.0),
          k2_(0.0),
          r1_(0.0),
          r2_(0.0)
    {
    }

    ViCameraLensModelRadtan(double k1, double k2, double r1, double r2)
        : ViCameraLensModel(LensModelTypes::RADTAN, "radtan"),
          k1_(k1),
          k2_(k2),
          r1_(r1),
          r2_(r2)
    {
    }

    std::vector<double> getCoefficients()
    {
      std::vector<double> coefficients;
      coefficients.push_back(k1_);
      coefficients.push_back(k2_);
      coefficients.push_back(r1_);
      coefficients.push_back(r2_);
      return coefficients;
    }

    double k1_;
    double k2_;
    double r1_;
    double r2_;
  };

  class ViCameraCalibration
  {
   public:
    static const unsigned int NUMBER_OF_R = 9;
    static const unsigned int NUMBER_OF_T = 3;
    static const unsigned int NUMBER_OF_RESOLUTION = 2;

    ViCameraCalibration()
        : cam_id_(0),
          slot_id_(0),
          is_flipped_(false),
          resolution_ { 0, 0 }
    {
    }

    ViCameraCalibration(ViCameraLensModel::LensModelTypes type_lens_model,
                        ViCameraProjectionModel::ProjectionModelTypes type_projection_model)
        : cam_id_(0),
          slot_id_(0),
          is_flipped_(false),
          resolution_ { 0, 0 }
    {
      switch (type_lens_model) {
        case ViCameraLensModel::LensModelTypes::EQUIDISTANT:
          lens_model_ = std::make_shared<ViCameraLensModelEquidistant>();
          break;
        case ViCameraLensModel::LensModelTypes::RADTAN:
          lens_model_ = std::make_shared<ViCameraLensModelRadtan>();
          break;
        default:
          lens_model_ = std::make_shared<ViCameraLensModelRadtan>();
      }
      switch (type_projection_model) {
        case ViCameraProjectionModel::ProjectionModelTypes::OMNIDIRECTIONAL:
          projection_model_ =std::make_shared<ViCameraProjectionModelOmnidirectional>();
          break;
        case ViCameraProjectionModel::ProjectionModelTypes::PINHOLE:
          projection_model_ = std::make_shared<ViCameraProjectionModelPinhole>();
          break;
        default:
          projection_model_ = std::make_shared<ViCameraProjectionModelPinhole>();
      }
    }

    ViCameraCalibration(int cam_id, int slot_id, int resolution_width, int resolution_high,
                        bool is_flipped, ViCameraLensModel::LensModelTypes type_lens_model,
                        ViCameraProjectionModel::ProjectionModelTypes type_projection_model,
                        double k1, double k2, double k3, double k4, double fu, double fv, double pu,
                        double pv, double xi = 0.0)
        : cam_id_(cam_id),
          slot_id_(slot_id),
          is_flipped_(is_flipped),
          resolution_ { resolution_width, resolution_high }
    {
      switch (type_lens_model) {
        case ViCameraLensModel::LensModelTypes::EQUIDISTANT:
          lens_model_ = std::make_shared<ViCameraLensModelEquidistant>(k1, k2, k3, k4);
          break;
        case ViCameraLensModel::LensModelTypes::RADTAN:
          lens_model_ = std::make_shared<ViCameraLensModelRadtan>(k1, k2, k3, k4);
          break;
        default:
          lens_model_ = std::make_shared<ViCameraLensModelRadtan>(k1, k2, k3, k4);
      }
      switch (type_projection_model) {
        case ViCameraProjectionModel::ProjectionModelTypes::OMNIDIRECTIONAL:
          projection_model_ = std::make_shared<ViCameraProjectionModelOmnidirectional>(fu, fv, pu, pv, xi);
          break;
        case ViCameraProjectionModel::ProjectionModelTypes::PINHOLE:
          projection_model_ = std::make_shared<ViCameraProjectionModelPinhole>(fu, fv, pu, pv);
          break;
        default:
          projection_model_ = std::make_shared<ViCameraProjectionModelPinhole>(fu, fv, pu, pv);
      }
    }

    ViCameraCalibration& operator=(const ViCameraCalibration& obj)
    {
      R_ = obj.R_;
      t_ = obj.t_;
      is_flipped_ = obj.is_flipped_;
      resolution_[0] = obj.resolution_[0];
      resolution_[1] = obj.resolution_[1];

    std::vector<double> tmp;
    switch (obj.lens_model_->type_) {
      case ViCameraLensModel::LensModelTypes::EQUIDISTANT:
        tmp = std::static_pointer_cast<ViCameraLensModelEquidistant>(obj.lens_model_)
              ->getCoefficients();
        lens_model_ = std::static_pointer_cast<ViCameraLensModel>(
            std::make_shared<ViCameraLensModelEquidistant>(tmp.at(0), tmp.at(1), tmp.at(2),
                                                           tmp.at(3)));
        break;
      case ViCameraLensModel::LensModelTypes::RADTAN:
        tmp = std::static_pointer_cast<ViCameraLensModelRadtan>(obj.lens_model_)->getCoefficients();
        lens_model_ = std::static_pointer_cast<ViCameraLensModel>(
            std::make_shared<ViCameraLensModelRadtan>(tmp.at(0), tmp.at(1), tmp.at(2), tmp.at(3)));
        break;
      default:
        tmp = std::static_pointer_cast<ViCameraLensModelRadtan>(obj.lens_model_)->getCoefficients();
        lens_model_ = std::static_pointer_cast<ViCameraLensModel>(
            std::make_shared<ViCameraLensModelRadtan>(tmp.at(0), tmp.at(1), tmp.at(2), tmp.at(3)));
    }
    switch (obj.projection_model_->type_) {
      case ViCameraProjectionModel::ProjectionModelTypes::OMNIDIRECTIONAL:
        tmp = std::static_pointer_cast<ViCameraProjectionModelOmnidirectional>(obj.projection_model_)
              ->getCoefficients();
        projection_model_ = std::static_pointer_cast<ViCameraProjectionModel>(
            std::make_shared<ViCameraProjectionModelOmnidirectional>(tmp.at(1), tmp.at(2),
                                                                     tmp.at(3), tmp.at(4),
                                                                     tmp.at(0)));
        break;
      case ViCameraProjectionModel::ProjectionModelTypes::PINHOLE:
        tmp = std::static_pointer_cast<ViCameraProjectionModelPinhole>(obj.projection_model_)
              ->getCoefficients();
        projection_model_ = std::static_pointer_cast<ViCameraProjectionModel>(
            std::make_shared<ViCameraProjectionModelPinhole>(tmp.at(0), tmp.at(1), tmp.at(2),
                                                             tmp.at(3)));
        break;
      default:
        tmp = std::static_pointer_cast<ViCameraProjectionModelPinhole>(obj.projection_model_)
              ->getCoefficients();
        projection_model_ = std::static_pointer_cast<ViCameraProjectionModel>(
            std::make_shared<ViCameraProjectionModelPinhole>(tmp.at(0), tmp.at(1), tmp.at(2),
                                                             tmp.at(3)));
    }

      slot_id_ = obj.slot_id_;
      cam_id_ = obj.cam_id_;
      return *this;
    }


    template<class TYPE>
    std::shared_ptr<TYPE> getLensModel() const
    {
      return std::static_pointer_cast<TYPE>(lens_model_);
    }
    template<class TYPE>
    std::shared_ptr<TYPE> getProjectionModel() const
    {
      return std::static_pointer_cast<TYPE>(projection_model_);
    }

    int cam_id_;
    int slot_id_;
    bool is_flipped_;

    //extrinsics
    std::vector<double> R_;
    std::vector<double> t_;
    int resolution_[2];    ///< [width,height]

    //intrinsics
    ViCameraLensModel::Ptr lens_model_;
    ViCameraProjectionModel::Ptr projection_model_;
  };


  struct DSO_EXPORT ViSerialData {
    enum direction_e {
      FROM_SERIAL_DEVICE,
      TO_SERIAL_DEVICE
    };

    unsigned int port_id;
    direction_e direction;
    std::string data;

    typedef boost::shared_ptr<ViSerialData> Ptr;
  };
  typedef std::vector<std::string> ViDeviceList;

  /**
   * ViSensor error codes
   */
  enum class ViErrorCodes{
    NO_ERROR,
    MEASUREMENT_DROPPED
  };
  typedef const ViErrorCodes& ViErrorCode;

}  //namespace visensor

#endif /* VISENSOR_DATATYPES_HPP_ */
