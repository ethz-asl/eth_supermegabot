#ifndef RESOURCES_COMMON_POINT_CLOUD_H_
#define RESOURCES_COMMON_POINT_CLOUD_H_

#include <vector>

#include <Eigen/Core>
#include <aslam/common/pose-types.h>

namespace resources {

typedef Eigen::Matrix<uint8_t, 4, 1> RgbaColor;

struct PointCloud {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  std::vector<float> xyz;
  std::vector<float> normals;
  std::vector<unsigned char> colors;
  std::vector<float> scalars;

  // Apply transformation T_A_B to pointcloud, assuming the pointcloud is
  // currently expressed in the B frame.
  void applyTransformation(const aslam::Transformation& T_A_B) {
    for (size_t idx = 0u; idx < xyz.size(); idx += 3u) {
      const Eigen::Vector3d point(xyz[idx], xyz[idx + 1u], xyz[idx + 2u]);
      const Eigen::Vector3f& transformed_point = (T_A_B * point).cast<float>();
      xyz[idx] = transformed_point.x();
      xyz[idx + 1u] = transformed_point.y();
      xyz[idx + 2u] = transformed_point.z();
    }
  }

  void resize(
      const size_t size, const bool has_normals = true,
      const bool has_colors = true, const bool has_scalars = true) {
    xyz.resize(3 * size);

    if (has_normals) {
      normals.resize(3 * size);
    }

    if (has_colors) {
      colors.resize(3 * size);
    }

    if (has_scalars) {
      scalars.resize(1 * size);
    }
  }

  size_t size() const {
    CHECK_EQ(xyz.size() % 3, 0u);
    return (xyz.size() / 3);
  }

  bool empty() const {
    return xyz.empty();
  }

  bool checkConsistency() const {
    bool consistent = true;
    consistent &= (normals.size() == xyz.size()) || normals.empty();
    consistent &= (colors.size() == xyz.size()) || colors.empty();
    consistent &= (scalars.size() == xyz.size() / 3u) || scalars.empty();
    return consistent;
  }

  void append(const PointCloud& other) {
    if (other.empty()) {
      return;
    }

    xyz.reserve(xyz.size() + other.xyz.size());
    normals.reserve(normals.size() + other.normals.size());
    colors.reserve(colors.size() + other.colors.size());
    scalars.reserve(scalars.size() + other.scalars.size());

    xyz.insert(xyz.end(), other.xyz.begin(), other.xyz.end());
    normals.insert(normals.end(), other.normals.begin(), other.normals.end());
    colors.insert(colors.end(), other.colors.begin(), other.colors.end());
    scalars.insert(scalars.end(), other.scalars.begin(), other.scalars.end());

    CHECK(checkConsistency()) << "Point cloud is not consistent!";
  }

  bool operator==(const PointCloud& other) const {
    bool is_same = xyz == other.xyz;
    is_same &= normals == other.normals;
    is_same &= colors == other.colors;
    is_same &= scalars == other.scalars;
    return is_same;
  }
};

}  // namespace resources

#endif  // RESOURCES_COMMON_POINT_CLOUD_H_
