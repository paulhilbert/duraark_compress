#ifndef _DURAARK_COMPRESS_COMMON_HPP_
#define _DURAARK_COMPRESS_COMMON_HPP_

#include <memory>
#include <vector>
#include <experimental/optional>
namespace ex = std::experimental;

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <e57_pcl/read.hpp>

namespace duraark_compress {

typedef pcl::PointXYZ                   point_xyz_t;
typedef pcl::PointNormal                point_normal_t;
typedef pcl::PointCloud<point_xyz_t>    cloud_xyz_t;
typedef pcl::PointCloud<point_normal_t> cloud_normal_t;

typedef Eigen::Vector2i                 vec2i_t;
typedef Eigen::Vector2f                 vec2f_t;
typedef Eigen::Vector3f                 vec3f_t;
typedef Eigen::Matrix3f                 base_t;
typedef Eigen::AlignedBox<float, 2>     bbox2f_t;
typedef Eigen::AlignedBox<float, 3>     bbox3f_t;

} // duraark_compress


#endif /* _DURAARK_COMPRESS_COMMON_HPP_ */
