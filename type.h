#pragma once
#include <stdint.h>
#include <vector>

namespace pcl {
using uindex_t = uint32_t;
using index_t = int32_t;

using Indices = std::vector<int32_t>;

template<typename PointT>
using PointCloud = std::vector<PointT>;
}
