#include "kdtree/kdtree_flann.h"
#include <Eigen/Core>
using V2F = Eigen::Vector2f;
using V3F = Eigen::Vector3f;
int main() {
	pcl::PointCloud<V3F> point_cloud_3d;
	pcl::PointCloud<V2F> point_cloud_2d;
	for (int i = 0; i < 30; ++i) {
		for (int j = 0; j < 30; ++j) {
			for (int k = 0; k < 30; ++k) {
				point_cloud_3d.emplace_back(i, j, k);
			}
			point_cloud_2d.emplace_back(i, j);
		}
	}


	pcl::KdTreeFLANN<V3F> kd_tree_3d;
	kd_tree_3d.setInputCloud(std::make_shared<pcl::PointCloud<V3F>>(point_cloud_3d));
	pcl::Indices indices;
	std::vector<float> sqr_dists;
	kd_tree_3d.nearestKSearch(V3F(0, 0, 0), 1, indices, sqr_dists);

	pcl::KdTreeFLANN<V2F> kd_tree_2d;
	kd_tree_2d.setInputCloud(std::make_shared<pcl::PointCloud<V2F>>(point_cloud_2d));
	kd_tree_2d.nearestKSearch(V2F(0, 0, 0), 1, indices, sqr_dists);
	return -1;
}