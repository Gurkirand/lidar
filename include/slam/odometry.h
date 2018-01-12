#ifndef SLAM_ODOMETRY_H
#define SLAM_ODOMETRY_H

#include <vector>

#include "common.h"
#include "math_utils.h"
#include "types/Vector3.h"
#include "types/Transform.h"
#include "types/Quaternion.h"
#include <Eigen/Eigen>
#include <pcl/kdtree/kdtree_flann.h>

namespace slam
{


class odometry
{
	
private:

	using kdtree_type = pcl::KdTreeFLANN<point_type>;
	using kdtree_ptr = kdtree_type::Ptr;

	/* float transform[6] = {0}; */
	/* float transformSum[6] = {0}; */

	Transform transform;

	std::vector<int> search_ind;
	std::vector<float> search_sqr_diff;

	int nearest_edge_ind1[192];
	int nearest_edge_ind2[192];

	int nearest_surface_ind1[384];
	int nearest_surface_ind2[384];
	int nearest_surface_ind3[384];

	int iteration = 0;

	Eigen::Matrix<float, 6, 6> matP;
	bool isDegenerate = false;

	kdtree_ptr edge_search_tree;
	kdtree_ptr surface_search_tree;

	cloud_ptr coeff_sel;
	cloud_ptr cloud_ori;

	CloudScans* source_scans;
	ScanFeatures* source_features;
	ScanFeatures* map_features;
	ScanFeatures const* target_features;

	void initial_relative_transform (point_type const& pi,
												float relPercent,
												point_type& po);
	void final_relative_transform (CloudScans& cloudScans);
	void edge_point_correspondences (point_type const& pointSel, size_t ind);
	void edge_point_transforms ();
	void surface_point_correspondences (point_type const& pointSel, size_t ind);
	void surface_point_transforms ();
	void stack_jacobian ( Eigen::Matrix<float,Eigen::Dynamic,6>& matA,
			Eigen::VectorXf& matB);
	bool pose_transform ();
	Rotation accumulate_rotation(
			Angle const& rot1x, Angle const& rot1y, Angle const& rot1z,
			Angle const& rot2x, Angle const& rot2y, Angle const& rot2z);
	void update_odometry ();

	void init ();


public:

	Transform transformSum;
	Quaternion geoQuat;

	odometry ();

	void process (CloudScans& source,
				ScanFeatures& source_feats,
				ScanFeatures& map_feats,
				ScanFeatures const& target_feats);

}; // odometry

} // slam


#endif // SLAM_ODOMETRY_H
