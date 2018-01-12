#ifndef SLAM_MAPPING_H
#define SLAM_MAPPING_H

#include "common.h"
#include "types/Transform.h"
#include "types/Quaternion.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <array>
#include <Eigen/Eigen>

namespace slam
{

class mapping
{

private:

	using kdtree_type = pcl::KdTreeFLANN<point_type>;
	using kdtree_ptr = kdtree_type::Ptr;

	static const int stackFrameNum = 1;
	static const int mapFrameNum = 5;

	int frameCount = stackFrameNum - 1;
	int mapFrameCount = mapFrameNum - 1;

	static const int laserCloudWidth = 21;
	static const int laserCloudHeight = 11;
	static const int laserCloudDepth = 21;
	static const int laserCloudNum = laserCloudWidth * laserCloudHeight * laserCloudDepth;
	int laserCloudCenWidth = 10;
	int laserCloudCenHeight = 5;
	int laserCloudCenDepth = 10;

	Transform transformSum;
	Transform transformIncre;
	Transform transformTobeMapped;
	Transform transformBefMapped;

	int laserCloudValidInd[125];
	int laserCloudSurroundInd[125];

	int laserCloudValidNum;
	int laserCloudSurroundNum;

	std::vector<int> pointSearchInd;
	std::vector<float> pointSearchSqDis;


	// replace with Feature data type


	ScanFeatures const* source_features;
	/* cloud_ptr laserCloudCornerLast; */ //Last is actually most recent scan.
	/* cloud_ptr laserCloudSurfLast; */

	CloudScans* source_scans;
	/* cloud_ptr laserCloudFullRes; */

	/* CloudFeatures features_stack; */
	/* CloudFeatures features_stackDS; */

	cloud_ptr laserCloudCornerStack;
	cloud_ptr laserCloudSurfStack;
	cloud_ptr laserCloudCornerStack2;
	cloud_ptr laserCloudSurfStack2;

	/* cloud_ptr cloud_ori; */
	/* cloud_ptr coeff_sel; */
	cloud_ptr laserCloudOri;
	cloud_ptr coeffSel;

	/* cloud_ptr cloud_surround; */
	/* cloud_ptr cloud_surroundDS; */
	cloud_ptr laserCloudSurround;
	cloud_ptr laserCloudSurround2;

	/* CloudFeatures map_features; */
	cloud_ptr laserCloudCornerFromMap;
	cloud_ptr laserCloudSurfFromMap;

	//std::array please


	/* std::array<CloudFeatures, laserCloudNum> segment_array; */
	cloud_ptr laserCloudCornerArray[laserCloudNum];
	cloud_ptr laserCloudSurfArray[laserCloudNum];

	/* std::array<CloudFeatures, laserCloudNum> segment_arrayDS; */
	cloud_ptr laserCloudCornerArray2[laserCloudNum];
	cloud_ptr laserCloudSurfArray2[laserCloudNum];

	kdtree_ptr kdtreeCornerFromMap;
	kdtree_ptr kdtreeSurfFromMap;

	pcl::VoxelGrid<point_type> downSizeFilterCorner;
	pcl::VoxelGrid<point_type> downSizeFilterSurf;
	pcl::VoxelGrid<point_type> downSizeFilterMap;
	

	int iterCount = 0;

	/* void update_transform(); */

	void transform_associate_to_map();
	void point_associate_to_map(point_type const& pi, point_type& po);
	void point_associate_to_be_mapped(point_type const& pi, point_type& po);

	typedef std::function<int(int, int, int)> index_func;

	int segmented_cloud_index (int width, int height, int depth) const;
	index_func const segmented_cloud_index_WHD = [this](int w, int h, int d) { return this->segmented_cloud_index(w, h, d); };
	index_func const segmented_cloud_index_HWD = [this](int h, int w, int d) { return this->segmented_cloud_index(w, h, d); };
	index_func const segmented_cloud_index_DWH = [this](int d, int w, int h) { return this->segmented_cloud_index(w, h, d); };
	/* index_calc segmented_cloud_index_WHD = std::bind(&slam::mapping::segmented_cloud_index, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3) */
	/* index_calc segmented_cloud_index_HWD = std::bind(&slam::mapping::segmented_cloud_index, this, std::placeholders::_2, std::placeholders::_1, std::placeholders::_3) */
	/* index_calc segmented_cloud_index_DWH = std::bind(&slam::mapping::segmented_cloud_index, this, std::placeholders::_3, std::placeholders::_2, std::placeholders::_1) */
	/* index_calc segmented_cloud_index_HWD (int height, int width, int depth); */
	/* index_calc segmented_cloud_index_DWH (int depth, int width, int height); */
	void segment_cloud_along_axis (
			int ref_len,
			int len2,
			int len3,
			int& center,
			int& ref_center_len,
			index_func const& index_calc);
	void find_valid_segments (int centerCubeW, int centerCubeH, int centerCubeD);
	void segment_cloud ();

	void stack_features();
	void downsample_last ();

	void edge_point_transforms ();
	void surface_point_transforms ();
	void stack_jacobian (
			Eigen::Matrix<float, Eigen::Dynamic, 6>& matA,
			Eigen::VectorXf& matB);
	bool pose_transform();
	void find_transform ();

	void store_downsamples ();

	void publish ();

	void init();


public:

	Transform transformAftMapped;

	Quaternion geoQuat;

	mapping();
	void process (
			CloudScans& sourceCloud,
			ScanFeatures const& sourceFeats,
			Transform& odometryTransform,
			Quaternion& odometryQuat);

}; // mapping

} // slam




#endif // SLAM_MAPPING_H
