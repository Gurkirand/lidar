#ifndef VLP_SLAM_H
#define VLP_SLAM_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "common.h"
#include "features.h"

namespace slam
{

class vlp_slam
{

	std::vector<std::pair<CloudScans, ScanFeatures, ScanFeatures> > process_stack;


}


}

#endif // VLP_SLAM_H
