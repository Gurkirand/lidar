#include <pcl/io/pcd_io.h>
#include "../../include/slam/mapping.h"
#include "../../include/slam/odometry.h"
#include "../../include/slam/feature_extraction.h"

int main(int argc, char* argv[])
{
	std::string pcdLast = "data/scan_cloud0.pcd";
	std::string pcdNew = "data/scan_cloud1.pcd";
	slam::cloud_ptr cloudLast(new slam::cloud_type());
	slam::cloud_ptr cloudNew(new slam::cloud_type());
	pcl::io::loadPCDFile<slam::point_type>(pcdLast, *cloudLast);
	pcl::io::loadPCDFile<slam::point_type>(pcdNew, *cloudNew);

	slam::CloudScans scanLast(cloudLast, 
		{{
			0,
			758,
			2192,
			3341,
			4784,
			6347,
			7821,
			9582,
			11057,
			12770,
			14141,
			15819,
			17061,
			18678,
			19791,
			21323,
			cloudLast->points.size()
		 }});

	slam::CloudScans scanNew(cloudNew, 
		{{
			0,
			755,
			2204,
			3367,
			4815,
			6382,
			7823,
			9584,
			11091,
			12790,
			14160,
			15847,
			17090,
			18694,
			19793,
			21298,
			cloudNew->points.size()
		 }});

	slam::feature_extraction features;
	slam::odometry odom;
	slam::mapping map;

	features.process(scanLast);
	slam::ScanFeatures odomFeatsLast = features.odometry_features;
	slam::ScanFeatures mapFeatsLast = features.map_features;
	
	features.process(scanNew);
	slam::ScanFeatures odomFeatsNew = features.odometry_features;
	slam::ScanFeatures mapFeatsNew = features.map_features;

	map.process(scanLast, mapFeatsLast, odom.transformSum, odom.geoQuat);

	odom.process(scanNew, odomFeatsNew, mapFeatsNew, mapFeatsLast);

	map.process(scanNew, mapFeatsNew, odom.transformSum, odom.geoQuat);

	std::cout << map.transformAftMapped.rotation.x.rad() << " "
		<< map.transformAftMapped.rotation.y.rad() << " "
		<< map.transformAftMapped.rotation.z.rad() << " "
		<< std::endl;

	return 0;
}
