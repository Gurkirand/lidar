#include <iostream>

#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>

#include "../../include/slam/feature_extraction.h"
#include "../time_lapse.h"

void test_feature_extraction(slam::feature_extraction&, slam::cloud_ptr);

int main(int argc, char* argv[])
{
	std::string pcd = "data/scan_cloud0.pcd";
	slam::cloud_ptr cloud(new slam::cloud_type());
	pcl::io::loadPCDFile<slam::point_type>(pcd, *cloud);

	slam::CloudScans scan(cloud, 
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
			cloud->points.size()
		 }});
			

	slam::feature_extraction features;
	features.process(scan);
	std::cout << "\nFinished Extraction\n\n";
	std::cout << "Odometry:\n";
	std::cout << "\tEdge - " << features.odometry_features.edge_scans.cloud->points.size() << " points\n";
	std::cout << "\tSurface - " << features.odometry_features.surface_scans.cloud->points.size() << " points\n";
	std::cout << "\nMapping:\n";
	std::cout << "\tEdge - " << features.map_features.edge_scans.cloud->points.size() << " points\n";
	std::cout << "\tSurface - " << features.map_features.surface_scans.cloud->points.size() << " points\n" << std::endl;

	return 0;
}

/* void test_feature_extraction(slam::vlp_features& features, slam::cloud_type& cloud) */
/* { */
/* 	features.extract_features(cloud); */
/* } */


