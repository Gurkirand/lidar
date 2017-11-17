#ifndef VLP_MAP_H
#define VLP_MAP_H

#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>

/*
 * TODO:
 *      - Iterative Registration to visualize
 *      - On VLPGrabber callback register new point cloud.
 *      - Keep a running map of all registrations concatenated.
 *      - Take odometry data and apply a rigid transform of where
 *       I expect the target cloud to be, before registration.
 *       
 */
namespace velodyne
{

class vlpmap
{

private:

	using point_type = pcl::PointXYZI;
	using grabber_type = pcl::VLPGrabber;
	using cloud_type = pcl::PointCloud<point_type>;
	using cloud_pointer = cloud_type::ConstPtr;
	
	boost::shared_ptr<grabber_type> grabber;
	boost::mutex cloud_mutex;
	cloud_pointer _cloud;
	std::vector<cloud_pointer> cloud_stream;
	boost::signals2::connection cloud_connection;
	cloud_type map;

	void cloud_callback(const cloud_pointer& cloud);


public:

	vlpmap(const std::string& pcap);
	~vlpmap();

	void run();
	void icp();

};

} //velodyne

#endif // VLP_MAP_H
