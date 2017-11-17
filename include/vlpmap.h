#ifndef VLP_MAP_H
#define VLP_MAP_H

#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>

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
	using normal_type = pcl::PointNormal;
	using grabber_type = pcl::VLPGrabber;
	using cloud_type = pcl::PointCloud<point_type>;
	using cloud_normal_type = pcl::PointCloud<normal_type>;
	using cloud_normal_pointer = pcl::PointCloud<normal_type>::Ptr;
	using cloud_pointer = cloud_type::Ptr;
	using cloud_const_pointer = cloud_type::ConstPtr;
	using visualizer_type = pcl::visualization::PCLVisualizer;
	using handler_type = pcl::visualization::PointCloudColorHandlerGenericField<point_type>;
	using visualizer_pointer = boost::shared_ptr<visualizer_type>;
	using handler_pointer = boost::shared_ptr<handler_type>;

	boost::shared_ptr<grabber_type> grabber;
	boost::signals2::connection cloud_connection;
	boost::mutex cloud_mutex;
	std::vector<cloud_pointer> cloud_stream;
	cloud_pointer _cloud;
	cloud_pointer previous;
	cloud_pointer map;
	cloud_const_pointer const_map;
	visualizer_pointer view;
	handler_pointer view_handler;
	int transforms = 0;

	void cloud_callback(const cloud_const_pointer& cloud);
	void register_cloud(cloud_pointer target);


public:

	vlpmap(const std::string& pcap);
	~vlpmap();

	void run();
	void step_run();

};

} //velodyne

#endif // VLP_MAP_H
