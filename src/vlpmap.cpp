#include <iostream>

#include "../include/vlpmap.h"

#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/registration/icp_nl.h>

namespace velodyne
{

vlpmap::vlpmap(const std::string& pcap)
	: map(new cloud_type)
{
	grabber = boost::shared_ptr<grabber_type>( new grabber_type(pcap));
	boost::function<void (const cloud_const_pointer&)> cloud_cb = boost::bind(&vlpmap::cloud_callback, this, _1);
	cloud_connection = grabber->registerCallback(cloud_cb);

	/* map(new cloud_type); */

	/* view(new visualizer_type("VLP Map")); */
	/* view_handler(new handler_type("intensity")); */
	view = visualizer_pointer(new visualizer_type("VLP Map"));

	view_handler = handler_pointer(new handler_type("intensity"));

    view->addCoordinateSystem( 3.0, "coordinate" );
    view->setBackgroundColor(0.0, 0.0, 0.0, 0);
    view->initCameraParameters();
    view->setCameraPosition( 0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0 );
}

vlpmap::~vlpmap()
{
	/* delete grabber; */
}

void vlpmap::cloud_callback(const cloud_const_pointer& cloud)
{
	boost::mutex::scoped_lock lock(cloud_mutex);
	_cloud = cloud_pointer(new cloud_type(*cloud));
}

void vlpmap::run()
{
	grabber->start();

	while (grabber->isRunning())
	{
		cloud_pointer cloud;

		if (cloud_mutex.try_lock())
		{
			_cloud.swap(cloud);
			if (cloud)
			{
				register_cloud(cloud);
			}
			cloud_mutex.unlock();
		}

	}

	grabber->stop();

	if (cloud_connection.connected())
	{
		cloud_connection.disconnect();
	}

	std::cout << "Finished Mapping" << std::endl;
	view->spin();
}

void vlpmap::step_run()
{
	std::cout << "Reading PCAP." << std::endl;

	grabber->start();

	while (grabber->isRunning())
	{
		cloud_pointer cloud;

		if (cloud_mutex.try_lock())
		{
			cloud.swap(_cloud);
			cloud_mutex.unlock();
		}

		if (cloud)
		{
			cloud_stream.push_back(cloud);
		}
	}

	grabber->stop();

	if (cloud_connection.connected())
	{
		cloud_connection.disconnect();
	}

	std::cout << "Cloud stream size: " << cloud_stream.size() << std::endl;

	char input = 's';
	auto itr = cloud_stream.begin(),
	     end = cloud_stream.end();

	std::cout << "Enter s to step or q to quit." << std::endl;
	for (auto itr = cloud_stream.begin(), end = cloud_stream.end();
			!view->wasStopped() &&  itr != end; itr++)
	{
		std::cout << "> ";
		std::cin >> input;
		if (input == 'q')
		{
			break;
		}

		register_cloud(*itr);
	}

	view->spin();
}

void vlpmap::register_cloud(cloud_pointer target)
{
	if (map->empty())
	{
		std::cout << "EMPTY MAP" << std::endl;
		map = target;
		previous = target;
		return;
	}

	Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();
	translate(1,3) = 0.5 * transforms;
	transforms++;

	pcl::transformPointCloud(*target, *target, translate);
	
	cloud_normal_type::Ptr previous_normals(new cloud_normal_type);
	cloud_normal_type::Ptr target_normals(new cloud_normal_type);

	pcl::NormalEstimation<point_type, normal_type> estimator;
	pcl::search::KdTree<point_type>::Ptr tree (new pcl::search::KdTree<point_type> ());
	estimator.setSearchMethod (tree);
	estimator.setKSearch (30);

	estimator.setInputCloud(previous);
	estimator.compute(*previous_normals);
	pcl::copyPointCloud(*previous, *previous_normals);

	estimator.setInputCloud(target);
	estimator.compute(*target_normals);
	pcl::copyPointCloud(*target, *target_normals);

	/* pcl::IterativeClosestPoint<point_type, point_type> icp; */
	pcl::IterativeClosestPoint<normal_type, normal_type> icp;
	icp.setMaxCorrespondenceDistance (0.05);
	icp.setMaximumIterations (100);
	icp.setTransformationEpsilon (1e-8);
	icp.setEuclideanFitnessEpsilon (1);


	Eigen::Matrix4f t;
	cloud_normal_pointer f_normals(new cloud_normal_type);
	cloud_pointer f(new cloud_type);
	/* icp.setInputSource(previous); */
	/* icp.setInputTarget(target); */
	icp.setInputSource(previous_normals);
	icp.setInputTarget(target_normals);

	/* icp.align(*f); */
	icp.align(*f_normals);
	t = icp.getFinalTransformation();
	pcl::transformPointCloud(*target, *f, t.inverse());
	/* pcl::transformPointCloud(*target, *f, t); */

	*map += *f;

	previous = target;

	/* cloud_type map_copy; */
	/* pcl::copyPointCloud(*map, map_copy); */
	/* cloud_type const& const_map_copy = map_copy; */
	const_map = cloud_const_pointer(new cloud_type(*map));

	view->removePointCloud("map");
	view_handler->setInputCloud(const_map);
	view->addPointCloud(const_map, *view_handler, "map");
	/* view->spinOnce(3000, true); */
	view->spinOnce();
	
}

} //velodyne
