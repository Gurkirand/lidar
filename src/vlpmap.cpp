#include <iostream>

#include "../include/vlpmap.h"

#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace velodyne
{

vlpmap::vlpmap(const std::string& pcap)
{
	grabber = boost::shared_ptr<grabber_type>( new grabber_type(pcap));
	boost::function<void (const cloud_pointer&)> cloud_cb = boost::bind(&vlpmap::cloud_callback, this, _1);
	cloud_connection = grabber->registerCallback(cloud_cb);
}

vlpmap::~vlpmap()
{
	delete grabber;
}

void vlpmap::cloud_callback(const cloud_pointer& cloud)
{
	boost::mutex::scoped_lock lock(cloud_mutex);
	_cloud = cloud;
}

void vlpmap::run()
{
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

	std::cout << "Cloud stream size: " << cloud_stream.size() << std::endl;

	grabber->stop();

	if (cloud_connection.connected())
	{
		cloud_connection.disconnect();
	}

	icp();
}

void vlpmap::icp()
{
	auto c0 = cloud_stream[10];
	auto c1 = cloud_stream[30];
	
	pcl::IterativeClosestPoint<point_type, point_type> icp;
	icp.setInputSource(c1);
	icp.setInputTarget(c0);

	cloud_type f;
	icp.setMaxCorrespondenceDistance (0.05);
	icp.setMaximumIterations (50);
	icp.setTransformationEpsilon (1e-8);
	icp.setEuclideanFitnessEpsilon (1);
	icp.align(f);
	std::cout << "has converged: " << icp.hasConverged() << " score: " << icp.getFitnessScore() << std::endl;

	cloud_pointer cf(boost::make_shared<const cloud_type>(f));

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "Velodyne Viewer" ) );
    viewer->addCoordinateSystem( 3.0, "coordinate" );
    viewer->setBackgroundColor(0.0, 0.0, 0.0, 0);
    viewer->initCameraParameters();
    viewer->setCameraPosition( 0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0 );

	pcl::visualization::PointCloudColorHandlerCustom<point_type> source (c0, 0, 255, 0);
	pcl::visualization::PointCloudColorHandlerCustom<point_type> target (c1, 0, 0, 255);
	pcl::visualization::PointCloudColorHandlerCustom<point_type> result (cf, 255, 0, 0);
    /* pcl::visualization::PointCloudColorHandler<point_type>::Ptr handler; */
    /*     boost::shared_ptr<pcl::visualization::PointCloudColorHandlerGenericField<point_type>> color_handler( new pcl::visualization::PointCloudColorHandlerGenericField<point_type>( "intensity" ) ); */

	/* handler = color_handler; */

	viewer->addPointCloud(c0, target, "c0");

	viewer->addPointCloud(c1, source, "c1");

	viewer->addPointCloud(cf, result, "cf");

	viewer->spin();
}

} //velodyne
