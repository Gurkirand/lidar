#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <vector>
#include <string>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/vlp_grabber.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace velodyne
{

template <typename P>
class dualport_visualizer
{

private:

	using cloud_pointer = P::ptr;
	using color_handler = pcl::visualization::PointCloudColorHandler;

	struct cloud_view
	{
	public:
		std::string name;
		cloud_pointer cloud;
		color_handler* handler;
	}

	pcl::visualization::PCLVisualizer *view;
	std::string name;
	int v1;
	int v2;
	std::vector<cloud_view> v1_clouds;
	std::vector<cloud_view> v2_clouds;

public:

	enum ports
	{
		LEFT, RIGHT
	}

	visualizer(std::string& name): name(name)
	{
		view = new pcl::visualization::PCLVisualizer(name);
		view->addCoordinateSystem( 3.0, "coordinate" );
		view->setBackgroundColor(0.0, 0.0, 0.0, 0);
		view->initCameraParameters();
		view->setCameraPosition( 0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0 );
		view->createViewPort(0.0, 0, 0.5, 1.0, v1);
		view->createViewPort(0.5, 0, 1.0, 1.0, v2);
	}

	~visualizer()
	{
		delete view;
	}
	
	void addPointCloud(cloud_pointer cloud,
					std::string name,
					color_handler* handler
					int port)
	{
		int port = (p == LEFT) ? v1: v2;
		auto *clouds = (p == LEFT) ? v1_clouds: v2_clouds;
		name = ((p == LEFT) ? "v1_": "v2_") + name;

		view->addPointCloud(cloud, *handler, name, port);

		cloud_view cv(name, cloud, handler);
		clouds->push_back(cv);
	}

	void update(ports p)
	{
		int port = (p == LEFT) ? v1: v2;
		auto *clouds = (p == LEFT) ? v1_clouds: v2_clouds;
		name = ((p == LEFT) ? "v1_": "v2_") + name;
		
		cloud_view *cv;
		for (auto itr = clouds.begin(), end = clouds.end();
				itr < end; itr++)
		{
			cv = *itr;
			view->removePointCloud(cv->name);
			view->addPointCloud(cv->cloud, *(cv->handler), cv->name, port);
		}

		view->spinOnce();
	}

};

} // velodyne

#endif // VISUALIZER_H
