#ifndef SLAM_COMMON_H
#define SLAM_COMMON_H

#include <array>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/make_shared.hpp>
/* #include <pcl/io/vlp_grabber.h> */
//#include <pcl/visualization/pcl_visualizer.h>

namespace slam
{

using point_type = pcl::PointXYZI;
using cloud_type = pcl::PointCloud<point_type>;
using cloud_ptr = cloud_type::Ptr;
using cloud_const_ptr = cloud_type::ConstPtr;

size_t const MAXIMUM_CLOUD_SIZE = 40000;
size_t const N_SCANS = 16;

struct CloudScans
{
	cloud_ptr cloud;
	std::array<size_t, N_SCANS + 1> scan_ind;

	CloudScans() {}

	CloudScans(cloud_ptr& _cloud, std::array<size_t, N_SCANS + 1> _inds)
		:cloud(_cloud), scan_ind(_inds) {}

	CloudScans(cloud_type& _cloud, std::array<size_t, N_SCANS + 1> _inds)
		:cloud(&_cloud), scan_ind(_inds) {}

	CloudScans(CloudScans const& other)
		: scan_ind(other.scan_ind)
	{
		cloud = boost::make_shared<cloud_type>(*other.cloud);
	}

	void reset_cloud()
	{
		cloud.reset(new cloud_type());
	}

	size_t scan(size_t ind) const
	{
		size_t scanInd = (size_t) (ind / (cloud->points.size() / 16.0));
		if (ind > scan_ind[scanInd])
		{
			while (scanInd < N_SCANS + 1)
			{
				scanInd++;
				if (ind < scan_ind[scanInd])
				{
					scanInd--;
					break;
				}
			}
		}
		else if (ind < scan_ind[scanInd])
		{
			while (scanInd < N_SCANS + 1)
			{
				scanInd--;
				if (ind > scan_ind[scanInd])
				{
					break;
				}
			}
		}
		return scanInd;
	}

	float scan_percent(size_t ind) const
	{
		size_t scanInd = scan(ind);
		size_t start = scan_ind[scanInd];
		size_t end = scan_ind[scanInd + 1];
		return ((float)ind - start) / (end - start);
	}

};

struct CloudFeatures
{
	cloud_ptr edge_cloud;
	cloud_ptr surface_cloud;

	CloudFeatures() {}

	CloudFeatures(CloudFeatures const& other)
	{
		edge_cloud = boost::make_shared<cloud_type>(*other.edge_cloud);
		surface_cloud = boost::make_shared<cloud_type>(*other.surface_cloud);
	}

	friend void swap(CloudFeatures& first, CloudFeatures& second)
	{
		std::swap(first.edge_cloud, second.edge_cloud);
		std::swap(first.surface_cloud, second.surface_cloud);
	}

	CloudFeatures& operator=(CloudFeatures other)
	{
		swap(*this, other);
		return *this;
	}

	void reset()
	{
		edge_cloud.reset(new cloud_type());
		surface_cloud.reset(new cloud_type());
	}

	void clear()
	{
		edge_cloud->clear();
		surface_cloud->clear();
	}
};


struct ScanFeatures
{
	CloudScans edge_scans;
	CloudScans surface_scans;

	ScanFeatures() {}

	ScanFeatures(ScanFeatures const& other)
		: edge_scans(other.edge_scans), surface_scans(other.surface_scans) {}

	friend void swap(ScanFeatures& first, ScanFeatures& second)
	{
		std::swap(first.edge_scans, second.edge_scans);
		std::swap(first.surface_scans, second.surface_scans);
	}
	
	ScanFeatures& operator=(ScanFeatures other)
	{
		swap(*this, other);
		return *this;
	}

	void reset()
	{
		edge_scans.reset_cloud();
		surface_scans.reset_cloud();
	}

	operator CloudFeatures()
	{
		CloudFeatures cf;
		cf.edge_cloud = edge_scans.cloud;
		cf.surface_cloud = surface_scans.cloud;
		return cf;
	}

};

} // slam

#endif // SLAM_COMMON_H
