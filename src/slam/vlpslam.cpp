#ifndef VLP_SLAM_H
#define VLP_SLAM_H

#include "../include/slam/vlpslam.h"
#include "../include/slam/scan_grabber.h"
#include <string>
#include <boost/make_shared.hpp>

//Circular buffer
//We have an asynchronous call to features
//

namespace slam
{
	pcap
	process_stack
	type for holding scans and features
	grabber
	connection
	_scans
	features
	odometry
	mapping
	scan_mutex
	feature_mutex
	mapping_mutex
void vlpslam::run(string fname)
{
	pcap = fname;
	grabber = boost::make_shared(new scan_grabber(pcap));
	boost::function<slam::scan_grabber::sig_cb_sweep__scans_array> scan_callback = 
		[&_scans, &scan_mutex] (const scan_grabber::scan_array_ptr& scan_arr)
		{
			boost::mutex::scoped_lock scan_lock(scan_mutex);
			_scans = scan_arr;
		};
	connection = grabber->registerCallback(scan_callback);
	grabber->start();
	while (grabber->isRunning())
	{
		boost::mutex::scoped_try_lock lock(scan_mutex);
		if (lock.owns_lock() && _scans)
		{
			process(_scans);
		}
	}

	if (scan_connection.connected())
	{
		scan_connection.disconnect();
	}
}

//features in another method called async
//
void process(scan_grabber::scan_array_ptr scans)
{
	CloudScans scan_cloud(scans);
	boost::mutex::scoped_lock f_lock(features_mutex);
	features.process(scans);
	process_stack.push_back(scans, features.odometry_features, map_features);
	f_lock.unlock();

	boost::mutex::scoped_lock m_lock(mapping_mutex);
	run odometry and mapping

}

}

#endif // VLP_SLAM_H
