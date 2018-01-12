#ifndef SLAM_SCAN_GRABBER_H
#define SLAM_SCAN_GRABBER_H

#include <pcl/io/vlp_grabber.h>
#include <pcl/io/hdl_grabber.h>
#include <pcl/io/grabber.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <string>

namespace slam
{

struct scan_grabber: public pcl::VLPGrabber
{

	static const uint8_t VLP_MAX_NUM_LASERS = 16;
	typedef std::array<pcl::PointCloud<pcl::PointXYZI>::Ptr, VLP_MAX_NUM_LASERS> scan_array_type;
	typedef boost::shared_ptr<scan_array_type> scan_array_ptr;
	typedef void (sig_cb_sweep_scans_array) (const scan_array_ptr&);

private:

	static const uint8_t VLP_DUAL_MODE = 0x39;

	pcl::RGB laser_rgb_mapping_[VLP_MAX_NUM_LASERS];

	boost::signals2::signal<sig_cb_sweep_scans_array>* sweep_scans_signal_;
	scan_array_ptr sweep_scans_;

	void initialize ();

	void initializeLaserMapping ();

	void fireCurrentSweep ();

	void toPointClouds (HDLDataPacket *dataPacket);

public:
	
	scan_grabber (const std::string& pcapFile="");

	scan_grabber (const boost::asio::ip::address& ipAddress,
				const uint16_t port);

	~scan_grabber () throw ();

};

} // slam

#endif // SLAM_SCAN_GRABBER_H
