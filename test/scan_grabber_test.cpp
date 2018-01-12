#include <iostream>
#include <string>
#include <vector>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/console/parse.h>
/* #include <pcl/io/hdl_grabber.h> */
/* #include <pcl/io/pcd_io.h> */
/* #include <pcl/visualization/pcl_visualizer.h> */
#include <cmath>

#include "../include/slam/scan_grabber.h"

typedef pcl::PointXYZI PointType;

int main( int argc, char *argv[] )
{
    // Command-Line Argument Parsing
    if( pcl::console::find_switch( argc, argv, "-help" ) ){
        std::cout << "usage: " << argv[0]
            << " [-ipaddress <192.168.1.70>]"
            << " [-port <2368>]"
            << " [-pcap <*.pcap>]"
            << " [-help]"
            << std::endl;
        return 0;
    }

    std::string ipaddress( "192.168.1.70" );
    std::string port( "2368" );
    std::string pcap;

    pcl::console::parse_argument( argc, argv, "-ipaddress", ipaddress );
    pcl::console::parse_argument( argc, argv, "-port", port );
    pcl::console::parse_argument( argc, argv, "-pcap", pcap );

    std::cout << "-ipadress : " << ipaddress << std::endl;
    std::cout << "-port : " << port << std::endl;
    std::cout << "-pcap : " << pcap << std::endl;

    // Point Cloud
    pcl::PointCloud<PointType>::ConstPtr cloud;

    boost::shared_ptr<slam::scan_grabber> grabber;
    if( !pcap.empty() ){
        std::cout << "Capture from PCAP..." << std::endl;
        grabber = boost::shared_ptr<slam::scan_grabber>( new slam::scan_grabber( pcap ) );
    }
    else if( !ipaddress.empty() && !port.empty() ){
        std::cout << "Capture from Sensor..." << std::endl;
        grabber = boost::shared_ptr<slam::scan_grabber>( new slam::scan_grabber( boost::asio::ip::address::from_string( ipaddress ), boost::lexical_cast<unsigned short>( port ) ) );
    }

	slam::scan_grabber::scan_array_ptr scans;

	boost::mutex scan_mutex;
	boost::function<slam::scan_grabber::sig_cb_sweep_scans_array> scan_callback = 
		[&scans, &scan_mutex] (const slam::scan_grabber::scan_array_ptr& scan_arr)
		{
			boost::mutex::scoped_lock scan_lock(scan_mutex);
			scans = scan_arr;
		};

	int sweeps = 0;
	boost::mutex cloud_mutex;
	boost::function<void (const pcl::PointCloud<PointType>::ConstPtr&)> cloud_callback = 
		[&cloud, &cloud_mutex, &sweeps] (const pcl::PointCloud<PointType>::ConstPtr& cloud_ptr)
		{
			boost::mutex::scoped_lock cloud_lock(cloud_mutex);
			std::cout << cloud_ptr->points.size() << std::endl;
			cloud = cloud_ptr;
			sweeps++;
		};

	boost::signals2::connection scan_connection = grabber->registerCallback(scan_callback);
	boost::signals2::connection cloud_connection = grabber->registerCallback(cloud_callback);

	grabber->start();

	while (grabber->isRunning())
	{
		if (sweeps > 2)
		{
			grabber->stop();
		}
	}

    if( scan_connection.connected() ){
        scan_connection.disconnect();
    }
    if( cloud_connection.connected() ){
        cloud_connection.disconnect();
    }

	size_t scan_size = 0;
	for (int i = 0; i < scans->size(); i++)
	{
		scan_size += (*scans)[i]->points.size();
		std::cout << "SCAN: " << i << " SIZE: " << (*scans)[i]->points.size() << std::endl;
	}
	std::cout << '\n' << "SCANS: " << scan_size << std::endl;
	std::cout << "CLOUD: " << cloud->points.size() << std::endl;

	float scanPeriod = 0.1;
	float startOri = -atan2(cloud->points[0].y, cloud->points[0].x);
	float endOri = -atan2(cloud->points[cloud->points.size() - 1].y, cloud->points[cloud->points.size() - 1].x) + 2 * M_PI;

	if (endOri - startOri > 3 * M_PI) {
		endOri -= 2 * M_PI;
	} else if (endOri - startOri < M_PI) {
		endOri += 2 * M_PI;
	}

	bool halfPassed = false;
	PointType point;
	std::array<pcl::PointCloud<PointType>::Ptr, 16> cloud_scans;
	for (int i = 0; i < cloud_scans.size(); i++)
	{
		cloud_scans[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
	}

	for (int i = 0; i < cloud->points.size(); i++)
	{
		point.x = cloud->points[i].y;
		point.y = cloud->points[i].z;
		point.z = cloud->points[i].x;

		float angle = atan(point.y / sqrt(point.x * point.x + point.z * point.z)) * 180 / M_PI;
		int scanID;
		int roundedAngle = int(angle + (angle<0.0?-0.5:+0.5)); 
		if (roundedAngle > 0){
			scanID = roundedAngle;
		}
		else {
			scanID = roundedAngle + (16 - 1);
		}
		if (scanID > (16 - 1) || scanID < 0 ){
			/* count--; */
			continue;
		}

		float ori = -atan2(point.x, point.z);
		if (!halfPassed) {
			if (ori < startOri - M_PI / 2) {
				ori += 2 * M_PI;
			} else if (ori > startOri + M_PI * 3 / 2) {
				ori -= 2 * M_PI;
			}

			if (ori - startOri > M_PI) {
				halfPassed = true;
			}
		} else {
			ori += 2 * M_PI;

			if (ori < endOri - M_PI * 3 / 2) {
				ori += 2 * M_PI;
			} else if (ori > endOri + M_PI / 2) {
				ori -= 2 * M_PI;
			} 
		}

		float relTime = (ori - startOri) / (endOri - startOri);
		point.intensity = scanID + scanPeriod * relTime;
		cloud_scans[scanID]->push_back(point);
	}

	for (int i = 0; i < 16; i++)
	{
		std::cout << "SCAN NUM: " << i << std::endl;
		std::cout << "SCANS: " << (*scans)[i]->points.size() << std::endl;
		std::cout << "CLOUD: " << cloud_scans[i] ->points.size() << std::endl;

	}

	pcl::PointCloud<PointType> scan_cloud;


	for (int i = 0; i < 16; i++)
	{
		scan_cloud += *(*scans)[i];
	}

	int laser = 0;
	for (int i = 0; i < scan_cloud.points.size(); i++)
	{
		point.x = scan_cloud.points[i].y;
		point.y = scan_cloud.points[i].z;
		point.z = scan_cloud.points[i].x;

		float angle = atan(point.y / sqrt(point.x * point.x + point.z * point.z)) * 180 / M_PI;
		int scanID;
		int roundedAngle = int(angle + (angle<0.0?-0.5:+0.5)); 
		if (roundedAngle > 0){
			scanID = roundedAngle;
		}
		else {
			scanID = roundedAngle + (16 - 1);
		}
		if (scanID > (16 - 1) || scanID < 0 ){
			/* count--; */
			continue;
		}
		if (scanID == laser + 1)
		{
			std::cout << "Point " << i << " end of laser " << laser << std::endl;
			laser++;
		}
		else if (scanID != laser)
		{
			std::cout << "Point " << i << " failed laser " << laser << " should be " << scanID << std::endl;
		}
	}

	startOri = -atan2((*scans)[0]->points[0].y, (*scans)[0]->points[0].x);
	endOri = -atan2((*scans)[0]->points[(*scans)[0]->points.size() - 1].y, (*scans)[0]->points[(*scans)[0]->points.size() - 1].x) + 2 * M_PI;
	for (int i = 0; i < (*scans)[0]->size(); i++)
	{
		point = (*scans)[0]->points[i];
		float ori = -atan2(point.x, point.z);
		if (!halfPassed) {
			if (ori < startOri - M_PI / 2) {
				ori += 2 * M_PI;
			} else if (ori > startOri + M_PI * 3 / 2) {
				ori -= 2 * M_PI;
			}

			if (ori - startOri > M_PI) {
				halfPassed = true;
			}
		} else {
			ori += 2 * M_PI;

			if (ori < endOri - M_PI * 3 / 2) {
				ori += 2 * M_PI;
			} else if (ori > endOri + M_PI / 2) {
				ori -= 2 * M_PI;
			} 
		}

		float relTime = (ori - startOri) / (endOri - startOri);
		if (i % 100 == 0)
		{
			std::cout << std::endl;
			std::cout << relTime << std::endl;
			std::cout << cloud_scans[0]->points[i].intensity << std::endl;
			std::cout << (1.0 * i) / (*scans)[0]->points.size() << std::endl;
		}
	}

    return 0;
}
