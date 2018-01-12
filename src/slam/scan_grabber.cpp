#include "../../include/slam/scan_grabber.h"
#include <iostream>
#include <cmath>

namespace slam
{

scan_grabber::scan_grabber (const std::string& pcapFile):
	pcl::VLPGrabber(pcapFile),
	sweep_scans_(new scan_array_type())
{
	initialize();
	/* initializeLaserMapping(); */
}

scan_grabber::scan_grabber (const boost::asio::ip::address& ipAddress,
							const uint16_t port)
    : pcl::VLPGrabber(ipAddress, port)
{
	initialize();
  /* initializeLaserMapping (); */
}

scan_grabber::~scan_grabber() throw()
{
	stop();
	disconnect_all_slots<sig_cb_sweep_scans_array>();
}

void scan_grabber::initialize()
{
	for (int i = 0; i < sweep_scans_->size(); i++)
	{
		(*sweep_scans_)[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
	}
	sweep_scans_signal_ = createSignal<sig_cb_sweep_scans_array>();
}

void scan_grabber::initializeLaserMapping ()
{
  for (uint8_t i = 0; i < VLP_MAX_NUM_LASERS / 2u; i++)
  {
     laser_rgb_mapping_[i * 2].b = static_cast<uint8_t> (i * 6 + 64);
     laser_rgb_mapping_[i * 2 + 1].b = static_cast<uint8_t> ((i + 16) * 6 + 64);
  }
}

void scan_grabber::fireCurrentSweep()
{
	/* HDLGrabber::fireCurrentSweep(); */
	if (sweep_xyzi_signal_ != NULL && sweep_xyzi_signal_->num_slots() > 0)
	{
		boost::shared_ptr<pcl::PointCloud<pcl::PointXYZI> > current_sweep_xyzi_(new pcl::PointCloud<pcl::PointXYZI>);
		for (size_t i = 0; i < sweep_scans_->size(); i++)
		{
			(*current_sweep_xyzi_) += *(*sweep_scans_)[i];
		}
		/* current_current_sweep_xyzi__.reset(current_sweep_xyzi_) */
		sweep_xyzi_signal_->operator() (current_sweep_xyzi_);
	}
	if (sweep_scans_signal_ != NULL && sweep_scans_signal_->num_slots () > 0)
	{
		sweep_scans_signal_->operator() (sweep_scans_);
	}
}

void scan_grabber::toPointClouds (HDLDataPacket *dataPacket)
{
	static uint32_t scan_counter = 0;
	static uint32_t sweep_counter = 0;
	static uint32_t pushed = 0;
	if (sizeof(HDLLaserReturn) != 3)
		return;

	time_t system_time;
	time (&system_time);
	time_t velodyne_time = (system_time & 0x00000000ffffffffl) << 32 | dataPacket->gpsTimestamp;

	scan_counter++;

	double interpolated_azimuth_delta;

	uint8_t index = 1;
	if (dataPacket->mode == VLP_DUAL_MODE)
	{
		index = 2;
	}
	if (dataPacket->firingData[index].rotationalPosition < dataPacket->firingData[0].rotationalPosition)
	{
		interpolated_azimuth_delta = ((dataPacket->firingData[index].rotationalPosition + 36000) - dataPacket->firingData[0].rotationalPosition) / 2.0;
	}
	else
	{
		interpolated_azimuth_delta = (dataPacket->firingData[index].rotationalPosition - dataPacket->firingData[0].rotationalPosition) / 2.0;
	}

	for (uint8_t i = 0; i < HDL_FIRING_PER_PKT; ++i)
	{
		HDLFiringData firing_data = dataPacket->firingData[i];

		for (uint8_t j = 0; j < HDL_LASER_PER_FIRING; j++)
		{
			int scan_index = int(j);
			if (scan_index > 15)
			{
				scan_index -= 16;
			}
			double current_azimuth = firing_data.rotationalPosition;
			if (j >= VLP_MAX_NUM_LASERS)
			{
				current_azimuth += interpolated_azimuth_delta;
			}
			if (current_azimuth > 36000)
			{
				current_azimuth -= 36000;
			}
			if (current_azimuth < HDLGrabber::last_azimuth_)
			{
				if (current_sweep_xyz_->size () > 0)
				{
					/* std::cout << "PUSHED: " << pushed << std::endl; */
					current_sweep_xyz_->is_dense = current_sweep_xyzrgb_->is_dense = current_sweep_xyzi_->is_dense = false;
					current_sweep_xyz_->header.stamp = velodyne_time;
					current_sweep_xyzrgb_->header.stamp = velodyne_time;
					current_sweep_xyzi_->header.stamp = velodyne_time;
					current_sweep_xyz_->header.seq = sweep_counter;
					current_sweep_xyzrgb_->header.seq = sweep_counter;
					current_sweep_xyzi_->header.seq = sweep_counter;

					(*sweep_scans_)[0]->header.stamp = velodyne_time;
					(*sweep_scans_)[0]->header.seq = sweep_counter;

					sweep_counter++;

					fireCurrentSweep ();
				}
				current_sweep_xyz_.reset (new pcl::PointCloud<pcl::PointXYZ> ());
				current_sweep_xyzrgb_.reset (new pcl::PointCloud<pcl::PointXYZRGBA> ());
				current_sweep_xyzi_.reset (new pcl::PointCloud<pcl::PointXYZI> ());

				sweep_scans_.reset(new scan_array_type());
				for (size_t i = 0; i < sweep_scans_->size(); i++)
				{
					(*sweep_scans_)[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
				}
				pushed = 0;
			}

			pcl::PointXYZ xyz;
			pcl::PointXYZI xyzi;
			pcl::PointXYZRGBA xyzrgb;
			pcl::PointXYZ dual_xyz;
			pcl::PointXYZI dual_xyzi;
			pcl::PointXYZRGBA dual_xyzrgb;

			HDLGrabber::computeXYZI (xyzi, current_azimuth, firing_data.laserReturns[j], laser_corrections_[j % VLP_MAX_NUM_LASERS]);

			xyz.x = xyzrgb.x = xyzi.x;
			xyz.y = xyzrgb.y = xyzi.y;
			xyz.z = xyzrgb.z = xyzi.z;

			xyzrgb.rgb = laser_rgb_mapping_[j % VLP_MAX_NUM_LASERS].rgb;

			if (dataPacket->mode == VLP_DUAL_MODE)
			{
				HDLGrabber::computeXYZI (dual_xyzi, current_azimuth, dataPacket->firingData[i + 1].laserReturns[j], laser_corrections_[j % VLP_MAX_NUM_LASERS]);

				dual_xyz.x = dual_xyzrgb.x = dual_xyzi.x;
				dual_xyz.y = dual_xyzrgb.y = dual_xyzi.y;
				dual_xyz.z = dual_xyzrgb.z = dual_xyzi.z;

				dual_xyzrgb.rgb = laser_rgb_mapping_[j % VLP_MAX_NUM_LASERS].rgb;
			}

			if (! (pcl_isnan (xyz.x) || pcl_isnan (xyz.y) || pcl_isnan (xyz.z)))
			{
				current_sweep_xyz_->push_back (xyz);
				current_sweep_xyzrgb_->push_back (xyzrgb);
				current_sweep_xyzi_->push_back (xyzi);

				/* std::cout << "PUSHING: " << scan_index << std::endl; */
				(*sweep_scans_)[scan_index]->push_back(xyzi);
				pushed++;

				last_azimuth_ = current_azimuth;
			}
			if (dataPacket->mode == VLP_DUAL_MODE)
			{
				if ((dual_xyz.x != xyz.x || dual_xyz.y != xyz.y || dual_xyz.z != xyz.z)
						&& ! (pcl_isnan (dual_xyz.x) || pcl_isnan (dual_xyz.y) || pcl_isnan (dual_xyz.z)))
				{
					current_sweep_xyz_->push_back (dual_xyz);
					current_sweep_xyzrgb_->push_back (dual_xyzrgb);
					current_sweep_xyzi_->push_back (dual_xyzi);

					/* std::cout << "PUSHING: " << scan_index << std::endl; */
					(*sweep_scans_)[scan_index]->push_back(dual_xyzi);
					pushed++;
				}
			}
		}
		if (dataPacket->mode == VLP_DUAL_MODE)
		{
			i++;
		}
	}

} // scan_grabber

} // slam
