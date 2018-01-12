#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include "../../include/slam/feature_extraction.h"
#include <pcl/filters/voxel_grid.h>

namespace slam
{

feature_extraction::feature_extraction() {}

void feature_extraction::process(CloudScans const& source)
{
	source_scans = &source;
	cloud_size = source.cloud->points.size();

	odometry_features.reset();
	map_features.reset();

	/* something(source.cloud); */
	/* preprocess_scans(sourceCloud); */
	evaluate_smoothness();
	reject_features();
	extract_features();
}

void feature_extraction::evaluate_smoothness ()
{
	cloud_type const& sourceCloud = *(source_scans->cloud);
	size_t end = sourceCloud.points.size() - sample_size; 

	for (size_t i = sample_size; i < end; i++)
	{
		float diffX = 0.0, diffY = 0.0, diffZ = 0.0;

		/*
		 * We want to calculate the size of the distance between the current point,
		 * sourceCloud.points[i], and the points in its sample. To avoid branching
		 * We just add up all points, including the current point, and then subtract
		 * the correct amount of the current point. We can just do addition since
		 * the sign will be lost when we find the square distance.
		 */
        for (int j = i - sample_size; j <= i + sample_size; j++)
        {
	        diffX += sourceCloud.points[j].x;
	        diffY += sourceCloud.points[j].y;
	        diffZ += sourceCloud.points[j].z;
        }

        /*
         * We added 2 * sample_size points around the current point and the
         * current point as well. Therefore we need to subtract 2 * sample_size
         * + 1 from the coordinates.
         */
		int const diff_multiplier = 2 * sample_size + 1;

        diffX -= sourceCloud.points[i].x * diff_multiplier;
        diffY -= sourceCloud.points[i].y * diff_multiplier;
        diffZ -= sourceCloud.points[i].z * diff_multiplier;

        // Place coefficient in our member container
        smoothness_coeff[i] = diffX*diffX + diffY*diffY + diffZ*diffZ; 
		sorted_ind[i] = i;
		neighbor_picked[i] = 0;
	}
}

void feature_extraction::reject_features ()
{
	cloud_type const& sourceCloud = *(source_scans->cloud);
	size_t end = sourceCloud.points.size() - sample_size - 1;

	for (size_t i = sample_size; i < end; i++) {
		float diff = sqr_diff(sourceCloud.points[i + 1], sourceCloud.points[i]);
		float dist = sqr_len(sourceCloud.points[i]);

		if (diff > 0.1) {
			
			float dist1 = sqrt(dist);
			float dist2 = length(sourceCloud.points[i + 1]);

			if (dist1 > dist2) {
				float scaledDiff = sqrt(sqr_diff(sourceCloud.points[i + 1], scale(sourceCloud.points[i], dist2 / dist1)));

				if (scaledDiff / dist2 < 0.1) {
					for (size_t j = i - sample_size; j <= i; j++)
					{
						neighbor_picked[j] = 1;
					}
				}
			} else {
				float scaledDiff = sqrt(sqr_diff(scale(sourceCloud.points[i + 1], dist1 / dist2), sourceCloud.points[i]));

				if (scaledDiff / dist1 < 0.1) {
					for (size_t j = i + 1; j <= i + 1 + sample_size; j++)
					{
						neighbor_picked[j] = 1;
					}
				}
			}
		}

		float diff2 = sqr_diff(sourceCloud.points[i], sourceCloud.points[i - 1]);

		if (diff > 0.0002 * dist && diff2 > 0.0002 * dist) {
			neighbor_picked[i] = 1;
		}
	}
}

void feature_extraction::sort_coeffs (size_t const start, size_t const end)
{
	for (size_t i = start + 1; i <= end; i++)
	{
		for (size_t j = i; j >= start + 1; j--)
		{
			if (smoothness_coeff[sorted_ind[j]] < smoothness_coeff[sorted_ind[j - 1]])
			{
				size_t temp = sorted_ind[j - 1];
				sorted_ind[j - 1] = sorted_ind[j];
				sorted_ind[j] = temp;
			}
		}
	}
}

void feature_extraction::extract_features ()
{
	cloud_type const& sourceCloud = *(source_scans->cloud);
	cloud_type& odometry_edge_points = *(odometry_features.edge_scans.cloud);
	cloud_type& odometry_surface_points = *(odometry_features.surface_scans.cloud);
	cloud_type& map_edge_points = *(map_features.edge_scans.cloud);
	cloud_type& map_surface_points = *(map_features.surface_scans.cloud);

	odometry_features.edge_scans.scan_ind[0] = 0;
	odometry_features.surface_scans.scan_ind[0] = 0;
	map_features.edge_scans.scan_ind[0] = 0;
	map_features.surface_scans.scan_ind[0] = 0;

	for (size_t i = 0; i < N_SCANS; i++)
	{
		cloud_ptr surfacePoints(new cloud_type);

		size_t const s = source_scans->scan_ind[i] + sample_size;
		size_t const e = source_scans->scan_ind[i + 1] - sample_size;

		for (size_t j = 0; j < sample_size + 1; j++) {

			size_t const start = (s * (sample_size - j + 1) + e * j) / (sample_size + 1);
			size_t const end = (s * (sample_size - j) + e * (j + 1)) / (sample_size + 1) - 1;

			sort_coeffs(start, end);

			size_t edgeFeatureCount = 0;
			for (size_t k = end; k >= start && edgeFeatureCount < 20; k--)
			{
				size_t ind = sorted_ind[k];
				if (neighbor_picked[ind] == 0 && smoothness_coeff[ind] > 0.1)
				{
					edgeFeatureCount++;
					if (edgeFeatureCount <= 2)
					{
						odometry_edge_points.push_back(sourceCloud.points[ind]);
					}
					map_edge_points.push_back(sourceCloud.points[ind]);

					neighbor_picked[ind] = 1;

					for (int l = 1; l <= sample_size; l++)
					{
						if (sqr_diff(sourceCloud.points[ind + l], sourceCloud.points[ind + l - 1]) > 0.05)
						{
							break;
						}
						neighbor_picked[ind + l] = 1;
					}
					for (int l = -1; l >= -sample_size; l--)
					{
						if (sqr_diff(sourceCloud.points[ind + l], sourceCloud.points[ind + l + 1]) > 0.05)
						{
							break;
						}
						neighbor_picked[ind + l] = 1;
					}
				}
			}

			size_t surfaceFeatureCount = 0;
			for (size_t k = start; k <= end; k++)
			{
				size_t ind = sorted_ind[k];
				if (neighbor_picked[ind] == 0 && smoothness_coeff[ind] < 0.1)
				{
					odometry_surface_points.push_back(sourceCloud.points[ind]);
					surfacePoints->push_back(sourceCloud.points[k]);

					surfaceFeatureCount++;
					if (surfaceFeatureCount == 4)
					{
						break;
					}

					neighbor_picked[ind] = 1;
					for (int l = 1; l <= sample_size; l++)
					{
						if (sqr_diff(sourceCloud.points[ind + l], sourceCloud.points[ind + l - 1]) > 0.05)
						{
							break;
						}
						neighbor_picked[ind + l] = 1;
					}
					for (int l = -1; l >= -sample_size; l--)
					{
						if (sqr_diff(sourceCloud.points[ind + l], sourceCloud.points[ind + l + 1]) > 0.05)
						{
							break;
						}
						neighbor_picked[ind + l] = 1;
					}
				}
			}

			/* for (int k = start; k <= end; k++) { */
			/* 	if (cloudLabel[k] <= 0) { */
			/* 		map_features.surface_pointsScan->push_back(laserCloud->points[k]); */
			/* 	} */
			/* } */
		}

		cloud_type surfacePointsDownSample;
		pcl::VoxelGrid<point_type> downSampleFilter;
		downSampleFilter.setInputCloud(surfacePoints);
		downSampleFilter.setLeafSize(0.2, 0.2, 0.2);
		downSampleFilter.filter(surfacePointsDownSample);
		
		map_surface_points += surfacePointsDownSample;

		odometry_features.edge_scans.scan_ind[i + 1] = odometry_edge_points.size();
		odometry_features.surface_scans.scan_ind[i + 1] = odometry_surface_points.size();
		map_features.edge_scans.scan_ind[i + 1] = map_edge_points.size();
		map_features.surface_scans.scan_ind[i + 1] = map_surface_points.size();
		/* std::cout << odometry_edge_points.points.size() << std::endl; */
	}
	/* std::cout << "CORNER: " <<  odometry_edge_points.points.size() << std::endl; */
	/* std::cout << "CORNER: " <<  map_edge_points.points.size() << std::endl; */
	/* std::cout << "SURF: " <<  odometry_surface_points.points.size() << std::endl; */
	/* std::cout << "SURF: " <<  map_surface_points.points.size() << '\n' << std::endl; */
}

void feature_extraction::something(cloud_type const& sourceCloud)
{
	std::vector<int> scanStartInd(N_SCANS, 0);
	std::vector<int> scanEndInd(N_SCANS, 0);
	float startOri = -atan2(sourceCloud.points[0].y, sourceCloud.points[0].x);
	float endOri = -atan2(sourceCloud.points[cloud_size - 1].y,
			sourceCloud.points[cloud_size - 1].x) + 2 * M_PI;
	float scanPeriod = 0.1;

	if (endOri - startOri > 3 * M_PI) {
		endOri -= 2 * M_PI;
	} else if (endOri - startOri < M_PI) {
		endOri += 2 * M_PI;
	}
	bool halfPassed = false;
	int count = cloud_size;
	point_type point;
	std::vector<cloud_type> scans(N_SCANS);
	for (int i = 0; i < cloud_size; i++) {
		point.x = sourceCloud.points[i].y;
		point.y = sourceCloud.points[i].z;
		point.z = sourceCloud.points[i].x;

		float angle = atan(point.y / sqrt(point.x * point.x + point.z * point.z)) * 180 / M_PI;
		int scanID;
		int roundedAngle = int(angle + (angle<0.0?-0.5:+0.5)); 
		if (roundedAngle > 0){
			scanID = roundedAngle;
		}
		else {
			scanID = roundedAngle + (N_SCANS - 1);
		}
		if (scanID > (N_SCANS - 1) || scanID < 0 ){
			count--;
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

/* 		if (i % 500 == 0 || (i - 1) % 500 == 0) */
/* 		{ */
/* 			std::cout << "Ori: " << ori << std::endl; */
/* 			std::cout << "startOri: " << startOri << std::endl; */
/* 			std::cout << "endOri: " << endOri << std::endl; */
/* 			std::cout << "totalori: " << startOri - endOri << std::endl; */
/* 			std::cout << "ID: " << scanID << std::endl; */
/* 			std::cout << "relTime: " << relTime << std::endl; */
/* 			std::cout << "Intensity: " << point.intensity << '\n' << std::endl; */
/* 		} */
		scans[scanID].push_back(point);
	}
	cloud_size = count;

	pcl::PointCloud<point_type>::Ptr laserCloud(new pcl::PointCloud<point_type>());

	for (int i = 0; i < N_SCANS; i++) {
		*laserCloud += scans[i];
	}
	/* std::cout << laserCloud->points.size() << std::endl; */
    /* boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "Velodyne Viewer" ) ); */
    /* viewer->addCoordinateSystem( 3.0, "coordinate" ); */
    /* viewer->setBackgroundColor( 0.0, 0.0, 0.0, 0 ); */
    /* viewer->initCameraParameters(); */
    /* viewer->setCameraPosition( 0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0 ); */

	/* boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<point_type>> color_handler( new pcl::visualization::PointCloudColorHandlerCustom<point_type>( 0, 255, 255) ); */

	/* color_handler->setInputCloud( laserCloud ); */
	/* viewer->addPointCloud( laserCloud, *color_handler, "laserCloud" ); */
	/* viewer->spin(); */
	int scanCount = -1;
	for (size_t i = sample_size; i < cloud_size - sample_size; i++)
	{
		float diffX = 0.0, diffY = 0.0, diffZ = 0.0;

		/*
		 * We want to calculate the size of the distance between the current point,
		 * laserCloud->points[i], and the points in its sample. To avoid branching
		 * We just add up all points, including the current point, and then subtract
		 * the correct amount of the current point. We can just do addition since
		 * the sign will be lost when we find the square distance.
		 */
        for (int j = i - sample_size; j <= i + sample_size; j++)
        {
	        diffX += laserCloud->points[j].x;
	        diffY += laserCloud->points[j].y;
	        diffZ += laserCloud->points[j].z;
        }

        /*
         * We added 2 * sample_size points around the current point and the
         * current point as well. Therefore we need to subtract 2 * sample_size
         * + 1 from the coordinates.
         */
		int const diff_multiplier = 2 * sample_size + 1;

        diffX -= laserCloud->points[i].x * diff_multiplier;
        diffY -= laserCloud->points[i].y * diff_multiplier;
        diffZ -= laserCloud->points[i].z * diff_multiplier;

        // Place coefficient in our member container
        smoothness_coeff[i] = diffX*diffX + diffY*diffY + diffZ*diffZ; 
		neighbor_picked[i] = 0;
		sorted_ind[i] = i;

		if (int(laserCloud->points[i].intensity) != scanCount) {
			scanCount = int(laserCloud->points[i].intensity);

			if (scanCount > 0 && scanCount < N_SCANS) {
				scanStartInd[scanCount] = i + 5;
				scanEndInd[scanCount - 1] = i - 5;
			}
		}
	}

/* 	for (int i = 0; i < N_SCANS; i++) { */
/* 		std::cout << "scanID: " << i << " start: " << scanStartInd[i] << " end: " << scanEndInd[i] << std::endl; */
/* 		for (int j = 0; j < 6; j++) { */
/* 			int sp = (scanStartInd[i] * (6 - j)  + scanEndInd[i] * j) / 6; */
/* 			int ep = (scanStartInd[i] * (5 - j)  + scanEndInd[i] * (j + 1)) / 6 - 1; */
/* 		} */
/* 	} */

	scanStartInd[0] = 5;
	scanEndInd.back() = cloud_size - 5;


	pcl::PointCloud<point_type>::Ptr rejectedPoints(new pcl::PointCloud<point_type>());
	for (int i = 5; i < cloud_size - 6; i++) {
		float diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x;
		float diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y;
		float diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z;
		float diff = diffX * diffX + diffY * diffY + diffZ * diffZ;

		if (diff > 0.1) {

			float depth1 = sqrt(laserCloud->points[i].x * laserCloud->points[i].x + 
					laserCloud->points[i].y * laserCloud->points[i].y +
					laserCloud->points[i].z * laserCloud->points[i].z);

			float depth2 = sqrt(laserCloud->points[i + 1].x * laserCloud->points[i + 1].x + 
					laserCloud->points[i + 1].y * laserCloud->points[i + 1].y +
					laserCloud->points[i + 1].z * laserCloud->points[i + 1].z);

			if (depth1 > depth2) {
				diffX = laserCloud->points[i + 1].x - laserCloud->points[i].x * depth2 / depth1;
				diffY = laserCloud->points[i + 1].y - laserCloud->points[i].y * depth2 / depth1;
				diffZ = laserCloud->points[i + 1].z - laserCloud->points[i].z * depth2 / depth1;

				if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth2 < 0.1) {
					neighbor_picked[i - 5] = 1;
					neighbor_picked[i - 4] = 1;
					neighbor_picked[i - 3] = 1;
					neighbor_picked[i - 2] = 1;
					neighbor_picked[i - 1] = 1;
					neighbor_picked[i] = 1;

					rejectedPoints->push_back(laserCloud->points[i]);
					rejectedPoints->push_back(laserCloud->points[i - 1]);
					rejectedPoints->push_back(laserCloud->points[i - 2]);
					rejectedPoints->push_back(laserCloud->points[i - 3]);
					rejectedPoints->push_back(laserCloud->points[i - 4]);
					rejectedPoints->push_back(laserCloud->points[i - 5]);
				}

			} else {
				diffX = laserCloud->points[i + 1].x * depth1 / depth2 - laserCloud->points[i].x;
				diffY = laserCloud->points[i + 1].y * depth1 / depth2 - laserCloud->points[i].y;
				diffZ = laserCloud->points[i + 1].z * depth1 / depth2 - laserCloud->points[i].z;

				if (sqrt(diffX * diffX + diffY * diffY + diffZ * diffZ) / depth1 < 0.1) {
					neighbor_picked[i + 1] = 1;
					neighbor_picked[i + 2] = 1;
					neighbor_picked[i + 3] = 1;
					neighbor_picked[i + 4] = 1;
					neighbor_picked[i + 5] = 1;
					neighbor_picked[i + 6] = 1;

					rejectedPoints->push_back(laserCloud->points[i + 1]);
					rejectedPoints->push_back(laserCloud->points[i + 2]);
					rejectedPoints->push_back(laserCloud->points[i + 3]);
					rejectedPoints->push_back(laserCloud->points[i + 4]);
					rejectedPoints->push_back(laserCloud->points[i + 5]);
					rejectedPoints->push_back(laserCloud->points[i + 6]);
				}
			}
		}

		float diffX2 = laserCloud->points[i].x - laserCloud->points[i - 1].x;
		float diffY2 = laserCloud->points[i].y - laserCloud->points[i - 1].y;
		float diffZ2 = laserCloud->points[i].z - laserCloud->points[i - 1].z;
		float diff2 = diffX2 * diffX2 + diffY2 * diffY2 + diffZ2 * diffZ2;

		float dis = laserCloud->points[i].x * laserCloud->points[i].x
			+ laserCloud->points[i].y * laserCloud->points[i].y
			+ laserCloud->points[i].z * laserCloud->points[i].z;

		if (diff > 0.0002 * dis && diff2 > 0.0002 * dis) {
			neighbor_picked[i] = 1;

			rejectedPoints->push_back(laserCloud->points[i]);
		}
	}

	pcl::PointCloud<point_type> cornerPointsSharp;
	pcl::PointCloud<point_type> cornerPointsLessSharp;
	pcl::PointCloud<point_type> surfPointsFlat;
	pcl::PointCloud<point_type> surfPointsLessFlat;
	for (int i = 0; i < N_SCANS; i++) {
		pcl::PointCloud<point_type>::Ptr surfPointsLessFlatScan(new pcl::PointCloud<point_type>);
		for (int j = 0; j < 6; j++) {
			int sp = (scanStartInd[i] * (6 - j)  + scanEndInd[i] * j) / 6;
			int ep = (scanStartInd[i] * (5 - j)  + scanEndInd[i] * (j + 1)) / 6 - 1;

			for (int k = sp + 1; k <= ep; k++) {
				for (int l = k; l >= sp + 1; l--) {
					if (smoothness_coeff[sorted_ind[l]] < smoothness_coeff[sorted_ind[l - 1]]) {
						int temp = sorted_ind[l - 1];
						sorted_ind[l - 1] = sorted_ind[l];
						sorted_ind[l] = temp;
					}
				}
			}

			int largestPickedNum = 0;
			for (int k = ep; k >= sp; k--) {
				int ind = sorted_ind[k];
				if (neighbor_picked[ind] == 0 &&
						smoothness_coeff[ind] > 0.1) {

					largestPickedNum++;
					if (largestPickedNum <= 2) {
						cornerPointsSharp.push_back(laserCloud->points[ind]);
						cornerPointsLessSharp.push_back(laserCloud->points[ind]);
					} else if (largestPickedNum <= 20) {
						cornerPointsLessSharp.push_back(laserCloud->points[ind]);
					} else {
						break;
					}

					neighbor_picked[ind] = 1;
					for (int l = 1; l <= 5; l++) {
						float diffX = laserCloud->points[ind + l].x 
							- laserCloud->points[ind + l - 1].x;
						float diffY = laserCloud->points[ind + l].y 
							- laserCloud->points[ind + l - 1].y;
						float diffZ = laserCloud->points[ind + l].z 
							- laserCloud->points[ind + l - 1].z;
						if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
							break;
						}

						neighbor_picked[ind + l] = 1;
					}
					for (int l = -1; l >= -5; l--) {
						float diffX = laserCloud->points[ind + l].x 
							- laserCloud->points[ind + l + 1].x;
						float diffY = laserCloud->points[ind + l].y 
							- laserCloud->points[ind + l + 1].y;
						float diffZ = laserCloud->points[ind + l].z 
							- laserCloud->points[ind + l + 1].z;
						if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
							break;
						}

						neighbor_picked[ind + l] = 1;
					}
				}
			}

			int smallestPickedNum = 0;
			for (int k = sp; k <= ep; k++) {
				int ind = sorted_ind[k];
				if (neighbor_picked[ind] == 0 &&
						smoothness_coeff[ind] < 0.1) {

					surfPointsFlat.push_back(laserCloud->points[ind]);
					surfPointsLessFlatScan->push_back(laserCloud->points[k]);

					smallestPickedNum++;
					if (smallestPickedNum >= 4) {
						break;
					}

					neighbor_picked[ind] = 1;
					for (int l = 1; l <= 5; l++) {
						float diffX = laserCloud->points[ind + l].x 
							- laserCloud->points[ind + l - 1].x;
						float diffY = laserCloud->points[ind + l].y 
							- laserCloud->points[ind + l - 1].y;
						float diffZ = laserCloud->points[ind + l].z 
							- laserCloud->points[ind + l - 1].z;
						if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
							break;
						}

						neighbor_picked[ind + l] = 1;
					}
					for (int l = -1; l >= -5; l--) {
						float diffX = laserCloud->points[ind + l].x 
							- laserCloud->points[ind + l + 1].x;
						float diffY = laserCloud->points[ind + l].y 
							- laserCloud->points[ind + l + 1].y;
						float diffZ = laserCloud->points[ind + l].z 
							- laserCloud->points[ind + l + 1].z;
						if (diffX * diffX + diffY * diffY + diffZ * diffZ > 0.05) {
							break;
						}

						neighbor_picked[ind + l] = 1;
					}
				}
			}
		}

		pcl::PointCloud<point_type> surfPointsLessFlatScanDS;
		pcl::VoxelGrid<point_type> downSampleFilter;
		downSampleFilter.setInputCloud(surfPointsLessFlatScan);
		downSampleFilter.setLeafSize(0.2, 0.2, 0.2);
		downSampleFilter.filter(surfPointsLessFlatScanDS);

		/* std::cout << "DS SIZE: " << surfPointsLessFlatScanDS.points.size() << std::endl; */
		surfPointsLessFlat += surfPointsLessFlatScanDS;
	}
	/* std::cout << smoothness_coeff[0] << " " << *(smoothness_coeff.end()) << std::endl; */
	/* std::sort(smoothness_coeff.begin(), smoothness_coeff.end(), std::greater<float>()); */
	/* std::cout << smoothness_coeff[0] << " " << *(smoothness_coeff.end()) << std::endl; */

	std::cout << "CORNER: " <<  cornerPointsSharp.points.size() << std::endl;
	std::cout << "CORNER: " <<  cornerPointsLessSharp.points.size() << std::endl;
	std::cout << "SURF: " <<  surfPointsFlat.points.size() << std::endl;
	std::cout << "SURF: " <<  surfPointsLessFlat.points.size() << std::endl;

	std::cout << "TOTAL: " <<  laserCloud->points.size() << std::endl;
	std::cout << "REJECTED: " << rejectedPoints->points.size() << std::endl;

    /* boost::shared_ptr<pcl::visualization::PCLVisualizer> view( new pcl::visualization::PCLVisualizer( "Velodyne Viewer" ) ); */
    /* view->addCoordinateSystem( 3.0, "coordinate" ); */
    /* view->setBackgroundColor(0, 0, 0); */
    /* view->initCameraParameters(); */
    /* view->setCameraPosition( 0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0 ); */
	
	/* boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<point_type> > c_handler1(new pcl::visualization::PointCloudColorHandlerCustom<point_type>(100, 100, 100)); */
	/* boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<point_type> > c_handler2(new pcl::visualization::PointCloudColorHandlerCustom<point_type>(0, 255, 255)); */
	/* boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<point_type> > c_handler3(new pcl::visualization::PointCloudColorHandlerCustom<point_type>(255, 0, 255)); */


	/* c_handler1->setInputCloud(laserCloud); */
	/* view->addPointCloud(laserCloud, *c_handler1, "base"); */
	/* cloud_type::Ptr c1 = boost::make_shared<cloud_type>(surfPointsLessFlat); */
	/* c_handler3->setInputCloud(c1); */
	/* view->addPointCloud(c1, *c_handler3, "rejected"); */
	/* cloud_type::Ptr c2 = boost::make_shared<cloud_type>(surfPointsFlat); */
	/* c_handler2->setInputCloud(c2); */
	/* view->addPointCloud(c2, *c_handler2, "selected"); */
	/* view->spin(); */
}

void feature_extraction::preprocess_scans(cloud_type const& sourceCloud)
{
	/* float startOri = -atan2(sourceCloud.points[0].y, sourceCloud.points[0].x); */
	/* float endOri = -atan2(sourceCloud.points[cloud_size - 1].y, */
	/* 		sourceCloud.points[cloud_size - 1].x) + 2 * M_PI; */
	/* /1* float scanPeriod = 0.1; *1/ */

	/* if (endOri - startOri > 3 * M_PI) { */
	/* 	endOri -= 2 * M_PI; */
	/* } else if (endOri - startOri < M_PI) { */
	/* 	endOri += 2 * M_PI; */
	/* } */
	/* bool halfPassed = false; */
	/* int count = cloud_size; */
	/* point_type point; */
	/* std::vector<cloud_type> scans(N_SCANS); */
	/* for (int i = 0; i < cloud_size; i++) { */
	/* 	point.x = sourceCloud.points[i].y; */
	/* 	point.y = sourceCloud.points[i].z; */
	/* 	point.z = sourceCloud.points[i].x; */

	/* 	float angle = atan(point.y / sqrt(point.x * point.x + point.z * point.z)) * 180 / M_PI; */
	/* 	int scanID; */
	/* 	int roundedAngle = int(angle + (angle<0.0?-0.5:+0.5)); */ 
	/* 	if (roundedAngle > 0){ */
	/* 		scanID = roundedAngle; */
	/* 	} */
	/* 	else { */
	/* 		scanID = roundedAngle + (N_SCANS - 1); */
	/* 	} */
	/* 	if (scanID > (N_SCANS - 1) || scanID < 0 ){ */
	/* 		count--; */
	/* 		continue; */
	/* 	} */

	/* 	float ori = -atan2(point.x, point.z); */
	/* 	if (!halfPassed) { */
	/* 		if (ori < startOri - M_PI / 2) { */
	/* 			ori += 2 * M_PI; */
	/* 		} else if (ori > startOri + M_PI * 3 / 2) { */
	/* 			ori -= 2 * M_PI; */
	/* 		} */

	/* 		if (ori - startOri > M_PI) { */
	/* 			halfPassed = true; */
	/* 		} */
	/* 	} else { */
	/* 		ori += 2 * M_PI; */

	/* 		if (ori < endOri - M_PI * 3 / 2) { */
	/* 			ori += 2 * M_PI; */
	/* 		} else if (ori > endOri + M_PI / 2) { */
	/* 			ori -= 2 * M_PI; */
	/* 		} */ 
	/* 	} */

	/* 	float relTime = (ori - startOri) / (endOri - startOri); */
	/* 	point.intensity = scanID + scanPeriod * relTime; */
	/* 	scans[scanID].push_back(point); */
	/* } */
	/* cloud_size = count; */

	/* auto processedCloud = cloud_ptr(new cloud_type()); */

	/* for (int i = 0; i < N_SCANS; i++) { */
	/* 	*processedCloud += scans[i]; */
	/* } */
	/* std::cout << processedCloud->points.size() << std::endl; */
    /* boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer( new pcl::visualization::PCLVisualizer( "Velodyne Viewer" ) ); */
    /* viewer->addCoordinateSystem( 3.0, "coordinate" ); */
    /* viewer->setBackgroundColor( 0.0, 0.0, 0.0, 0 ); */
    /* viewer->initCameraParameters(); */
    /* viewer->setCameraPosition( 0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0 ); */

	/* boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<point_type>> color_handler( new pcl::visualization::PointCloudColorHandlerCustom<point_type>( 0, 255, 255) ); */

	/* color_handler->setInputCloud( processedCloud ); */
	/* viewer->addPointCloud( processedCloud, *color_handler, "processedCloud" ); */
	/* viewer->spin(); */
}

}
