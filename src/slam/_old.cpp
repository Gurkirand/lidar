
void vlp_features::something(cloud_type const& sourceCloud)
{
	const int N_SCANS = 16;
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
			std::cout << i << " Failed: " << scanID << std::endl;
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
		scans[scanID].push_back(point);
	}
	cloud_size = count;

	pcl::PointCloud<point_type>::Ptr laserCloud(new pcl::PointCloud<point_type>());

	for (int i = 0; i < N_SCANS; i++) {
		*laserCloud += scans[i];
	}
	int scanCount = -1;
	for (size_t i = subregion_size; i < cloud_size - subregion_size; i++)
	{
		float diffX = 0.0, diffY = 0.0, diffZ = 0.0;

		/*
		 * We want to calculate the size of the distance between the current point,
		 * laserCloud->points[i], and the points in its subregion. To avoid branching
		 * We just add up all points, including the current point, and then subtract
		 * the correct amount of the current point. We can just do addition since
		 * the sign will be lost when we find the square distance.
		 */
        for (int j = i - subregion_size; j <= i + subregion_size; j++)
        {
	        diffX += laserCloud->points[j].x;
	        diffY += laserCloud->points[j].y;
	        diffZ += laserCloud->points[j].z;
        }

        /*
         * We added 2 * subregion_size points around the current point and the
         * current point as well. Therefore we need to subtract 2 * subregion_size
         * + 1 from the coordinates.
         */
		int const diff_multiplier = 2 * subregion_size + 1;

        diffX -= laserCloud->points[i].x * diff_multiplier;
        diffY -= laserCloud->points[i].y * diff_multiplier;
        diffZ -= laserCloud->points[i].z * diff_multiplier;

        // Place coefficient in our member container
        attributes.smoothness_coeffs[i] = diffX*diffX + diffY*diffY + diffZ*diffZ; 
		attributes.neighbor_picked[i] = 0;
		attributes.sort_inds[i] = i;
		attributes.labels[i] = 0;

		if (int(laserCloud->points[i].intensity) != scanCount) {
			scanCount = int(laserCloud->points[i].intensity);

			if (scanCount > 0 && scanCount < N_SCANS) {
				scanStartInd[scanCount] = i + 5;
				scanEndInd[scanCount - 1] = i - 5;
			}
		}
	}

	for (int i = 0; i < N_SCANS; i++) {
		std::cout << "scanID: " << i << " start: " << scanStartInd[i] << " end: " << scanEndInd[i] << std::endl;
		for (int j = 0; j < 6; j++) {
			int sp = (scanStartInd[i] * (6 - j)  + scanEndInd[i] * j) / 6;
			int ep = (scanStartInd[i] * (5 - j)  + scanEndInd[i] * (j + 1)) / 6 - 1;
		}
	}

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
					attributes.neighbor_picked[i - 5] = 1;
					attributes.neighbor_picked[i - 4] = 1;
					attributes.neighbor_picked[i - 3] = 1;
					attributes.neighbor_picked[i - 2] = 1;
					attributes.neighbor_picked[i - 1] = 1;
					attributes.neighbor_picked[i] = 1;

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
					attributes.neighbor_picked[i + 1] = 1;
					attributes.neighbor_picked[i + 2] = 1;
					attributes.neighbor_picked[i + 3] = 1;
					attributes.neighbor_picked[i + 4] = 1;
					attributes.neighbor_picked[i + 5] = 1;
					attributes.neighbor_picked[i + 6] = 1;

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
			attributes.neighbor_picked[i] = 1;

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
					if (attributes.smoothness_coeffs[attributes.sort_inds[l]] < attributes.smoothness_coeffs[attributes.sort_inds[l - 1]]) {
						int temp = attributes.sort_inds[l - 1];
						attributes.sort_inds[l - 1] = attributes.sort_inds[l];
						attributes.sort_inds[l] = temp;
					}
				}
			}

			int largestPickedNum = 0;
			for (int k = ep; k >= sp; k--) {
				int ind = attributes.sort_inds[k];
				if (attributes.neighbor_picked[ind] == 0 &&
						attributes.smoothness_coeffs[ind] > 0.1) {

					largestPickedNum++;
					if (largestPickedNum <= 2) {
						attributes.labels[ind] = 2;
						cornerPointsSharp.push_back(laserCloud->points[ind]);
						cornerPointsLessSharp.push_back(laserCloud->points[ind]);
					} else if (largestPickedNum <= 20) {
						attributes.labels[ind] = 1;
						cornerPointsLessSharp.push_back(laserCloud->points[ind]);
					} else {
						break;
					}

					attributes.neighbor_picked[ind] = 1;
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

						attributes.neighbor_picked[ind + l] = 1;
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

						attributes.neighbor_picked[ind + l] = 1;
					}
				}
			}

			int smallestPickedNum = 0;
			for (int k = sp; k <= ep; k++) {
				int ind = attributes.sort_inds[k];
				if (attributes.neighbor_picked[ind] == 0 &&
						attributes.smoothness_coeffs[ind] < 0.1) {

					attributes.labels[ind] = -1;
					surfPointsFlat.push_back(laserCloud->points[ind]);

					smallestPickedNum++;
					if (smallestPickedNum >= 4) {
						break;
					}

					attributes.neighbor_picked[ind] = 1;
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

						attributes.neighbor_picked[ind + l] = 1;
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

						attributes.neighbor_picked[ind + l] = 1;
					}
				}
			}

			for (int k = sp; k <= ep; k++) {
				if (attributes.labels[k] <= 0) {
					surfPointsLessFlatScan->push_back(laserCloud->points[k]);
				}
			}
		}

		pcl::PointCloud<point_type> surfPointsLessFlatScanDS;
		pcl::VoxelGrid<point_type> downSizeFilter;
		downSizeFilter.setInputCloud(surfPointsLessFlatScan);
		downSizeFilter.setLeafSize(0.5, 0.5, 0.5);
		downSizeFilter.filter(surfPointsLessFlatScanDS);

		surfPointsLessFlat += surfPointsLessFlatScanDS;
	}
	std::cout << attributes.smoothness_coeffs[0] << " " << *(attributes.smoothness_coeffs.end()) << std::endl;
	std::sort(attributes.smoothness_coeffs.begin(), attributes.smoothness_coeffs.end(), std::greater<float>());
	std::cout << attributes.smoothness_coeffs[0] << " " << *(attributes.smoothness_coeffs.end()) << std::endl;

	std::cout << "CORNER: " <<  cornerPointsSharp.points.size() << std::endl;
	std::cout << "CORNER: " <<  cornerPointsLessSharp.points.size() << std::endl;
	std::cout << "SURF: " <<  surfPointsFlat.points.size() << std::endl;
	std::cout << "SURF: " <<  surfPointsLessFlat.points.size() << std::endl;

	std::cout << "TOTAL: " <<  laserCloud->points.size() << std::endl;
	std::cout << "REJECTED: " << rejectedPoints->points.size() << std::endl;

    boost::shared_ptr<pcl::visualization::PCLVisualizer> view( new pcl::visualization::PCLVisualizer( "Velodyne Viewer" ) );
    view->addCoordinateSystem( 3.0, "coordinate" );
    view->setBackgroundColor(0, 0, 0);
    view->initCameraParameters();
    view->setCameraPosition( 0.0, 0.0, 30.0, 0.0, 1.0, 0.0, 0 );
	
	boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<point_type> > c_handler1(new pcl::visualization::PointCloudColorHandlerCustom<point_type>(100, 100, 100));
	boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<point_type> > c_handler2(new pcl::visualization::PointCloudColorHandlerCustom<point_type>(0, 255, 255));
	boost::shared_ptr<pcl::visualization::PointCloudColorHandlerCustom<point_type> > c_handler3(new pcl::visualization::PointCloudColorHandlerCustom<point_type>(255, 0, 255));


	c_handler1->setInputCloud(laserCloud);
	view->addPointCloud(laserCloud, *c_handler1, "base");
	cloud_type::Ptr c1 = boost::make_shared<cloud_type>(surfPointsLessFlat);
	c_handler3->setInputCloud(c1);
	view->addPointCloud(c1, *c_handler3, "rejected");
	cloud_type::Ptr c2 = boost::make_shared<cloud_type>(surfPointsFlat);
	c_handler2->setInputCloud(c2);
	view->addPointCloud(c2, *c_handler2, "selected");
	view->spin();
}
