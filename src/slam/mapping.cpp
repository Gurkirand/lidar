#include "../../include/slam/mapping.h"
#include "../../include/slam/math_utils.h"
#include <Eigen/Eigenvalues>
#include <Eigen/QR>

#include <cmath>

namespace slam
{
/* void mapping::update_transform() */
/* { */
/* 	transformBefMapped = transformSum; */
/* 	transformAftMapped = transformTobeMapped; */
/* } */

mapping::mapping()
	: laserCloudCornerStack(new cloud_type),
	laserCloudSurfStack(new cloud_type),
	laserCloudCornerStack2(new cloud_type),
	laserCloudSurfStack2(new cloud_type),
	laserCloudOri(new cloud_type),
	coeffSel(new cloud_type),
	laserCloudSurround(new cloud_type),
	laserCloudSurround2(new cloud_type),
	laserCloudCornerFromMap(new cloud_type),
	laserCloudSurfFromMap(new cloud_type),
	kdtreeCornerFromMap(new kdtree_type),
	kdtreeSurfFromMap(new kdtree_type)
{

	downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
	downSizeFilterSurf.setLeafSize(0.4, 0.4, 0.4);
	downSizeFilterMap.setLeafSize(0.6, 0.6, 0.6);
}

void mapping::process (
		CloudScans& sourceCloud,
		ScanFeatures const& sourceFeats,
		Transform& odometryTransform,
		Quaternion& odometryQuat)
{
	init();

	source_scans = &sourceCloud;
	source_features = &sourceFeats;

	transformSum = odometryTransform;


	if (++frameCount < stackFrameNum)
	{
		return;
	}

	frameCount = 0;

	///SET VARIABLES

	transform_associate_to_map();
	stack_features();
	segment_cloud();
	//Maybe can move above to stackfeatures / include in method
	downsample_last();

	/* if (map_features.edge_cloud->points.size() > 10 */ 
	/* 		&& map_features.surface_cloud->points.size() > 100) */
	if (laserCloudCornerFromMap->points.size() > 10
			&& laserCloudSurfFromMap->points.size() > 100)
	{
		find_transform();
	}
	
	store_downsamples();

	publish();
}

void mapping::init()
{
	laserCloudValidNum = 0;
	laserCloudSurroundNum = 0;
	for (int i = 0; i < laserCloudNum; i++)
	{
		laserCloudCornerArray[i].reset(new pcl::PointCloud<point_type>());
		laserCloudSurfArray[i].reset(new pcl::PointCloud<point_type>());
		laserCloudCornerArray2[i].reset(new pcl::PointCloud<point_type>());
		laserCloudSurfArray2[i].reset(new pcl::PointCloud<point_type>());
	}
}

void mapping::stack_features()
{

	cloud_type& edges = *(source_features->edge_scans.cloud);
	size_t edgeSize = edges.points.size();
	for (int i = 0; i < edgeSize; i++)
	{
		point_type pointSel;
		point_associate_to_map(edges.points[i], pointSel);
		laserCloudCornerStack2->push_back(pointSel);
	}

	cloud_type& surfaces = *(source_features->surface_scans.cloud);
	size_t surfaceSize = surfaces.points.size();
	for (int i = 0; i < surfaceSize; i++)
	{
		point_type pointSel;
		point_associate_to_map(surfaces.points[i], pointSel);
		laserCloudSurfStack2->push_back(pointSel);
	}
	/* cloud_type& edges = *(source_features->edge_scans.cloud); */
	/* cloud_type& edgeStack = *(features_stack.edge_cloud); */
	/* size_t edgeSize = edges.points.size(); */
	/* for (int i = 0; i < edgeSize; i++) { */
	/* 	point_type pointSel; */
	/* 	point_associate_to_map(edges.points[i], pointSel); */
	/* 	edgeStack.push_back(pointSel); */
	/* } */

	/* cloud_type& surfaces = *(source_features->surface_scans.cloud); */
	/* cloud_type& surfaceStack = *(features_stack.edge_cloud); */
	/* size_t surfaceSize = surfaces.points.size(); */
	/* for (int i = 0; i < surfaceSize; i++) { */
	/* 	point_type pointSel; */
	/* 	point_associate_to_map(surfaces.points[i], pointSel); */
	/* 	surfaceStack.push_back(pointSel); */
	/* } */
}

void mapping::segment_cloud_along_axis (
		int ref_len,
		int len2,
		int len3,
		int& center,
		int& ref_center_len,
		index_func const& index_calc)
{
	while (center < 3)
	{
		for (int j = 0; j < len2; j++)
		{
			for (int k = 0; k < len3; k++)
			{
				int i = ref_len - 1;
				size_t index = index_calc(i, j, k);
	
				/* CloudFeatures segmentFeatures = segment_array[index]; */
				cloud_ptr laserCloudCubeCornerPointer = laserCloudCornerArray[index];
				cloud_ptr laserCloudCubeSurfPointer = laserCloudSurfArray[index];

				for (; i >= 1; i--) 
				{
					size_t index2 = index_calc(i - 1, j, k);
					/* segment_array[index2] = segment_array[index2 - 1]; */
					laserCloudCornerArray[index2] = laserCloudCornerArray[index2 - 1];
					laserCloudSurfArray[index2] = laserCloudSurfArray[index2 - 1];
				}

				/* segment_array[index] = segmentFeatures; */
				laserCloudCornerArray[index] = laserCloudCubeCornerPointer;
				laserCloudSurfArray[index] = laserCloudCubeSurfPointer;

				//CHECK TO SEE WHY THEY CLEAR AND IF THIS WORKS
				/* segmentFeatures.clear(); */
				laserCloudCubeCornerPointer->clear();
				laserCloudCubeSurfPointer->clear();
			}
		}

		center++;
		ref_center_len++;
	}

	while (center >= ref_len - 3)
	{
		for (int j = 0; j < len2; j++)
		{
			for (int k = 0; k < len3; k++)
			{
				int i = 0;
				int index = index_calc(i, j, k);

				/* CloudFeatures segmentFeatures = segment_array[index]; */
				cloud_ptr laserCloudCubeCornerPointer = laserCloudCornerArray[index];
				cloud_ptr laserCloudCubeSurfPointer = laserCloudSurfArray[index];

				for (; i < ref_len - 1; i++)
				{
					int index2 = index_calc(i + 1, j, k);
					/* segment_array[index2] = segment_array[index2 + 1]; */
					laserCloudCornerArray[index2] = laserCloudCornerArray[index2 + 1];
					laserCloudSurfArray[index2] = laserCloudSurfArray[index2 + 1];
				}

				/* segment_array[index] = segmentFeatures; */
				laserCloudCornerArray[index] = laserCloudCubeCornerPointer;
				laserCloudSurfArray[index] = laserCloudCubeSurfPointer;
				/* segmentFeatures.clear(); */
				laserCloudCubeCornerPointer->clear();
				laserCloudCubeSurfPointer->clear();
			}
		}

		center--;
		ref_center_len--;
	}
}

int mapping::segmented_cloud_index (int width, int height, int depth) const
{
	return width + laserCloudWidth * height + laserCloudWidth * laserCloudHeight * depth;
}

/* int mapping::segmented_cloud_index_HWD (int height, int width, int depth) */
/* { */
/* 	return segmented_cloud_index_WHD(width, height, depth); */
/* } */

/* int mapping::segmented_cloud_index_DWH (int depth, int width, int height) */
/* { */
/* 	return segmented_cloud_index_WHD(width, height, depth); */
/* } */

void mapping::find_valid_segments (int centerCubeW, int centerCubeH, int centerCubeD)
{
	laserCloudValidNum = 0;
	laserCloudSurroundNum = 0;
	point_type pointOnYAxis;
	pointOnYAxis.x = 0.0;
	pointOnYAxis.y = 10.0;
	pointOnYAxis.z = 0.0;
	point_associate_to_map(pointOnYAxis, pointOnYAxis);

	for (int i = centerCubeW - 2; i <= centerCubeW + 2; i++)
	{
		for (int j = centerCubeH - 2; j <= centerCubeH + 2; j++)
		{
			for (int k = centerCubeD - 2; k <= centerCubeD + 2; k++)
			{
				if (i >= 0 && i < laserCloudWidth && 
						j >= 0 && j < laserCloudHeight && 
						k >= 0 && k < laserCloudDepth)
				{

					float centerX = 50.0 * (i - laserCloudCenWidth);
					float centerY = 50.0 * (j - laserCloudCenHeight);
					float centerZ = 50.0 * (k - laserCloudCenDepth);

					bool isInLaserFOV = false;
					for (int ii = -1; ii <= 1; ii += 2)
					{
						for (int jj = -1; jj <= 1; jj += 2)
						{
							for (int kk = -1; kk <= 1; kk += 2)
							{
								Vector3 corner;
								corner.x() = centerX + 25.0 * ii;
								corner.y() = centerY + 25.0 * jj;
								corner.z() = centerZ + 25.0 * kk;

								Vector3 point_on_axis( pointOnYAxis.x, pointOnYAxis.y, pointOnYAxis.z);
								float squaredSide1 = (transformTobeMapped.translation - corner).squaredNorm();
								float squaredSide2 = (point_on_axis - corner).squaredNorm();

								float check1 = 100.0 + squaredSide1 - squaredSide2
									- 10.0 * sqrt(3.0) * sqrt(squaredSide1);

								float check2 = 100.0 + squaredSide1 - squaredSide2
									+ 10.0 * sqrt(3.0) * sqrt(squaredSide1);

								if (check1 < 0 && check2 > 0) {
									isInLaserFOV = true;
								}
							}
						}
					}

					int index = segmented_cloud_index_WHD(i, j, k);

					if (isInLaserFOV)
					{
						laserCloudValidInd[laserCloudValidNum] = index;
						laserCloudValidNum++;
					}
					laserCloudSurroundInd[laserCloudSurroundNum] = index;
					laserCloudSurroundNum++;
				}
			}
		}
	}
}


void mapping::segment_cloud ()
{
	int centerCubeW = int((transformTobeMapped.translation.x() + 25.0) / 50.0) + laserCloudCenWidth;
	int centerCubeH = int((transformTobeMapped.translation.y() + 25.0) / 50.0) + laserCloudCenHeight;
	int centerCubeD = int((transformTobeMapped.translation.z() + 25.0) / 50.0) + laserCloudCenDepth;

	if (transformTobeMapped.translation.x() + 25.0 < 0) centerCubeW--;
	if (transformTobeMapped.translation.y() + 25.0 < 0) centerCubeH--;
	if (transformTobeMapped.translation.z() + 25.0 < 0) centerCubeD--;

	segment_cloud_along_axis(laserCloudWidth, laserCloudHeight, laserCloudDepth,
			centerCubeW, laserCloudCenWidth, segmented_cloud_index_WHD);

	segment_cloud_along_axis(laserCloudHeight, laserCloudWidth, laserCloudDepth,
			centerCubeH, laserCloudCenHeight, segmented_cloud_index_HWD);

	segment_cloud_along_axis(laserCloudDepth, laserCloudHeight, laserCloudWidth,
			centerCubeD, laserCloudCenDepth, segmented_cloud_index_DWH);

	find_valid_segments(centerCubeW, centerCubeH, centerCubeD);

	// maybe move to reset
	laserCloudCornerFromMap->clear();
	laserCloudSurfFromMap->clear();
	for (int i = 0; i < laserCloudValidNum; i++)
	{
		*laserCloudCornerFromMap += *laserCloudCornerArray[laserCloudValidInd[i]];
		*laserCloudSurfFromMap += *laserCloudSurfArray[laserCloudValidInd[i]];
	}
}

void mapping::downsample_last ()
{
	int laserCloudCornerStack2Num = laserCloudCornerStack2->points.size();
	for (int i = 0; i < laserCloudCornerStack2Num; i++) {
		point_associate_to_be_mapped(laserCloudCornerStack2->points[i], laserCloudCornerStack2->points[i]);
	}

	int laserCloudSurfStack2Num = laserCloudSurfStack2->points.size();
	for (int i = 0; i < laserCloudSurfStack2Num; i++) {
		point_associate_to_be_mapped(laserCloudSurfStack2->points[i], laserCloudSurfStack2->points[i]);
	}

	laserCloudCornerStack->clear();
	downSizeFilterCorner.setInputCloud(laserCloudCornerStack2);
	downSizeFilterCorner.filter(*laserCloudCornerStack);

	laserCloudSurfStack->clear();
	downSizeFilterSurf.setInputCloud(laserCloudSurfStack2);
	downSizeFilterSurf.filter(*laserCloudSurfStack);

	laserCloudCornerStack2->clear();
	laserCloudSurfStack2->clear();
}

void mapping::edge_point_transforms ()
{
	Eigen::Matrix3f matA1;
	Eigen::Matrix<float, 1, 3> matD1;
	Eigen::Matrix3f matV1;

	matA1.setZero();
	matD1.setZero();
	matV1.setZero();

	int laserCloudCornerStackNum = laserCloudCornerStack->points.size();

	for (int i = 0; i < laserCloudCornerStackNum; i++)
	{
		point_type pointOri = laserCloudCornerStack->points[i],
			pointSel;
		point_associate_to_map(pointOri, pointSel);
		kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

		if (pointSearchSqDis[4] < 1.0)
		{
			Vector3 vc(0,0,0);

			for (int j = 0; j < 5; j++)
			{
				vc += laserCloudCornerFromMap->points[pointSearchInd[j]];
			}
			vc /= 5.0;

			Eigen::Matrix3f mat_a;
			mat_a.setZero();

			for (int j = 0; j < 5; j++)
			{
				int index = pointSearchInd[j];
				Vector3 vc_sub = -1 * vc + laserCloudCornerFromMap->points[pointSearchInd[j]];

				mat_a(0,0) += vc_sub.x() * vc_sub.x();
				mat_a(0,1) += vc_sub.x() * vc_sub.y();
				mat_a(0,2) += vc_sub.x() * vc_sub.z();
				mat_a(1,1) += vc_sub.y() * vc_sub.y();
				mat_a(1,2) += vc_sub.y() * vc_sub.z();
				mat_a(2,2) += vc_sub.z() * vc_sub.z();
			}
			matA1 = mat_a / 5.0;

			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> esolver(matA1);
			matD1 = esolver.eigenvalues().real();
			matV1 = esolver.eigenvectors().real();

			if (matD1(0, 0) > 3 * matD1(0, 1))
			{

				Eigen::Matrix3f const matV1_scaled = 0.1 * matV1;
				slam::Vector3 p1(vc.x() + matV1_scaled(0, 0), vc.y() + matV1_scaled(0, 1), vc.z() + matV1_scaled(0,2));
				slam::Vector3 p2(vc.x() - matV1_scaled(0, 0), vc.y() - matV1_scaled(0, 1), vc.z() - matV1_scaled(0,2));

				point_type normal = cross_product(diff(pointSel, p1), diff(pointSel, p2));

				point_type base = diff(p1, p2);

				float area = length(normal);

				float baseLength = length(base);

				float heightLength = area * baseLength;

				point_type pointProj = scale(cross_product(base, normal), 1 / heightLength);

				float ld2 = area / baseLength;

				float s = 1 - 0.9 * fabs(ld2);

				point_type coeff;
				coeff.x = s * pointProj.x;
				coeff.y = s * pointProj.y;
				coeff.z = s * pointProj.z;
				coeff.intensity = s * ld2;

				if (s > 0.1)
				{
					laserCloudOri->push_back(pointOri);
					coeffSel->push_back(coeff);
				}
			}
		}
	}
}

void mapping::surface_point_transforms ()
{
	Eigen::Matrix<float, 5, 3> matA0;
	Eigen::Matrix<float, 5, 1> matB0;
	Eigen::Vector3f matX0;

	matA0.setZero();
	matB0.setConstant(-1);
	matX0.setZero();

	int laserCloudSurfStackNum = laserCloudSurfStack->points.size();

	for (int i = 0; i < laserCloudSurfStackNum; i++)
	{
		point_type pointOri = laserCloudSurfStack->points[i],
			pointSel;
		point_associate_to_map(pointOri, pointSel); 
		kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

		if (pointSearchSqDis[4] < 1.0)
		{
			int index;

			for (int j = 0; j < 5; j++)
			{
				index = pointSearchInd[j];
				matA0(j, 0) = laserCloudSurfFromMap->points[index].x;
				matA0(j, 1) = laserCloudSurfFromMap->points[index].y;
				matA0(j, 2) = laserCloudSurfFromMap->points[index].z;
			}
			matX0 = matA0.colPivHouseholderQr().solve(matB0);

			/* float pa = matX0(0, 0); */
			/* float pb = matX0(1, 0); */
			/* float pc = matX0(2, 0); */
			/* float pd = 1; */

			/* float ps = sqrt(pa * pa + pb * pb + pc * pc); */
			/* pa /= ps; */
			/* pb /= ps; */
			/* pc /= ps; */
			/* pd /= ps; */

			point_type pointEigen = point(matX0(0, 0), matX0(1, 0), matX0(2, 0), 1);
			normalize(pointEigen);

			bool planeValid = true;
			for (int j = 0; j < 5; j++)
			{
				int mult = fabs(dot_product(pointEigen, laserCloudSurfFromMap->points[pointSearchInd[j]])) + pointEigen.intensity;
				if (mult > 0.2)
				{
					planeValid = false;
					break;
				}
			}

			if (planeValid)
			{
				float pd2 = dot_product(pointEigen, pointSel) + pointEigen.intensity;
				point_type pointProj = diff(pointSel, scale(pointEigen, pd2));

				/* float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd; */

				/* pointProj = pointSel; */
				/* pointProj.x -= pa * pd2; */
				/* pointProj.y -= pb * pd2; */
				/* pointProj.z -= pc * pd2; */

				/* float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x */
				/* 			+ pointSel.y * pointSel.y + pointSel.z * pointSel.z)); */
				float s = 1 - 0.9 * fabs(pd2) / sqrt(length(pointSel));

				point_type coeff;

				coeff.x = s * pointProj.x;
				coeff.y = s * pointProj.y;
				coeff.z = s * pointProj.z;
				coeff.intensity = s * pd2;

				if (s > 0.1)
				{
					laserCloudOri->push_back(pointOri);
					coeffSel->push_back(coeff);
				}
			}
		}
	}
}

void mapping::stack_jacobian (
		Eigen::Matrix<float, Eigen::Dynamic, 6>& matA,
		Eigen::VectorXf& matB)
{
	float sinrx = transformTobeMapped.rotation.x.sin();
	float sinry = transformTobeMapped.rotation.y.sin();
	float sinrz = transformTobeMapped.rotation.z.sin();
	float cosrx = transformTobeMapped.rotation.x.cos();
	float cosry = transformTobeMapped.rotation.y.cos();
	float cosrz = transformTobeMapped.rotation.z.cos();

	int laserCloudSelNum = laserCloudOri->points.size();

	for (int i = 0; i < laserCloudSelNum; i++)
	{
		point_type pointOri = laserCloudOri->points[i];
		point_type coeff = coeffSel->points[i];

		float arx = (cosrx*sinry*sinrz*pointOri.x + cosrx*cosrz*sinry*pointOri.y - sinrx*sinry*pointOri.z) * coeff.x
			+ (-sinrx*sinrz*pointOri.x - cosrz*sinrx*pointOri.y - cosrx*pointOri.z) * coeff.y
			+ (cosrx*cosry*sinrz*pointOri.x + cosrx*cosry*cosrz*pointOri.y - cosry*sinrx*pointOri.z) * coeff.z;

		float ary = ((cosry*sinrx*sinrz - cosrz*sinry)*pointOri.x 
				+ (sinry*sinrz + cosry*cosrz*sinrx)*pointOri.y + cosrx*cosry*pointOri.z) * coeff.x
			+ ((-cosry*cosrz - sinrx*sinry*sinrz)*pointOri.x 
					+ (cosry*sinrz - cosrz*sinrx*sinry)*pointOri.y - cosrx*sinry*pointOri.z) * coeff.z;

		float arz = ((cosrz*sinrx*sinry - cosry*sinrz)*pointOri.x + (-cosry*cosrz-sinrx*sinry*sinrz)*pointOri.y)*coeff.x
			+ (cosrx*cosrz*pointOri.x - cosrx*sinrz*pointOri.y) * coeff.y
			+ ((sinry*sinrz + cosry*cosrz*sinrx)*pointOri.x + (cosrz*sinry-cosry*sinrx*sinrz)*pointOri.y)*coeff.z;

		matA(i, 0) = arx;
		matA(i, 1) = ary;
		matA(i, 2) = arz;
		matA(i, 3) = coeff.x;
		matA(i, 4) = coeff.y;
		matA(i, 5) = coeff.z;
		matB(i, 0) = -coeff.intensity;
	}
}

bool mapping::pose_transform()
{
	bool isDegenerate = false;

	int laserCloudSelNum = laserCloudOri->points.size();

	Eigen::Matrix<float,Eigen::Dynamic,6> matA(laserCloudSelNum, 6);
	Eigen::Matrix<float,6, Eigen::Dynamic> matAt(6, laserCloudSelNum);
	Eigen::Matrix<float,6, 6> matAtA;
	Eigen::VectorXf matB(laserCloudSelNum);
	Eigen::VectorXf matAtB;
	Eigen::VectorXf matX;
	Eigen::Matrix<float, 6, 6> matP;


	stack_jacobian(matA, matB);

	matAt = matA.transpose();
	matAtA = matAt * matA;
	matAtB = matAt * matB;
	matX = matAtA.colPivHouseholderQr().solve(matAtB);

	if (iterCount == 0)
	{
		Eigen::Matrix<float,1, 6> matE;
		Eigen::Matrix<float,6, 6> matV;
		Eigen::Matrix<float,6, 6> matV2;

		Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float,6, 6> > esolver(matAtA);
		matE = esolver.eigenvalues().real();
		matV = esolver.eigenvectors().real();

		matV2 = matV;

		isDegenerate = false;
		float eignThre[6] =
		{100, 100, 100, 100, 100, 100};
		for (int i = 5; i >= 0; i--)
		{
			if (matE(0, i) < eignThre[i])
			{
				for (int j = 0; j < 6; j++)
				{
					matV2(i, j) = 0;
				}
				isDegenerate = true;
			} else
			{
				break;
			}
		}
		matP = matV.inverse() * matV2;
	}

	if (isDegenerate)
	{
		Eigen::Matrix<float,6, 1> matX2(matX);
		matX = matP * matX2;
	}

	transformTobeMapped.rotation.x += matX(0, 0);
	transformTobeMapped.rotation.y += matX(1, 0);
	transformTobeMapped.rotation.z += matX(2, 0);
	transformTobeMapped.translation.x() += matX(3, 0);
	transformTobeMapped.translation.y() += matX(4, 0);
	transformTobeMapped.translation.z() += matX(5, 0);

	float deltaR = sqrt(
			pow(rad2deg(matX(0, 0)), 2) +
			pow(rad2deg(matX(1, 0)), 2) +
			pow(rad2deg(matX(2, 0)), 2));
	float deltaT = sqrt(
			pow(matX(3, 0) * 100, 2) +
			pow(matX(4, 0) * 100, 2) +
			pow(matX(5, 0) * 100, 2));

	if (deltaR < 0.05 && deltaT < 0.05)
	{
		return true;
	}
	return false;
}

void mapping::find_transform ()
{
	kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMap);
	kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMap);

	for (; iterCount < 10; iterCount++)
	{
		laserCloudOri->clear();
		coeffSel->clear();

		edge_point_transforms(); 
		surface_point_transforms();

		//stackJacobian etc.
		if (laserCloudOri->points.size() < 50)
		{
			continue;
		}


		if (pose_transform())
		{
			break;
		}
	}

	transformBefMapped = transformSum;
	transformAftMapped = transformTobeMapped;
}

void mapping::store_downsamples ()
{
	int laserCloudCornerStackNum = laserCloudCornerStack->points.size();
	for (int i = 0; i < laserCloudCornerStackNum; i++) {
		point_type pointSel;
		point_associate_to_map(laserCloudCornerStack->points[i], pointSel);

		int cubeW = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
		int cubeH = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
		int cubeD = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

		if (pointSel.x + 25.0 < 0) cubeW--;
		if (pointSel.y + 25.0 < 0) cubeH--;
		if (pointSel.z + 25.0 < 0) cubeD--;

		if (cubeW >= 0 && cubeW < laserCloudWidth && 
				cubeH >= 0 && cubeH < laserCloudHeight && 
				cubeD >= 0 && cubeD < laserCloudDepth) {
			int index = segmented_cloud_index_WHD(cubeW, cubeH, cubeD);
			laserCloudCornerArray[index]->push_back(pointSel);
		}
	}

	int laserCloudSurfStackNum = laserCloudSurfStack->points.size();
	for (int i = 0; i < laserCloudSurfStackNum; i++) {
		point_type pointSel;
		point_associate_to_map(laserCloudSurfStack->points[i], pointSel);

		int cubeW = int((pointSel.x + 25.0) / 50.0) + laserCloudCenWidth;
		int cubeH = int((pointSel.y + 25.0) / 50.0) + laserCloudCenHeight;
		int cubeD = int((pointSel.z + 25.0) / 50.0) + laserCloudCenDepth;

		if (pointSel.x + 25.0 < 0) cubeW--;
		if (pointSel.y + 25.0 < 0) cubeH--;
		if (pointSel.z + 25.0 < 0) cubeD--;

		if (cubeW >= 0 && cubeW < laserCloudWidth && 
				cubeH >= 0 && cubeH < laserCloudHeight && 
				cubeD >= 0 && cubeD < laserCloudDepth) {
			int index = segmented_cloud_index_WHD(cubeW, cubeH, cubeD);
			laserCloudSurfArray[index]->push_back(pointSel);
		}
	}

	for (int i = 0; i < laserCloudValidNum; i++) {
		int ind = laserCloudValidInd[i];

		laserCloudCornerArray2[ind]->clear();
		downSizeFilterCorner.setInputCloud(laserCloudCornerArray[ind]);
		downSizeFilterCorner.filter(*laserCloudCornerArray2[ind]);

		laserCloudSurfArray2[ind]->clear();
		downSizeFilterSurf.setInputCloud(laserCloudSurfArray[ind]);
		downSizeFilterSurf.filter(*laserCloudSurfArray2[ind]);

		cloud_ptr laserCloudTemp = laserCloudCornerArray[ind];
		laserCloudCornerArray[ind] = laserCloudCornerArray2[ind];
		laserCloudCornerArray2[ind] = laserCloudTemp;

		laserCloudTemp = laserCloudSurfArray[ind];
		laserCloudSurfArray[ind] = laserCloudSurfArray2[ind];
		laserCloudSurfArray2[ind] = laserCloudTemp;
	}
}

void mapping::publish ()
{
	mapFrameCount++;
	if (mapFrameCount >= mapFrameNum) {
		mapFrameCount = 0;

		laserCloudSurround2->clear();
		for (int i = 0; i < laserCloudSurroundNum; i++) {
			int ind = laserCloudSurroundInd[i];
			*laserCloudSurround2 += *laserCloudCornerArray[ind];
			*laserCloudSurround2 += *laserCloudSurfArray[ind];
		}

		laserCloudSurround->clear();
		downSizeFilterCorner.setInputCloud(laserCloudSurround2);
		downSizeFilterCorner.filter(*laserCloudSurround);

		/* sensor_msgs::PointCloud2 laserCloudSurround3; */
		/* pcl::toROSMsg(*laserCloudSurround, laserCloudSurround3); */
		/* laserCloudSurround3.header.stamp = ros::Time().fromSec(timeLaserOdometry); */
		/* laserCloudSurround3.header.frame_id = "/camera_init"; */
		/* pubLaserCloudSurround.publish(laserCloudSurround3); */
	}

	cloud_type& source_cloud = *(source_scans->cloud);
	int cloudSize = source_cloud.points.size();
	for (int i = 0; i < cloudSize; i++) {
		point_associate_to_map(source_cloud.points[i], source_cloud.points[i]);
	}

	/* sensor_msgs::PointCloud2 laserCloudFullRes3; */
	/* pcl::toROSMsg(*laserCloudFullRes, laserCloudFullRes3); */
	/* laserCloudFullRes3.header.stamp = ros::Time().fromSec(timeLaserOdometry); */
	/* laserCloudFullRes3.header.frame_id = "/camera_init"; */
	/* pubLaserCloudFullRes.publish(laserCloudFullRes3); */

	geoQuat = make_quaternion_rpy(
			transformAftMapped.rotation.z.rad(),
			transformAftMapped.rotation.x.rad(),
			transformAftMapped.rotation.y.rad());


	/* geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw */
	/* 	( transformAftMapped.rotation.z, */
	/* 	  -transformAftMapped.rotation.x, */
	/* 	  -transformAftMapped.rotation.y); */

	/* odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry); */
	/* odomAftMapped.pose.pose.orientation.x = -geoQuat.y; */
	/* odomAftMapped.pose.pose.orientation.y = -geoQuat.z; */
	/* odomAftMapped.pose.pose.orientation.z = geoQuat.x; */
	/* odomAftMapped.pose.pose.orientation.w = geoQuat.w; */
	/* odomAftMapped.pose.pose.position.x = transformAftMapped.pos.x(); */
	/* odomAftMapped.pose.pose.position.y = transformAftMapped.pos.y(); */
	/* odomAftMapped.pose.pose.position.z = transformAftMapped.pos.z(); */
	/* odomAftMapped.twist.twist.angular.x = transformBefMapped.rotation.x; */
	/* odomAftMapped.twist.twist.angular.y = transformBefMapped.rotation.y; */
	/* odomAftMapped.twist.twist.angular.z = transformBefMapped.rotation.z; */
	/* odomAftMapped.twist.twist.linear.x = transformBefMapped.translation.x(); */
	/* odomAftMapped.twist.twist.linear.y = transformBefMapped.translation.y(); */
	/* odomAftMapped.twist.twist.linear.z = transformBefMapped.translation.z(); */
	/* pubOdomAftMapped.publish(odomAftMapped); */

	/* aftMappedTrans.stamp_ = ros::Time().fromSec(timeLaserOdometry); */
	/* aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w)); */
	/* aftMappedTrans.setOrigin(tf::Vector3(transformAftMapped.translation.x(), */
	/* 			transformAftMapped.translation.y(), */
	/* 			transformAftMapped.translation.z())); */
	/* tfBroadcaster.sendTransform(aftMappedTrans); */

}



void mapping::transform_associate_to_map()
{

	// Essentially find angles from combined (multiplied) rotation matrices from the three different rotations
	//
	// Since the rotation matrix looks like this:
	/* Rotation Matrix:
	   [ cosry*cosrz + sinrx*sinry*sinrz,  -sinrz*cosry + cosrz*sinrx*sinry, cosrx*sinry]
	   [                  cosrx*sinrz,                    cosrx*cosrz,     -sinrx]
	   [-sinry*cosrz + cosry*sinrx*sinrz, -sinry*-sinrz + cosry*cosrz*sinrx, cosrx*cosry]
	   */ 
	// We can find sinx by multiplying the rotation matrix and just taking the element in position [2][3]
	// The find other angles using it
	//
	// The rotation matrix is found by multiplying R3 * R2.T * R1 
	// 

	transformIncre.translation = transformBefMapped.translation - transformSum.translation;
	rotateYXZ(transformIncre.translation, transformSum.rotation * -1);

	float sin1x = transformSum.rotation.x.sin();
	float cos1x = transformSum.rotation.x.cos();
	float sin1y = transformSum.rotation.y.sin();
	float cos1y = transformSum.rotation.y.cos();
	float sin1z = transformSum.rotation.z.sin();
	float cos1z = transformSum.rotation.z.cos();

	float sin2x = transformBefMapped.rotation.x.sin();
	float cos2x = transformBefMapped.rotation.x.cos();
	float sin2y = transformBefMapped.rotation.y.sin();
	float cos2y = transformBefMapped.rotation.y.cos();
	float sin2z = transformBefMapped.rotation.z.sin();
	float cos2z = transformBefMapped.rotation.z.cos();

	float sin3x = transformAftMapped.rotation.x.sin();
	float cos3x = transformAftMapped.rotation.x.cos();
	float sin3y = transformAftMapped.rotation.y.sin();
	float cos3y = transformAftMapped.rotation.y.cos();
	float sin3z = transformAftMapped.rotation.z.sin();
	float cos3z = transformAftMapped.rotation.z.cos();

	float srx = -sin1x * (sin3x * sin2x + cos3x * cos2x * sin3z * sin2z + cos3x * cos3z * cos2x * cos2z)
		- cos1x * sin1y * (cos3x * cos3z * (cos2y * sin2z - cos2z * sin2x * sin2y)
				- cos3x * sin3z * (cos2y * cos2z + sin2x * sin2y * sin2z) + cos2x * sin3x * sin2y)
		- cos1x * cos1y * (cos3x * sin3z * (cos2z * sin2y - cos2y * sin2x * sin2z) 
				- cos3x * cos3z * (sin2y * sin2z + cos2y * cos2z * sin2x) + cos2x * cos2y * sin3x);
	transformTobeMapped.rotation.x = -asin(srx);

	float srycrx = sin1x * (cos2x * cos2z * (cos3y * sin3z - cos3z * sin3x * sin3y)
			- cos2x * sin2z * (cos3y * cos3z + sin3x * sin3y * sin3z) + cos3x * sin3y * sin2x)
		- cos1x * cos1y * ((cos3y * cos3z + sin3x * sin3y * sin3z) * (cos2z * sin2y - cos2y * sin2x * sin2z)
				+ (cos3y * sin3z - cos3z * sin3x * sin3y) * (sin2y * sin2z + cos2y * cos2z * sin2x) - cos3x * cos2x * cos2y * sin3y)
		+ cos1x * sin1y * ((cos3y * cos3z + sin3x * sin3y * sin3z) * (cos2y * cos2z + sin2x * sin2y * sin2z)
				+ (cos3y * sin3z - cos3z * sin3x * sin3y) * (cos2y * sin2z - cos2z * sin2x * sin2y) + cos3x * cos2x * sin3y * sin2y);
	float crycrx = sin1x * (cos2x * sin2z * (cos3z * sin3y - cos3y * sin3x * sin3z)
			- cos2x * cos2z * (sin3y * sin3z + cos3y * cos3z * sin3x) + cos3x * cos3y * sin2x)
		+ cos1x * cos1y * ((sin3y * sin3z + cos3y * cos3z * sin3x) * (sin2y * sin2z + cos2y * cos2z * sin2x)
				+ (cos3z * sin3y - cos3y * sin3x * sin3z) * (cos2z * sin2y - cos2y * sin2x * sin2z) + cos3x * cos3y * cos2x * cos2y)
		- cos1x * sin1y * ((sin3y * sin3z + cos3y * cos3z * sin3x) * (cos2y * sin2z - cos2z * sin2x * sin2y)
				+ (cos3z * sin3y - cos3y * sin3x * sin3z) * (cos2y * cos2z + sin2x * sin2y * sin2z) - cos3x * cos3y * cos2x * sin2y);
	transformTobeMapped.rotation.y = atan2(srycrx / transformTobeMapped.rotation.x.cos(),
			crycrx / transformTobeMapped.rotation.x.cos());

	float srzcrx = (cos1z * sin1y - cos1y * sin1x * sin1z) * (cos3x * sin3z * (cos2z * sin2y - cos2y * sin2x * sin2z)
			- cos3x * cos3z * (sin2y * sin2z + cos2y * cos2z * sin2x) + cos2x * cos2y * sin3x)
		- (cos1y * cos1z + sin1x * sin1y * sin1z) * (cos3x * cos3z * (cos2y * sin2z - cos2z * sin2x * sin2y)
				- cos3x * sin3z * (cos2y * cos2z + sin2x * sin2y * sin2z) + cos2x * sin3x * sin2y)
		+ cos1x * sin1z * (sin3x * sin2x + cos3x * cos2x * sin3z * sin2z + cos3x * cos3z * cos2x * cos2z);
	float crzcrx = (cos1y * sin1z - cos1z * sin1x * sin1y) * (cos3x * cos3z * (cos2y * sin2z - cos2z * sin2x * sin2y)
			- cos3x * sin3z * (cos2y * cos2z + sin2x * sin2y * sin2z) + cos2x * sin3x * sin2y)
		- (sin1y * sin1z + cos1y * cos1z * sin1x) * (cos3x * sin3z * (cos2z * sin2y - cos2y * sin2x * sin2z)
				- cos3x * cos3z * (sin2y * sin2z + cos2y * cos2z * sin2x) + cos2x * cos2y * sin3x)
		+ cos1x * cos1z * (sin3x * sin2x + cos3x * cos2x * sin3z * sin2z + cos3x * cos3z * cos2x * cos2z);
	transformTobeMapped.rotation.z = atan2(srzcrx / transformTobeMapped.rotation.x.cos(),
			crzcrx / transformTobeMapped.rotation.x.cos());


	Vector3 v = transformIncre.translation;
	rotateZXY(v, transformTobeMapped.rotation);
	transformTobeMapped.translation = transformAftMapped.translation - v;

}

void mapping::point_associate_to_map(point_type const& pi, point_type& po)
{
	po.x = pi.x;
	po.y = pi.y;
	po.z = pi.z;
	po.intensity = pi.intensity;

	rotateZXY(po, transformTobeMapped.rotation);

	po.x += transformTobeMapped.translation.x();
	po.y += transformTobeMapped.translation.y();
	po.z += transformTobeMapped.translation.z();
}

void mapping::point_associate_to_be_mapped(point_type const& pi, point_type& po)
{
	po.x = pi.x - transformTobeMapped.translation.x();
	po.y = pi.y - transformTobeMapped.translation.y();
	po.z = pi.z - transformTobeMapped.translation.z();
	po.intensity = pi.intensity;

	rotateYXZ(po, transformTobeMapped.rotation * -1);
}


} // slam
