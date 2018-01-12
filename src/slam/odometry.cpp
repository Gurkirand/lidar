#include "../../include/slam/odometry.h"
#include <Eigen/Eigenvalues>
#include <Eigen/QR>

namespace slam
{ 
/*
 * So probably don't need those stupid index arrays
 * Also can probably condense the kdtree search
 * Make it work first then see if just using knearest / radius search
 * speeds it up / gets the same or similar result
 * 
 * Arrays can be 192 for corners and 384 for surfaces
 */

odometry::odometry () 
	: edge_search_tree(new kdtree_type()),
	surface_search_tree(new kdtree_type())
	{}

void odometry::init ()
{
	coeff_sel.reset(new cloud_type);
	cloud_ori.reset(new cloud_type);
	iteration = 0;
}

void odometry::process (CloudScans& source,
		ScanFeatures& source_feats,
		ScanFeatures& map_feats,
		ScanFeatures const& target_feats)
{
	init();

	source_scans = &source;
	source_features = &source_feats;
	map_features = &map_feats;
	target_features = &target_feats;

	edge_search_tree->setInputCloud(target_feats.edge_scans.cloud);
	surface_search_tree->setInputCloud(target_feats.surface_scans.cloud);

	for (; iteration < 25; iteration++)
	{
		cloud_ori->clear();
		coeff_sel->clear();

		edge_point_transforms();
		surface_point_transforms();

		if (cloud_ori->size() < 10)
		{
			continue;
		}
		
		if (pose_transform())
		{
			break;
		}
	}

	update_odometry();
	//Set new to previous
}

void odometry::initial_relative_transform (point_type const& pi, float const relPercent, point_type& po)
{
	po.x = pi.x - relPercent * transform.translation.x();
	po.y = pi.y - relPercent * transform.translation.y();
	po.z = pi.z - relPercent * transform.translation.z();
	po.intensity = pi.intensity;

	Angle rx = -relPercent * transform.rotation.x.rad();
	Angle ry = -relPercent * transform.rotation.y.rad();
	Angle rz = -relPercent * transform.rotation.z.rad();
	rotateZXY(po, rz, rx, ry);
}



void odometry::final_relative_transform (CloudScans& cloudScans)
{
	cloud_type& cloud = *(cloudScans.cloud);
	auto& scan_ind = cloudScans.scan_ind;
	size_t cloudSize = cloud.points.size();
	size_t scan = 0;
	size_t scanStart = 0;
	size_t scanEnd = scan_ind[scan + 1];

	for (size_t i = 0; i < cloudSize; i++) {
		if (i == scanEnd)
		{
			scan += 1;
			scanStart = scan_ind[scan];
			scanEnd = scan_ind[scan + 1];
		}

		point_type& point = cloud.points[i];
		float relPercent = ((float) i - scanStart) / (scanEnd - scanStart);

		point.x -= relPercent * transform.translation.x();
		point.y -= relPercent * transform.translation.y();
		point.z -= relPercent * transform.translation.z();

		Angle rx = -relPercent * transform.rotation.x.rad();
		Angle ry = -relPercent * transform.rotation.y.rad();
		Angle rz = -relPercent * transform.rotation.z.rad();
		rotateZXY(point, rz, rx, ry);
		rotateYXZ(point, transform.rotation.y, transform.rotation.x, transform.rotation.z);

		/* point.x += transform.pos.x() - _imuShiftFromStart.x(); */
		/* point.y += transform.pos.y() - _imuShiftFromStart.y(); */
		/* point.z += transform.pos.z() - _imuShiftFromStart.z(); */

		/* rotateZXY(point, _imuRollStart, _imuPitchStart, _imuYawStart); */
		/* rotateYXZ(point, -_imuYawEnd, -_imuPitchEnd, -_imuRollEnd); */
	}
}


/*
 * Search surroudning 2.5 scans in order to find another point in the closest distance.
 * Maybe a better way to do it by using hte kd Tree
 */

void odometry::edge_point_correspondences (point_type const& pointSel, size_t ind)
{
	CloudScans const& targetEdgeScans = target_features->edge_scans;
	cloud_type const& targetEdges = *(targetEdgeScans.cloud);
	/* std::vector<int> indices; */
	/* pcl::removeNaNFromPointCloud(*laserCloudCornerLast,*laserCloudCornerLast, indices); */
	//WHy are they only searching for 1 point
	edge_search_tree->nearestKSearch(pointSel, 1, search_ind, search_sqr_diff);
	int nearestEdgeInd1 = -1, nearestEdgeInd2 = -1;

	if (search_sqr_diff[0] < 25) {
		nearestEdgeInd1 = search_ind[0];
		int nearestEdgeScan = targetEdgeScans.scan(nearestEdgeInd1);
		/* int closestPointScan = int(laserCloudCornerLast->points[nearestEdgeInd1].intensity); */

		/* float edgeSqrDist, minEdgeSqrDist = 25; */
		float edgeSqrDist, minEdgeSqrDist = 25;

		int scanSearchEndInd = nearestEdgeScan + 2;
		int end = scanSearchEndInd > N_SCANS - 3 ?
			targetEdges.points.size() :
			0.5 * targetEdgeScans.scan_ind[scanSearchEndInd] + 0.5 * targetEdgeScans.scan_ind[scanSearchEndInd + 1]; 

		for (int j = nearestEdgeInd1 + 1; j < end; j++) {
			/* if (int(target_cloud->points[j].intensity) > closestPointScan + 2.5) { */
			/* 	break; */
			/* } */

			edgeSqrDist = sqr_diff(targetEdges.points[j], pointSel);

			/* if (int(laserCloudCornerLast->points[j].intensity) > closestPointScan) { */
			if (edgeSqrDist < minEdgeSqrDist) {
				minEdgeSqrDist = edgeSqrDist;
				nearestEdgeInd2 = j;
			}
			/* } */
		}

		// Plus one because >= for 0
		scanSearchEndInd = nearestEdgeScan - 2;
		end = scanSearchEndInd < 3 ? 0 :
			0.5 * targetEdgeScans.scan_ind[scanSearchEndInd] + 0.5 * targetEdgeScans.scan_ind[scanSearchEndInd - 1] + 1; 

		for (int j = nearestEdgeInd1 - 1; j >= end; j--) {
			/* if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan - 2.5) { */
			/* 	break; */
			/* } */

			edgeSqrDist = sqr_diff(targetEdges.points[j], pointSel);

			/* if (int(laserCloudCornerLast->points[j].intensity) < closestPointScan) { */
			if (edgeSqrDist < minEdgeSqrDist) {
				minEdgeSqrDist = edgeSqrDist;
				nearestEdgeInd2 = j;
			}
			/* } */
		}
	}

	nearest_edge_ind1[ind] = nearestEdgeInd1;
	nearest_edge_ind2[ind] = nearestEdgeInd2;
}

void odometry::edge_point_transforms ()
{
	/* point_type pointOri, pointSel, tripod1, tripod2, tripod3, pointProj, coeff; */
	point_type pointSel, pointOri, tripod1, tripod2, coeff, pointProj ;
	//class ? 

	CloudScans& edgeScans = source_features->edge_scans;
	cloud_type& edges = *(source_features->edge_scans.cloud);
	cloud_type const& targetEdges = *(target_features->edge_scans.cloud);

	size_t edgeSize = edges.points.size();

	for (size_t i = 0; i < edgeSize; i++) {
		initial_relative_transform(edges.points[i], edgeScans.scan_percent(i), pointSel);

		if (iteration % 5 == 0) {
			edge_point_correspondences(pointSel, i);
		}

		if (nearest_edge_ind2[i] >= 0) {
			tripod1 = targetEdges.points[nearest_edge_ind1[i]];
			tripod2 = targetEdges.points[nearest_edge_ind2[i]];

			point_type normal = cross_product(diff(pointSel, tripod1), diff(pointSel, tripod2));

			point_type base = diff(tripod1, tripod2);

			float area = length(normal);

			float baseLength = length(base);

			// Since normal vector is orthogonal to base vector
			// The length of their cross product ( height vector) 
			// is |a| * |b| sin(90) = |a| * |b |
			float heightLength = area * baseLength;

			// Projection vector is h / |h|

			pointProj = scale(cross_product(base, normal), 1 / heightLength);

			//What is this for? 
			//
			float ld2 = area / baseLength;

			float s = 1;
			if (iteration >= 5) {
				s = 1 - 1.8 * fabs(ld2);
			}

			coeff.x = s * pointProj.x;
			coeff.y = s * pointProj.y;
			coeff.z = s * pointProj.z;
			coeff.intensity = s * ld2;

			if (s > 0.1 && ld2 != 0) {
				cloud_ori->push_back(edges.points[i]);
				coeff_sel->push_back(coeff);
			}
		}
	}
}

void odometry::surface_point_correspondences (point_type const& pointSel, size_t ind)
{
	CloudScans const& targetSurfaceScans = target_features->surface_scans;
	cloud_type const& targetSurfaces = *(targetSurfaceScans.cloud);

	surface_search_tree->nearestKSearch(pointSel, 1, search_ind, search_sqr_diff);
	int nearestSurfInd1 = -1, nearestSurfInd2 = -1, nearestSurfInd3 = -1;

	if (search_sqr_diff[0] < 25) {
		nearestSurfInd1 = search_ind[0];
		int const nearestSurfScan = targetSurfaceScans.scan(nearestSurfInd1);

		float surfSqrDist, minSurfSqrDist2 = 25, minSurfSqrDist3 = 25;
		int scanSearchEndInd = nearestSurfScan + 2;
		int end = scanSearchEndInd > N_SCANS - 3 ?
			targetSurfaces.points.size() :
			0.5 * targetSurfaceScans.scan_ind[scanSearchEndInd] + 0.5 * targetSurfaceScans.scan_ind[scanSearchEndInd + 1]; 
		for (int j = nearestSurfInd1 + 1; j < end; j++) {
			/* if (int(targetSurfaces->points[j].intensity) > closestSurfScan + 2.5) { */
			/* 	break; */
			/* } */

			surfSqrDist = sqr_diff(targetSurfaces.points[j], pointSel);

			//Not sure what do...
			//
			if (surfSqrDist < minSurfSqrDist2)
			{
				minSurfSqrDist2 = surfSqrDist;
				nearestSurfInd2 = j;
			}

			/* if (int(targetSurfaces->points[j].intensity) <= closestSurfScan) { */
			/* 	if (surfSqrDist < minSurfSqrDist2) { */
			/* 		minSurfSqrDist2 = surfSqrDist; */
			/* 		nearestSurfInd2 = j; */
			/* 	} */
			/* } else { */
			/* 	if (surfSqrDist < minSurfSqrDist3) { */
			/* 		minSurfSqrDist3 = surfSqrDist; */
			/* 		nearestSurfInd3 = j; */
			/* 	} */
			/* } */
		}
		scanSearchEndInd = nearestSurfScan - 2;
		end = scanSearchEndInd < 3 ? 0 :
			0.5 * targetSurfaceScans.scan_ind[scanSearchEndInd] + 0.5 * targetSurfaceScans.scan_ind[scanSearchEndInd - 1] + 1; 

		for (int j = nearestSurfInd1 - 1; j >= 0; j--) {
			/* if (int(targetSurfaces->points[j].intensity) < closestSurfScan - 2.5) { */
			/* 	break; */
			/* } */

			surfSqrDist = sqr_diff(targetSurfaces.points[j], pointSel);

			if (surfSqrDist < minSurfSqrDist3)
			{
				minSurfSqrDist3 = surfSqrDist;
				nearestSurfInd3 = j;
			}
			/* if (int(targetSurfaces->points[j].intensity) >= closestSurfScan) { */
			/* 	if (surfSqrDist < minSurfSqrDist2) { */
			/* 		minSurfSqrDist2 = surfSqrDist; */
			/* 		nearestSurfInd2 = j; */
			/* 	} */
			/* } else { */
			/* 	if (surfSqrDist < minSurfSqrDist3) { */
			/* 		minSurfSqrDist3 = surfSqrDist; */
			/* 		nearestSurfInd3 = j; */
			/* 	} */
			/* } */
		}
	}

	nearest_surface_ind1[ind] = nearestSurfInd1;
	nearest_surface_ind2[ind] = nearestSurfInd2;
	nearest_surface_ind3[ind] = nearestSurfInd3;
}

void odometry::surface_point_transforms ()
{
	point_type pointSel, tripod1, tripod2, tripod3, coeff;
	//class ? 

	CloudScans& surfaceScans = source_features->surface_scans;
	cloud_type& surfaces = *(source_features->surface_scans.cloud);
	cloud_type const& targetSurfaces = *(target_features->surface_scans.cloud);

	size_t surfaceSize = surfaces.points.size();

	for (size_t i = 0; i < surfaceSize; i++) {

		initial_relative_transform(surfaces.points[i], surfaceScans.scan_percent(i), pointSel);

		if (iteration % 5 == 0) 
		{
			surface_point_correspondences(pointSel, i);
		}

		if (nearest_surface_ind2[i] >= 0 && nearest_surface_ind3[i] >= 0) {
			tripod1 = targetSurfaces.points[nearest_surface_ind1[i]];
			tripod2 = targetSurfaces.points[nearest_surface_ind2[i]];
			tripod3 = targetSurfaces.points[nearest_surface_ind3[i]];

			point_type normal = cross_product(diff(tripod2, tripod1), diff(tripod3, tripod1));

			float normal_len = length(normal);

			//normalize
			normalize(normal);

			float projectionDist = dot_product(normal, diff(pointSel, tripod1));

			float s = 1;
			if (iteration >= 5) {
				s = 1 - 1.8 * fabs(projectionDist) / sqrt(length(pointSel));
			}

			coeff.x = s * normal.x;
			coeff.y = s * normal.y;
			coeff.z = s * normal.z;
			coeff.intensity = s * projectionDist;

			if (s > 0.1 && projectionDist != 0) {
				cloud_ori->push_back(surfaces.points[i]);
				coeff_sel->push_back(coeff);
			}
		}
	}
}

void odometry::stack_jacobian (
		Eigen::Matrix<float,Eigen::Dynamic,6>& matA,
		Eigen::VectorXf& matB)
{
	/*
	 * Find the total transform from the individual transforms
	 * of edge and surface features.
	 * 
	 * Transform T = [tx, ty, tz, Ox, Oy, Oz]
	 * where t = translate and O = rotation angle
	 * 
	 * General Rotation = Yaw * Pitch * Roll
	 * Our Oy = yaw, Ox = pitch, and Oz = roll
	 * We use transform matrices R for each axis and our general
	 * transform is then Ry * Rx * Rz
	 * 
	 * Rotation Matrix:
		[ cosry*cosrz + sinrx*sinry*sinrz,  -sinrz*cosry + cosrz*sinrx*sinry, cosrx*sinry]
		[                  cosrx*sinrz,                    cosrx*cosrz,     -sinrx]
		[-sinry*cosrz + cosry*sinrx*sinrz, -sinry*-sinrz + cosry*cosrz*sinrx, cosrx*cosry]
	 * 
	 * Since we need to rotate our translations as well, our final 
	 * transform of a point is
	 * X(final) = R * (X - t) where t is a translation vector
	 * 
	 * We then multiply our final transform with the transforms 
	 * we found for individual points (coeffs)
	 * 
	 * Then we calculate the Jacobian with respect to each axis.
	 * 
	 */
	
	float sinrx = transform.rotation.x.sin();
	float sinry = transform.rotation.y.sin();
	float sinrz = transform.rotation.z.sin();
	float cosrx = transform.rotation.x.cos();
	float cosry = transform.rotation.y.cos();
	float cosrz = transform.rotation.z.cos();

	float tx = transform.translation.x();
	float ty = transform.translation.y();
	float tz = transform.translation.z();

	size_t pointSelNum = cloud_ori->size();

	for (int i = 0; i < pointSelNum; i++)
	{
		point_type pointOri = cloud_ori->points[i];
		point_type coeff = coeff_sel->points[i];

		float arx = (-(cosrx*sinry*sinrz)*(pointOri.x - tx) + (cosrx*cosrz*sinry)*(pointOri.y - ty) + (sinrx*sinry)*(pointOri.z - tz)) * coeff.x
			+ ((sinrx*sinrz)*(pointOri.x - tx) - (cosrz*sinrx)*(pointOri.y - ty) + cosrx * (pointOri.z - tz)) * coeff.y
			+ ((cosrx*cosry*sinrz)*(pointOri.x - tx) - (cosrx*cosry*cosrz)*(pointOri.y - ty) - (cosry*sinrx)*(pointOri.z - tz)) * coeff.z;

		float ary = ((-cosrz*sinry - cosry*sinrx*sinrz) * (pointOri.x - tx) 
				+ (cosry*cosrz*sinrx - sinry*sinrz) * (pointOri.y - ty) - (cosrx*cosry)*(pointOri.z - tz)) * coeff.x
			+ ((cosry*cosrz - sinrx*sinry*sinrz) * (pointOri.x - tx) 
					+ (cosry*sinrz + cosrz*sinrx*sinry) * (pointOri.y - ty) - (cosrx*sinry)*(pointOri.z - tz)) * coeff.z;

		float arz = ((-cosry*sinrz - cosrz*sinrx*sinry) * (pointOri.x - tx) + (cosry*cosrz - sinrx*sinry*sinrz) * (pointOri.y - ty)) * coeff.x
			+ (-(cosrx*cosrz)*(pointOri.x - tx) - (cosrx*sinrz)*(pointOri.y - ty)) * coeff.y
			+ ((cosry*cosrz*sinrx - sinry*sinrz) * (pointOri.x - tx) + (cosrz*sinry + cosry*sinrx*sinrz) * (pointOri.y - ty)) * coeff.z;

		float atx = -(cosry*cosrz - sinrx*sinry*sinrz) * coeff.x + cosrx*sinrz * coeff.y 
			- (cosrz*sinry + cosry*sinrx*sinrz) * coeff.z;

		float aty = -(cosry*sinrz + cosrz*sinrx*sinry) * coeff.x - cosrx*cosrz * coeff.y 
			- (sinry*sinrz - cosry*cosrz*sinrx) * coeff.z;

		float atz = cosrx*sinry * coeff.x - sinrx * coeff.y - cosrx*cosry * coeff.z;

		float d2 = coeff.intensity;

		matA(i, 0) = arx;
		matA(i, 1) = ary;
		matA(i, 2) = arz;
		matA(i, 3) = atx;
		matA(i, 4) = aty;
		matA(i, 5) = atz;
		matB(i, 0) = -0.05 * d2;
	}
}

bool odometry::pose_transform ()
{
	/* int pointSelNum = laserCloudOri->points.size(); */
	/* if (pointSelNum < 10) { */
	/*   continue; */
	/* } */

	size_t pointSelNum = cloud_ori->size();

	/*
	 * A = Jacobian of the total transform (stacked)
	 * B = individual tranfsorms from correspondences
	 * Solve for A.T * A * X = A.T * B
	 */


	Eigen::Matrix<float,Eigen::Dynamic,6> matA(pointSelNum, 6);
	Eigen::Matrix<float,6,Eigen::Dynamic> matAt(6,pointSelNum);
	Eigen::Matrix<float,6,6> matAtA;
	Eigen::VectorXf matB(pointSelNum);
	Eigen::Matrix<float,6,1> matAtB;
	Eigen::Matrix<float,6,1> matX;

	stack_jacobian(matA, matB);

	matAt = matA.transpose();
	matAtA = matAt * matA;
	matAtB = matAt * matB;

	matX = matAtA.colPivHouseholderQr().solve(matAtB);

	if (iteration == 0) {
		Eigen::Matrix<float,1,6> matE;
		Eigen::Matrix<float,6,6> matV;
		Eigen::Matrix<float,6,6> matV2;

		Eigen::SelfAdjointEigenSolver< Eigen::Matrix<float,6, 6> > esolver(matAtA);
		matE = esolver.eigenvalues().real();
		matV = esolver.eigenvectors().real();

		matV2 = matV;

		isDegenerate = false;
		float eignThre[6] = {10, 10, 10, 10, 10, 10};
		for (int i = 5; i >= 0; i--) {
			if (matE(0, i) < eignThre[i]) {
				for (int j = 0; j < 6; j++) {
					matV2(i, j) = 0;
				}
				isDegenerate = true;
			} else {
				break;
			}
		}
		matP = matV.inverse() * matV2;
	}
	if (isDegenerate) {
		Eigen::Matrix<float,6,1> matX2;
		matX2 = matX;
		matX = matP * matX2;
	}

	transform.rotation.x = transform.rotation.x.rad() + matX(0, 0);
	transform.rotation.y = transform.rotation.y.rad() + matX(1, 0);
	transform.rotation.z = transform.rotation.z.rad() + matX(2, 0);
	transform.translation.x() += matX(3, 0);
	transform.translation.y() += matX(4, 0);
	transform.translation.z() += matX(5, 0);

	if( isnan(transform.rotation.x.rad()) ) transform.rotation.x = Angle();
	if( isnan(transform.rotation.y.rad()) ) transform.rotation.y = Angle();
	if( isnan(transform.rotation.z.rad()) ) transform.rotation.z = Angle();

	if( isnan(transform.translation.x()) ) transform.translation.x() = 0.0;
	if( isnan(transform.translation.y()) ) transform.translation.y() = 0.0;
	if( isnan(transform.translation.z()) ) transform.translation.z() = 0.0;

	float deltaR = sqrt(
			pow(rad2deg(matX(0, 0)), 2) +
			pow(rad2deg(matX(1, 0)), 2) +
			pow(rad2deg(matX(2, 0)), 2));
	float deltaT = sqrt(
			pow(matX(3, 0) * 100, 2) +
			pow(matX(4, 0) * 100, 2) +
			pow(matX(5, 0) * 100, 2));

	if (deltaR < 0.1 && deltaT < 0.1) {
		return true;
	}

	return false;
}

Rotation odometry::accumulate_rotation(
		Angle const& rot1x, Angle const& rot1y, Angle const& rot1z,
		Angle const& rot2x, Angle const& rot2y, Angle const& rot2z
		)
{
	//Essentially find angles from combined (multiplied) rotation matrices from the two different rotations
	//One from imu, one from our scan correction
	//
	//Since the rotation matrix looks like this:
		 /* Rotation Matrix:
			[ cosry*cosrz + sinrx*sinry*sinrz,  -sinrz*cosry + cosrz*sinrx*sinry, cosrx*sinry]
			[                  cosrx*sinrz,                    cosrx*cosrz,     -sinrx]
			[-sinry*cosrz + cosry*sinrx*sinrz, -sinry*-sinrz + cosry*cosrz*sinrx, cosrx*cosry]
		 */ 
	//We can find sinx by multiplying the rotation matrix and just taking the element in position [2][3]
	//The find other angles using it
	//

	Rotation accRot;

	float sinrx = rot2x.cos()*rot1x.cos()*rot2y.sin()*rot1z.sin() 
		- rot1x.cos()*rot1z.cos()*rot2x.sin() 
		- rot2x.cos()*rot2y.cos()*rot1x.sin();

	accRot.x = -asin(sinrx);

	float sinrycosrx = rot2x.sin()*(rot1y.cos()*rot1z.sin() - rot1z.cos()*rot1x.sin()*rot1y.sin()) 
		+ rot2x.cos()*rot2y.sin()*(rot1y.cos()*rot1z.cos() + rot1x.sin()*rot1y.sin()*rot1z.sin()) 
		+ rot2x.cos()*rot2y.cos()*rot1x.cos()*rot1y.sin();

	float cosrycosrx = rot2x.cos()*rot2y.cos()*rot1x.cos()*rot1y.cos() 
		- rot2x.cos()*rot2y.sin()*(rot1z.cos()*rot1y.sin() - rot1y.cos()*rot1x.sin()*rot1z.sin()) 
		- rot2x.sin()*(rot1y.sin()*rot1z.sin() + rot1y.cos()*rot1z.cos()*rot1x.sin());

	accRot.y = atan2(sinrycosrx / accRot.x.cos(), cosrycosrx / accRot.x.cos());

	float sinrzcosrx = rot1x.sin()*(rot2z.cos()*rot2y.sin() - rot2y.cos()*rot2x.sin()*rot2z.sin()) 
		+ rot1x.cos()*rot1z.sin()*(rot2y.cos()*rot2z.cos() + rot2x.sin()*rot2y.sin()*rot2z.sin()) 
		+ rot2x.cos()*rot1x.cos()*rot1z.cos()*rot2z.sin();

	float cosrzcosrx = rot2x.cos()*rot2z.cos()*rot1x.cos()*rot1z.cos() 
		- rot1x.cos()*rot1z.sin()*(rot2y.cos()*rot2z.sin() - rot2z.cos()*rot2x.sin()*rot2y.sin()) 
		- rot1x.sin()*(rot2y.sin()*rot2z.sin() + rot2y.cos()*rot2z.cos()*rot2x.sin());

	accRot.z = atan2(sinrzcosrx / accRot.x.cos(), cosrzcosrx / accRot.x.cos());

	return accRot;
}

void odometry::update_odometry ()
{
	transformSum.rotation = accumulate_rotation(
			transformSum.rotation.x,
			transformSum.rotation.y,
			transformSum.rotation.z,
			transform.rotation.x,
			transform.rotation.y * 1.05,
			transform.rotation.z);

	Vector3 v0(transform.translation.x(),
			transform.translation.y(),
			transform.translation.z() * 1.05);
	rotateZXY(v0, transformSum.rotation);

	/* transformSum - v0; */
	transformSum.translation = (transformSum.translation - v0);

	geoQuat = make_quaternion_rpy(
			transformSum.rotation.z.rad(),
			-transformSum.rotation.x.rad(),
			-transformSum.rotation.y.rad());

	/*
	 * .....
	 */
	
	final_relative_transform(map_features->edge_scans);
	final_relative_transform(map_features->surface_scans);
	final_relative_transform(*source_scans);

}

} // slam

