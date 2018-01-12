#ifndef SLAM_FEATURE_EXTRACTION_H
#define SLAM_FEATURE_EXTRACTION_H

#include "common.h"
#include "math_utils.h"
#include <array>
#include <bitset>

namespace slam
{

class feature_extraction
{

private:

	float const scanPeriod = 0.1;

	size_t const sample_size = 5;

	CloudScans const* source_scans;
	size_t cloud_size;

	std::array<float, MAXIMUM_CLOUD_SIZE> smoothness_coeff;
	std::bitset<MAXIMUM_CLOUD_SIZE> neighbor_picked;
	std::array<size_t, MAXIMUM_CLOUD_SIZE> sorted_ind;

	void something (cloud_type const& sourceCloud);
	void preprocess_scans (cloud_type const& sourceCloud);
	void evaluate_smoothness ();
	void reject_features ();
	void sort_coeffs (size_t const start, size_t const end);
	void extract_features ();

public:

	ScanFeatures odometry_features;

	//may not need scans for map_features
	ScanFeatures map_features;

	feature_extraction();

	void process (CloudScans const& source);

};

}

#endif // SLAM_FEATURE_EXTRACTION_H
