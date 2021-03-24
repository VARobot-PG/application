/*
 * medianFilter.h
 *
 *  Created on: Mar 19, 2019
 *      Author: philipf
 *
 *  MedianFilter is a small wrapper around the function cloud::MergeSamples. It consists of a vector multiple samples and tries
 *  to create a error free pointcloud out of all samples.
 */

#ifndef SRC_DOBOT_DETECTION_INCLUDE_MEDIANFILTER_H_
#define SRC_DOBOT_DETECTION_INCLUDE_MEDIANFILTER_H_

#include "cloud.h"
#include <tf2/LinearMath/Transform.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
namespace cloud
{

class MedianFilter
{
public:
	MedianFilter(bool bKeepNaN=true, double gMinValidPercentage=0.9);
	//Adds a new pointcloud as a sample
	void push_back(pcl::PointCloud<cloud::PointType>::ConstPtr cloud);
	//Gets the number of samples
	size_t size();
	//Clears all the samples, run this wenn processing a new scene
	void clear();
	//Uses all samples to compute an optimized pointcloud with less noise
	bool compute(pcl::PointCloud<cloud::PointType>::Ptr result);
private:
	//Methods for reconstructing a point from multiple samples and reconstructing a cloud from multiple samples.
	bool reconstructColor(const std::vector<PointType> points, const int maxPoints, tf2::Vector3& color);
	bool reconstructPosition(const std::vector<PointType> points, const int maxPoints, tf2::Vector3& position, tf2::Vector3& color);
	void reconstructCloudFromSamples(std::vector<pcl::PointCloud<PointType>::ConstPtr>& clouds, pcl::PointCloud<PointType>::Ptr output);

	std::vector<pcl::PointCloud<cloud::PointType>::ConstPtr> samples;
	bool keepNaN = true;
	double minValidPercentage = 0.9;

};

}//Namespace



#endif /* SRC_DOBOT_DETECTION_INCLUDE_MEDIANFILTER_H_ */
