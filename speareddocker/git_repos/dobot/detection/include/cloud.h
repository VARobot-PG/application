/*
 * PointCloudFunctions.h
 *
 *  Created on: Feb 14, 2019
 *      Author: philipf
 */

#ifndef SRC_DOBOT_DETECTION_INCLUDE_CLOUD_H_
#define SRC_DOBOT_DETECTION_INCLUDE_CLOUD_H_
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <tf2/LinearMath/Transform.h>
#include <marker_helper/markerHelper.h>

namespace cloud
{
	//Used as a shotcut and to change the pointtype of all functions
	typedef pcl::PointXYZRGB PointType;

	/**
	 * Only will keep points which are inside a rectangle.
	 * @param start Start position of the rectangle.
	 * @param end End position of the rectangle.
	 */
	void cutPointCloudRect(pcl::PointCloud<PointType>::ConstPtr cloudIn, pcl::PointCloud<PointType>::Ptr cloudOut, tf2::Vector3 start, tf2::Vector3 end);
	void cutPointCloudCylinder(pcl::PointCloud<PointType>::ConstPtr cloudIn, pcl::PointCloud<PointType>::Ptr cloudOut, double radius);
	void cutBackgroundColor(pcl::PointCloud<PointType>::ConstPtr cloudIn, pcl::PointCloud<PointType>::Ptr cloudOut, double threshold, tf2::Vector3 color);

	void findRegionWithColor(pcl::PointCloud<PointType>::Ptr cloudIn, pcl::PointCloud<PointType>::Ptr cloudOut, const double threshold, const PointType color);

	tf2::Vector3 convertToLabColorspace(const PointType a);
	/**
	 * Cuts away all points which are not facing along the given normal vector.
	 * @param normal Normal vector which will be compared with each single normal from cloudIn.
	 * @param threshold If the angle between the given normal and the normal in cloudIn is higher than this, the point will not be copied into cloudOut
	 * */
	void normalBasedSegmentation(pcl::PointCloud<PointType>::Ptr cloudIn, pcl::PointCloud<PointType>::Ptr cloudOut,
			tf2::Vector3 normal, double threshold, double threshold2, double radius, int k);

	/**
	 * Uses region growing for color segmentation.
	 * @param cloudIn is the input cloud
	 * @param colorCloudOut is mainly used for visualization/debug. Each cluster will get a own color.
	 * @param cloudsOut is an array of pointclouds where every cloud is a connected cluster with similar color
	 */
	void colorBasedRegionGrowing(pcl::PointCloud<PointType>::Ptr cloudIn, pcl::PointCloud<PointType>::Ptr colorCloudOut,
			std::vector<pcl::PointCloud<PointType>::Ptr>& cloudsOut, int minClusterSize=20, int maxClusterSize=500,
			double distanceThreshold=0.01, int pointColorThreshold=6, int regionColorThreshold=5);

    /**
     * Uses region growing for color segmentation.
     * @param cloudIn is the input cloud
     * @param colorCloudOut is mainly used for visualization/debug. Each cluster will get a own color.
     * @param cloudsOut is an array of pointclouds where every cloud is a connected cluster with similar color
     */
    void regionGrowing(pcl::PointCloud<PointType>::Ptr cloudIn, pcl::PointCloud<PointType>::Ptr colorCloudOut,
                       std::vector<pcl::PointCloud<PointType>::Ptr>& cloudsOut, long minClusterSize, long maxClusterSize, long numberOfNeighbours);

    void regionGrowingNew(pcl::PointCloud<PointType>::Ptr cloudIn, pcl::PointCloud<PointType>::Ptr colorCloudOut,
    			std::vector<pcl::PointCloud<PointType>::Ptr>& cloudsOut, int minClusterSize, int maxClusterSize,
    			double distanceThreshold);

	/**
	 * Will find the most negative and most positive point in the x-y plane of the given cloud.
	 * @param cloudIn is the input cloud. It should be an extracted object, where the user wants to know the dimensions from.
	 * @param start is the smallest/most negative point.
	 * @param end is the greatest/most positive point.
	 * @param height is the z value of the highest point.
	 */
	void getEndAndStartPoint2D(pcl::PointCloud<PointType>::Ptr cloudIn, pcl::PointXYZ& start, pcl::PointXYZ& end, double& height);

	/**
	 * Calculates the median points location of the given cloud.
	 * @param cloudIn is the input cloud
	 * @param id is the medians point index in cloudIn
	 */
	void getMedianPoint(pcl::PointCloud<PointType>::Ptr cloudIn, size_t& id);

	/**
	 * Gets the closest point next to loc and sets Id and foundLoc.
	 * @param loc Location where to find the nearest point.
	 * @param cloudRGB Pointcloud where the nearest point has to be searched in
	 * @param id This id will be set to the nearest points id.
	 * @param foundLoc This vector will be set to the nearest points location.
	 * @return returns false if pcl methods fail.
	 */
	bool getClosestPoint (tf2::Vector3 loc, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloudIn, size_t &id, tf2::Vector3 &foundLoc);

	void grow(pcl::PointCloud<PointType>::Ptr cloudIn, pcl::search::Search <PointType>::Ptr tree, std::vector<unsigned int>* pointsList, const size_t p,
			size_t *currentSize, const unsigned int clusterId, const size_t maxClusterSize, const double distanceThreshold);
}



#endif /* SRC_DOBOT_DETECTION_INCLUDE_CLOUD_H_ */
