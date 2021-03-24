/*
 * MedianFilter.cpp
 *
 *  Created on: Mar 19, 2019
 *      Author: philipf
 */
#include "../include/medianFilter.h"
namespace cloud
{

MedianFilter::MedianFilter(bool bKeepNaN, double gMinValidPercentage): keepNaN(bKeepNaN), minValidPercentage(gMinValidPercentage)
{

}
void MedianFilter::push_back(pcl::PointCloud<cloud::PointType>::ConstPtr cloud)
{
	//create a new pointcloud containing the sample and push it into the vector of samples
	pcl::PointCloud<cloud::PointType>::ConstPtr newSample = boost::make_shared<pcl::PointCloud<cloud::PointType>>(*cloud);
	samples.push_back(newSample);
}
size_t MedianFilter::size()
{
	return samples.size();
}
void MedianFilter::clear()
{
	samples.clear();
}
/*Recunstructs a cloud based on the saved samples.*/
bool MedianFilter::compute(pcl::PointCloud<cloud::PointType>::Ptr result)
{
	if(samples.size() == 0) return false;
	if(samples.size() == 1)
	{
		*result = *samples.at(0);
		return true;
	}
	reconstructCloudFromSamples(samples,result);
	result->header = samples.at(0)->header;
	return true;
}

/*Gives the mean color of a set of points which are close to the median. NaN not allowed as input!*/
bool MedianFilter::reconstructColor(const std::vector<PointType> points, const int maxPoints, tf2::Vector3& color)
{
	double averageError = 0.0;
	double squareAverage = 0.0;
	double smallestError = -1.0;
	std::vector<double> localErrors;
	//Get average error and eliminate huge errors
	for(size_t i = 0; i < points.size(); i++)
	{
		double localError = 0;
		for(size_t j = 0; j < points.size(); j++) //Get the local error sum
		{
			if(i == j) continue;
			localError += std::pow(points.at(i).r - points.at(j).r,2.0)
			+ std::pow(points.at(i).g - points.at(j).g,2.0) + std::pow(points.at(i).b - points.at(j).b,2.0);
		}
		if((smallestError > localError) || (smallestError == -1.0))
		{
			smallestError = localError;
		}
		averageError += localError;
		squareAverage += (localError*localError);
		localErrors.push_back(localError);
	}
	averageError /= (double)points.size(); //Get the average local error sum
	squareAverage /= (double)points.size();
	double variance = squareAverage - (averageError*averageError);
	variance = std::sqrt(variance);
	color = tf2::Vector3(0,0,0);
	int errorFreeSamples = 0;
	//Get average color from all points wich are near the median solution
	for(size_t i = 0; i < points.size(); i++)
	{
		if((localErrors.at(i)-smallestError) <= variance) //Is the point better than the average, or at least better then average+variance?
		{
			errorFreeSamples++;
			color += tf2::Vector3(points.at(i).r,points.at(i).g,points.at(i).b);
		}
	}
	if(points.size() < size_t(maxPoints*minValidPercentage)) return false;
	if(errorFreeSamples == 0) return false;
	color = color / ((double)errorFreeSamples);
	return true;
}

/*Gives the mean position of a set of points which are close to the median. NaN not allowed as input!*/
bool MedianFilter::reconstructPosition(const std::vector<PointType> points, const int maxPoints, tf2::Vector3& position, tf2::Vector3& color)
{
	double averageError = 0.0;
	double squareAverage = 0.0;
	double smallestError = -1.0;
	std::vector<double> localErrors;
	//Get average error and eliminate huge errors
	for(size_t i = 0; i < points.size(); i++)
	{
		double localError = 0;
		for(size_t j = 0; j < points.size(); j++) //Get the local error sum
		{
			if(i == j) continue;
			localError += std::pow(points.at(i).x - points.at(j).x,2.0)
			+ std::pow(points.at(i).y - points.at(j).y,2.0) + std::pow(points.at(i).z - points.at(j).z,2.0);
		}
		if((smallestError > localError) || (smallestError == -1.0))
		{
			smallestError = localError;
		}
		averageError += localError;
		squareAverage += (localError*localError);
		localErrors.push_back(localError);
	}
	averageError /= (double)points.size(); //Get the average local error sum
	squareAverage /= (double)points.size();
	double variance = squareAverage - (averageError*averageError);
	variance = std::sqrt(variance);
	position = tf2::Vector3(0,0,0);
	int errorFreeSamples = 0;
	//Get average color from all points wich are near the median solution
	for(size_t i = 0; i < points.size(); i++)
	{
		if((localErrors.at(i)-smallestError) <= variance) //Is the point better than the average, or at least better then average+variance?
		{
			errorFreeSamples++;
			position += tf2::Vector3(points.at(i).x,points.at(i).y,points.at(i).z);
		}
	}
	if(points.size() < size_t(maxPoints*minValidPercentage)) return false;
	if(errorFreeSamples == 0) return false;
	position = position / ((double)errorFreeSamples);
	return true;
}

/*Samples from the same camera position and the same scene have to be used as input. Median points for all indexes in the clouds will be created.*/
void MedianFilter::reconstructCloudFromSamples(std::vector<pcl::PointCloud<PointType>::ConstPtr>& clouds, pcl::PointCloud<PointType>::Ptr output)
{
	output->clear();
	*output = *clouds.at(0);

	#pragma omp parallel for shared(clouds, output)
	for(size_t i = 0; i < clouds.at(0)->size(); i++) //Create each point which will be put into the output cloud
	{
		std::vector<PointType> points; //All Points here represent the same pixel, but came from multiple samples
		for(size_t j = 0; j < clouds.size(); j++) //Check if the points are valid, then push them into the vector of points
		{
			//Prevent array out of bounds
			if(i >= clouds.at(j)->size())
			{
				std::cout << "[mergeSamples]: index = " << i << " but cloud " << j << " has size: " << clouds.at(j)->size() << std::endl;
				continue;
			}
			//Ignore NaN
			if(clouds.at(j)->points.at(i).x != clouds.at(j)->points.at(i).x) continue;

			points.push_back(clouds.at(j)->points.at(i));
		}
		if(points.size() == 0) //There was no valid sample, so the output becomes a NaN point
		{
			if(keepNaN)
			{
				PointType nanPoint;
				output->at(i) = nanPoint;
			}
			continue;
		}

		tf2::Vector3 position,color, colorB;
		bool noValidPoint = true;
		if(reconstructColor(points,clouds.size(),color)) //Get the best fitting color by inspecting all points
		{
			if(reconstructPosition(points,clouds.size(),position,colorB)) //Get the best position by inspecting all points
			{
				PointType perfectPoint;
				perfectPoint.x = position.x();
				perfectPoint.y = position.y();
				perfectPoint.z = position.z();
				perfectPoint.r = color.x();
				perfectPoint.g = color.y();
				perfectPoint.b = color.z();
				output->at(i) = perfectPoint;
				noValidPoint = false;
			}
		}
		if(noValidPoint && keepNaN) //There was no valid sample, so the output becomes a NaN point
		{
			PointType nanPoint;
			output->at(i) = nanPoint;
		}
	}
}

}//Namespace


