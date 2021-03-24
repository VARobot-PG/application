#include "../include/cloud.h"

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

namespace cloud
{
	void cutPointCloudRect(pcl::PointCloud<PointType>::ConstPtr cloudIn, pcl::PointCloud<PointType>::Ptr cloudOut, tf2::Vector3 start, tf2::Vector3 end)
	{
		if((start.x() >= end.x()) || (start.y() >= end.y()) || (start.z() >= end.z()))
		{
			std::cout << "[cutPointCloudRect]:: Error! Start > End!\n";
			return;
		}

		cloudOut->clear();
		for(size_t i = 0; i < cloudIn->size(); i++)
		{
			if(cloudIn->at(i).x != cloudIn->at(i).x) continue; //Ignoring NaN
			double x = cloudIn->at(i).x;
			double y = cloudIn->at(i).y;
			double z = cloudIn->at(i).z;
			if((x == 0.0) && (y == 0.0) && (z == 0.0)) continue;
			if((x > start.x()) && (x < end.x()) && (y > start.y()) && (y < end.y()) && (z > start.z()) && (z < end.z()))
			{
				cloudOut->push_back(cloudIn->at(i));
			}
		}
	}

	void cutPointCloudCylinder(pcl::PointCloud<PointType>::ConstPtr cloudIn, pcl::PointCloud<PointType>::Ptr cloudOut, double radius)
	{
		if(radius < 0)
		{
			std::cout << "[cutPointCloudRect]:: Error! Radius < 0!\n";
			return;
		}

		cloudOut->clear();

		#pragma omp parallel for shared(cloudIn, radius)
		for(size_t i = 0; i < cloudIn->size(); i++)
		{
			if(cloudIn->at(i).x != cloudIn->at(i).x) continue; //Ignoring NaN
			double x = cloudIn->at(i).x;
			double y = cloudIn->at(i).y;
			if((x == 0.0) && (y == 0.0)) continue;

			double r = std::sqrt((x*x) + (y*y));
			if(r < radius)
			{
				cloudOut->push_back(cloudIn->at(i));
			}
		}
	}

	/*Using the LAB colorspace: https://en.wikipedia.org/wiki/CIELAB_color_space*/
	tf2::Vector3 convertToLabColorspace(const PointType a)
	{
		cv::Mat3b mat;
		mat.push_back(cv::Vec3b(a.b,a.g,a.r));

		cv::Mat3b matConv;
		cv::cvtColor(mat, matConv, cv::COLOR_BGR2Lab);

		return tf2::Vector3((double)matConv.at<uint8_t>(0,0),(double)matConv.at<uint8_t>(0,1),(double)matConv.at<uint8_t>(0,2));
	}

	void cutBackgroundColor(pcl::PointCloud<PointType>::ConstPtr cloudIn, pcl::PointCloud<PointType>::Ptr cloudOut, double threshold, tf2::Vector3 color)
	{
		for(size_t i = 0; i < cloudIn->size(); i++)
		{
			tf2::Vector3 other((double)cloudIn->at(i).r,(double)cloudIn->at(i).g,(double)cloudIn->at(i).b);
			double d = other.distance(color);
			if(d > threshold)
			{
				cloudOut->push_back(cloudIn->at(i));
			}
		}
	}

	void findRegionWithColor(pcl::PointCloud<PointType>::Ptr cloudIn, pcl::PointCloud<PointType>::Ptr cloudOut, const double threshold, const PointType color)
	{
		const tf2::Vector3 colorConv = convertToLabColorspace(color);
		double smallestDist = -1.0;
		//#pragma omp parallel for shared(cloudIn,cloudOut)
		for(size_t i = 0; i < cloudIn->size(); i++)
		{
			tf2::Vector3 other = convertToLabColorspace(cloudIn->at(i));
			if(other.distance(colorConv) < threshold)
			{
				cloudOut->push_back(cloudIn->at(i));
			}
			if(smallestDist < 0 || smallestDist > other.distance(colorConv))
			{
				smallestDist = other.distance(colorConv);
			}
		}
		std::cout << "SmallestDist = " << smallestDist << std::endl;
	}

	void normalBasedSegmentation(pcl::PointCloud<PointType>::Ptr cloudIn, pcl::PointCloud<PointType>::Ptr cloudOut,
			tf2::Vector3 normal, double threshold, double threshold2, double radius, int k)
	{
		cloudOut->clear();
		if(cloudIn->size() == 0) return;
		//Calculate normals. We will later calculate the angle between the given normal and the normal of each point.
		pcl::PointCloud<pcl::Normal>::Ptr normals = boost::make_shared<pcl::PointCloud<pcl::Normal>>();
		pcl::PointCloud<PointType>::Ptr out2 = boost::make_shared<pcl::PointCloud<PointType>>();
		pcl::NormalEstimationOMP<cloud::PointType,pcl::Normal> norm_est;
		norm_est.setInputCloud(cloudIn);
		pcl::search::KdTree<cloud::PointType>::Ptr tree (new pcl::search::KdTree<cloud::PointType> ());
		norm_est.setSearchMethod(tree);
		norm_est.setKSearch(k);
		norm_est.compute(*normals);
		normal.normalize();
		pcl::PointCloud<PointType>::Ptr candidates = boost::make_shared<pcl::PointCloud<PointType>>();
		//Classify all points as winner, candidate or looser
		#pragma omp parallel for shared(cloudIn, cloudOut, threshold, out2, threshold2, candidates, normals)
		for(size_t i = 0; i < cloudIn->size(); i++)
		{
			//remove all (0,0,0) points;
			if((cloudIn->at(i).x == 0.0) && (cloudIn->at(i).y == 0.0)) continue;
			//remove all Nan points
			if(cloudIn->at(i).x != cloudIn->at(i).x) continue;
			//remove all Nan normals
			if(normals->at(i).normal_x != normals->at(i).normal_x) continue;

			//Calculate the differences between the given normal and all calculated normals.
			tf2::Vector3 n(normals->at(i).normal_x,normals->at(i).normal_y,normals->at(i).normal_z);
			double angle = std::fabs(std::fmod(n.angle(normal)+M_PI_2,M_PI)-M_PI_2);

			if(angle < threshold) //Hysteresis thresholding
			{
				#pragma omp critical
				cloudOut->push_back(cloudIn->at(i));
			}else if(angle < threshold2)
			{
				#pragma omp critical
				candidates->push_back(cloudIn->at(i));
			}
		}
		//If there are no candidates, we dont have to do the next for loop. If we dont have winners, there is nothing good in the scene.
		if((candidates->size() == 0) || (cloudOut->size() == 0))
		{
			return;
		}

		//We will take a look, if candidates have winners in their neighborhood
		pcl::KdTreeFLANN<PointType>::Ptr kdTree = boost::make_shared<pcl::KdTreeFLANN<PointType>>();
		kdTree->setInputCloud(cloudOut);
		/*If candidates have a lot of winners in their neighborhood, they will also be classified as winner!*/
		#pragma omp parallel for shared(candidates, cloudOut, kdTree)
		for(size_t i = 0; i < candidates->size(); i++)
		{
			std::vector<int> neighbors(4);
			std::vector<float> squareDistances(4);
			kdTree->radiusSearch(candidates->at(i),radius,neighbors,squareDistances,4);
			if(neighbors.size() == 4)
			{
				#pragma omp critical
				cloudOut->push_back(candidates->at(i));
			}
		}
	}

	void regionGrowingNew(pcl::PointCloud<PointType>::Ptr cloudIn, pcl::PointCloud<PointType>::Ptr colorCloudOut,
			std::vector<pcl::PointCloud<PointType>::Ptr>& cloudsOut, int minClusterSize, int maxClusterSize,
			double distanceThreshold)
	{
		if(cloudIn->size() == 0) return;
		pcl::search::Search <PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
		tree->setInputCloud(cloudIn);

		std::vector<unsigned int> pointsList(cloudIn->size());
		for(size_t i = 0; i < pointsList.size(); i++)
		{
			pointsList.at(i) = 0;
		}

		unsigned int clusterId = 1;
		std::vector<unsigned int> validCluster;
		for(size_t i = 0; i < pointsList.size(); i++)
		{
			if(pointsList.at(i) != 0) continue;

			size_t currentSize = 0;
			grow(cloudIn,tree,&pointsList,i,&currentSize,clusterId,maxClusterSize,distanceThreshold);
			validCluster.push_back(currentSize > (size_t)minClusterSize);
			clusterId++;
		}

		for(size_t i = 0; i < clusterId; i++)
		{
			cloudsOut.push_back(boost::make_shared<pcl::PointCloud<PointType>>());
		}

		for(size_t i = 0; i < cloudIn->size(); i++)
		{
			if(pointsList.at(i) == 0) continue;

			if(validCluster.at(pointsList.at(i) - 1))
			{
				cloudsOut.at(pointsList.at(i) - 1)->push_back(cloudIn->at(i));

				PointType p;
				p.x = cloudIn->at(i).x;
				p.y = cloudIn->at(i).y;
				p.z = cloudIn->at(i).z;
				p.r = (pointsList.at(i) % 3)*125;
				p.g = ((pointsList.at(i)/3) % 3)*125;
				p.b = ((pointsList.at(i)/9) % 3)*125;
				colorCloudOut->push_back(p);
			}
		}

		std::vector<pcl::PointCloud<PointType>::Ptr>::iterator iter = cloudsOut.begin();
		while(iter != cloudsOut.end())
		{
			if(iter->get()->size() == 0)
			{
				iter = cloudsOut.erase(iter);
			}else{
				++iter;
			}
		}
	}

	void grow(pcl::PointCloud<PointType>::Ptr cloudIn, pcl::search::Search <PointType>::Ptr tree, std::vector<unsigned int>* pointsList, const size_t p,
			size_t *currentSize, const unsigned int clusterId, const size_t maxClusterSize, const double distanceThreshold)
	{
		if(pointsList->at(p) != 0) return;
		if((*currentSize) >= maxClusterSize) return;

		(*currentSize)++;
		pointsList->at(p) = clusterId;
		if((*currentSize) >= maxClusterSize) return;

		size_t maxAllowedGrowSize = maxClusterSize-(*currentSize);
		std::vector<int> neighbors(maxAllowedGrowSize);
		std::vector<float> squareDistances(maxAllowedGrowSize);
		size_t num = (size_t)tree->radiusSearch(cloudIn->at(p),distanceThreshold,neighbors,squareDistances,maxAllowedGrowSize);
		for(size_t i = 0; i < num; i++)
		{
			if(pointsList->at(neighbors.at(i)) == 0)
			{
				grow(cloudIn,tree,pointsList,neighbors.at(i),currentSize,clusterId,maxClusterSize,distanceThreshold);
			}
		}
	}

	void colorBasedRegionGrowing(pcl::PointCloud<PointType>::Ptr cloudIn, pcl::PointCloud<PointType>::Ptr colorCloudOut,
			std::vector<pcl::PointCloud<PointType>::Ptr>& cloudsOut, int minClusterSize, int maxClusterSize,
			double distanceThreshold, int pointColorThreshold, int regionColorThreshold)
	{
		pcl::IndicesPtr indices (new std::vector <int>);

		for(size_t i = 0; i < cloudIn->size(); i++)
		{
			indices->push_back(i);
		}
		pcl::search::Search <PointType>::Ptr tree (new pcl::search::KdTree<PointType>);
		pcl::RegionGrowingRGB<PointType> reg;
		reg.setInputCloud (cloudIn);
		reg.setIndices(indices);
		reg.setSearchMethod (tree);
		reg.setDistanceThreshold (distanceThreshold);
		reg.setPointColorThreshold (pointColorThreshold);
		reg.setRegionColorThreshold (regionColorThreshold);
		reg.setMinClusterSize (minClusterSize);
		reg.setMaxClusterSize(maxClusterSize);
		std::vector <pcl::PointIndices> clusters;
		reg.extract (clusters);

		std::cout << "Done with region growing!\n";
		std::cout << "Found " << clusters.size() << " clusters!\n";

		colorCloudOut->clear();

		int colorCounter = 0;
		for(pcl::PointIndices i : clusters) //Create a vector of clouds, also fill the colorCloudOut PointCloud which is usefull for debugging and visualization
		{
			colorCounter++;
			pcl::PointCloud<PointType>::Ptr cloud = boost::make_shared<pcl::PointCloud<PointType>>();
			for(int p : i.indices)
			{
				cloud->push_back(cloudIn->at(p));

				int r = colorCounter % 4;
				int g = (colorCounter/2) % 4;
				int b = (colorCounter/4) % 4;

				cloudIn->at(p).r = (r*255)/3;
				cloudIn->at(p).g = (g*255)/3;
				cloudIn->at(p).b = (b*255)/3;
				colorCloudOut->push_back(cloudIn->at(p));
			}
			cloudsOut.push_back(cloud);
		}
	}

	void regionGrowing(pcl::PointCloud<PointType>::Ptr cloudIn, pcl::PointCloud<PointType>::Ptr colorCloudOut,
                       std::vector<pcl::PointCloud<PointType>::Ptr>& cloudsOut, long minClusterSize, long maxClusterSize, long numberOfNeighbours)
    {
        pcl::search::Search<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
        pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimator;
        normal_estimator.setSearchMethod (tree);
        normal_estimator.setInputCloud (cloudIn);
        normal_estimator.setKSearch (10);
        normal_estimator.compute (*normals);

        pcl::RegionGrowing<pcl::PointXYZRGB, pcl::Normal> reg;
        reg.setMinClusterSize (minClusterSize);
        reg.setMaxClusterSize (maxClusterSize);
        reg.setSearchMethod (tree);
        reg.setNumberOfNeighbours (numberOfNeighbours);
        reg.setInputCloud (cloudIn);
        reg.setInputNormals (normals);

        std::vector <pcl::PointIndices> clusters;
        reg.extract (clusters);

        std::cout << "Done with region growing!\n";
        std::cout << "Found " << clusters.size() << " clusters!\n";

        colorCloudOut->clear();

        int colorCounter = 0;
        for(pcl::PointIndices i : clusters) //Create a vector of clouds, also fill the colorCloudOut PointCloud which is usefull for debugging and visualization
        {
            colorCounter++;
            pcl::PointCloud<PointType>::Ptr cloud = boost::make_shared<pcl::PointCloud<PointType>>();
            for(int p : i.indices)
            {
                cloud->push_back(cloudIn->at(p));

                int r = colorCounter % 4;
                int g = (colorCounter/2) % 4;
                int b = (colorCounter/4) % 4;

                cloudIn->at(p).r = (r*255)/3;
                cloudIn->at(p).g = (g*255)/3;
                cloudIn->at(p).b = (b*255)/3;
                colorCloudOut->push_back(cloudIn->at(p));
            }
            cloudsOut.push_back(cloud);
        }
    }

	void getMedianPoint(pcl::PointCloud<PointType>::Ptr cloudIn, size_t& id)
	{
		double smallestDistanceSum = -1.0;
		id = 0;
		for(size_t p = 0; p < cloudIn->size(); p++)//For each point, get the distance sum to all other points
		{
			double currentDistanceSum = 0.0;
			tf2::Vector3 a(cloudIn->at(p).x,cloudIn->at(p).y,cloudIn->at(p).z);
			for(size_t i = 0; i < cloudIn->size(); i++)
			{
				if(i == p) continue;
				tf2::Vector3 b(cloudIn->at(i).x,cloudIn->at(i).y,cloudIn->at(i).z);
				currentDistanceSum += a.distance(b);
			}

			//Does this point have the smallest distance to all other points? If yes, then this is the median
			if((smallestDistanceSum < 0) || (smallestDistanceSum > currentDistanceSum))
			{
				smallestDistanceSum = currentDistanceSum;
				id = p;
			}
		}
	}

	void getEndAndStartPoint2D(pcl::PointCloud<PointType>::Ptr cloudIn, pcl::PointXYZ& start, pcl::PointXYZ& end, double& height)
	{
		if(cloudIn->size() == 0) return;

		//Init results with the first point
		start.x = cloudIn->at(0).x;
		start.y = cloudIn->at(0).y;
		start.z = cloudIn->at(0).z;
		double sumStart = start.x + start.y;
		end.x = cloudIn->at(0).x;
		end.y = cloudIn->at(0).y;
		end.y = cloudIn->at(0).z;
		height = cloudIn->at(0).z;
		double sumEnd = end.x + end.y;

		for(size_t i = 1; i < cloudIn->size(); i++)
		{
			double sum = cloudIn->at(i).x + cloudIn->at(i).y;
			if(sumStart > sum) //Start is the "smalles" point
			{
				start.x = cloudIn->at(i).x;
				start.y = cloudIn->at(i).y;
				start.z = cloudIn->at(i).z;
				sumStart = sum;
			}
			else if(sumEnd < sum) //End is the "greatest" point
			{
				end.x = cloudIn->at(i).x;
				end.y = cloudIn->at(i).y;
				end.z = cloudIn->at(i).z;
				sumEnd = sum;
			}
			if(height < cloudIn->at(i).z)
			{
				height = cloudIn->at(i).z;
			}
		}
	}

	bool getClosestPoint (tf2::Vector3 loc, pcl::PointCloud<PointType>::ConstPtr cloudIn, size_t &id, tf2::Vector3 &foundLoc)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr tmpCloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
		pcl::copyPointCloud(*cloudIn,*tmpCloud);

		//using a kdtree to search for the nearest point
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		kdtree.setInputCloud (tmpCloud);

		pcl::PointXYZ searchPoint;
		searchPoint.x = loc.x();
		searchPoint.y = loc.y();
		searchPoint.z = loc.z();
		std::vector<int> pointIdxNKNSearch(1);
		std::vector<float> pointNKNSquaredDistance(1);

		//Do a nearest K Search
		if ( kdtree.nearestKSearch (searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
		{
			id = pointIdxNKNSearch[0];
			foundLoc.setX(tmpCloud->points[ pointIdxNKNSearch[0]].x);
			foundLoc.setY(tmpCloud->points[ pointIdxNKNSearch[0]].y);
			foundLoc.setZ(tmpCloud->points[ pointIdxNKNSearch[0]].z);
			return true;
		}
		return false;
	}
}
