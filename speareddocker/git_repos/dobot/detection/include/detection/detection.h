//
// Created by lars on 05.08.19.
//

#ifndef SRC_DETECTION_H
#define SRC_DETECTION_H

#include <ros/ros.h>
#include <ros/time.h>
#include <detection_msgs/DetectedObjects.h>
#include <detection_msgs/DetectedObject.h>
#include <mutex>
#include <vector>
#include <string>

/**
 * Namespace for detection
 */
namespace detection {

    namespace grasp_point_getter_names
    {
        const std::string RAIL_GRASP_POINT_DETECTION = "RailGraspDetection";

        const std::string LOAD_GRASP_POINT_DETECTION = "LoadGraspDetection";
    }

    /**
     * base class for retrieving detected objects
     */
    class GraspObjectGetter
    {
    private:
        std::mutex mtx;
        detection_msgs::DetectedObjects::ConstPtr detectedObjects;
        std::shared_ptr<ros::NodeHandle> nh_ptr;
        bool objectsOld;
        std::string name;
        ros::Subscriber detectedObjectsSub;
        void detectedObjectsCallback(const detection_msgs::DetectedObjects::ConstPtr& obj);
    protected:
        /**
         * protected default constructor to prevent initializing this class without using derived ones
         */
        GraspObjectGetter() {}
        /**
         * constructor to create this object
         * @param name name of the GraspObjectGetter
         * @param nh_ptr ros node handle
         */
        GraspObjectGetter(const std::string name, std::shared_ptr<ros::NodeHandle> nh_ptr);
        /**
         * register a new subscriber for this GraspObjectGetter
         * @tparam M the message type
         * @tparam T the object type for which the member function is called
         * @param topic the name of the topic to subscribe
         * @param queue_size the size of the message queue
         * @param fp function pointer to the callback function
         * @param obj the object for which the callback function should be called
         * @return a subscriber to the specified topic
         */
        template<typename M, typename T>
        ros::Subscriber register_subscriber(const std::string topic, uint32_t queue_size,
                                            void (T::*fp)(const boost::shared_ptr<const M> &), T *obj);
        /**
         * @param latestTime latest ros time to look for objects from
         * @param probabilty minimum probability an object needs
         * @param x_dir direction of the x axis (-1 for left, 1 for right)
         * @return the object with the smallest y coordinate (most right one) with a timestamp newer than latestTime
         * and a probability greater than or equal to probability
         */
        detection_msgs::DetectedObject getRightMostObject(ros::Time latestTime, double probability, int8_t x_dir);
    public:
        /**
         * @param probabilty minimum probability an object needs
         * @return the newest object based on timestamp and that has a greater or equal probability than provided
         */
        detection_msgs::DetectedObject getLatestObject(double probability);
        /**
         * @param latestTime latest ros time to look for objects from
         * @param probabilty minimum probability an object needs
         * @return all detected objects with a timestamp newer than latestTime and a probability greater than or equal to probability
         */
        std::vector<detection_msgs::DetectedObject> getDetectedObjects(ros::Time latestTime, double probability);
        /**
         * @return the frame id of the detected objects
         */
        std::string getFrameID();
        /**
         * @return {@code true} iff there are no detected objects
         */
        bool isEmpty();
    };

    class RailGraspObjectGetter : public GraspObjectGetter
    {
    public:
        RailGraspObjectGetter(std::shared_ptr<ros::NodeHandle> nh_ptr);

        detection_msgs::DetectedObject getRightMostObject(ros::Time latestTime, double probability);
    };

    class LoadGraspObjectGetter : public GraspObjectGetter
    {
    public:
        LoadGraspObjectGetter(std::shared_ptr<ros::NodeHandle> nh_ptr);

        detection_msgs::DetectedObject getRightMostObject(ros::Time latestTime, double probability);
    };
}

#endif //SRC_DETECTION_H
