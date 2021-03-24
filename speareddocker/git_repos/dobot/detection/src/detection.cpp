//
// Created by lars on 05.08.19.
//

#include "../include/detection/detection.h"


namespace detection {

    //------------------------------------------------------------------------------------------------------------------
    //GraspObjectGetter

    GraspObjectGetter::GraspObjectGetter(const std::string name, std::shared_ptr<ros::NodeHandle> nh_ptr)
    {
        this->name = name;
        this->nh_ptr = nh_ptr;
        this->objectsOld = true;

        detectedObjectsSub = register_subscriber("objectsSave", 1,
                                                 &GraspObjectGetter::detectedObjectsCallback, this);
    }

    template<typename M, typename T>
    ros::Subscriber GraspObjectGetter::register_subscriber(const std::string topic, uint32_t queue_size,
                                                           void (T::*fp)(const boost::shared_ptr<const M> &), T *obj)
    {
        return nh_ptr->subscribe(name + "/" + topic, queue_size, fp, obj);
    }

    void GraspObjectGetter::detectedObjectsCallback(const detection_msgs::DetectedObjects::ConstPtr &obj)
    {
        std::lock_guard<std::mutex> lock(mtx);
        detectedObjects = obj;
        if(detectedObjects->objects.size() > 0)
            objectsOld = false;
    }

    detection_msgs::DetectedObject GraspObjectGetter::getLatestObject(double probability)
    {
        std::lock_guard<std::mutex> lock(mtx);

        detection_msgs::DetectedObject found;
        found.probability = -1;

        for(auto obj : detectedObjects->objects)
        {
            //lastest object has the newest timestamp
            if((found.probability == -1 || found.timestamp < obj.timestamp) && obj.probability >= probability)
                found = obj;
        }
        //mark objects as old
        objectsOld = true;
        return found;
    }

    detection_msgs::DetectedObject GraspObjectGetter::getRightMostObject(ros::Time latestTime, double probability, int8_t x_dir)
    {
        std::lock_guard<std::mutex> lock(mtx);

        detection_msgs::DetectedObject found;
        found.probability = -1;

        for(auto obj : detectedObjects->objects)
        {
            //most right object has the greatest x coordinate
            if((found.probability == -1 || found.graspPoint.x*x_dir < obj.graspPoint.x*x_dir)
                && obj.probability >= probability && obj.timestamp >= latestTime)
                found = obj;
        }
        //mark objects as old
        objectsOld = true;
        return found;
    }

    std::vector<detection_msgs::DetectedObject> GraspObjectGetter::getDetectedObjects(ros::Time latestTime, double probability)
    {
        std::vector<detection_msgs::DetectedObject> objs;
        std::lock_guard<std::mutex> lock(mtx);

        for(auto obj : detectedObjects->objects)
        {
            if (obj.probability >= probability && obj.timestamp >= latestTime)
                objs.push_back(obj);
        }
        //mark objects as old
        objectsOld = true;
        return objs;
    }

    std::string GraspObjectGetter::getFrameID() {
        std::lock_guard<std::mutex> lock(mtx);
        return detectedObjects->frame_id;
    }

    bool GraspObjectGetter::isEmpty() {
        return detectedObjects == nullptr || objectsOld || detectedObjects->objects.size() == 0;
    }

    //------------------------------------------------------------------------------------------------------------------
    //RailGraspObjectGetter

    RailGraspObjectGetter::RailGraspObjectGetter(std::shared_ptr<ros::NodeHandle> nh_ptr)
        : GraspObjectGetter(grasp_point_getter_names::RAIL_GRASP_POINT_DETECTION, nh_ptr) { }

    detection_msgs::DetectedObject RailGraspObjectGetter::getRightMostObject(ros::Time latestTime,
                                                                             double probability){
        return GraspObjectGetter::getRightMostObject(latestTime, probability, 1);
    }

    //------------------------------------------------------------------------------------------------------------------
    //LoadGraspObjectGetter

    LoadGraspObjectGetter::LoadGraspObjectGetter(std::shared_ptr<ros::NodeHandle> nh_ptr)
            : GraspObjectGetter(grasp_point_getter_names::LOAD_GRASP_POINT_DETECTION, nh_ptr) { }

    detection_msgs::DetectedObject LoadGraspObjectGetter::getRightMostObject(ros::Time latestTime,
                                                                             double probability){
        return GraspObjectGetter::getRightMostObject(latestTime, probability, -1);
    }
}