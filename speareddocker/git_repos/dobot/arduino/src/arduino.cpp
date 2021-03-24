//
// Created by lars on 12.03.19.
//

#include "../include/arduino/arduino.h"

namespace arduino {
/**
 * ---------------------------------------------------------------------------------------------------------------------
 * arduino base class functions
 */

    Arduino::Arduino(std::string name, std::shared_ptr<ros::NodeHandle> nh_ptr) : name(name), nh_ptr(nh_ptr) {}


    template<typename M>
    ros::ServiceClient Arduino::register_service_client(const std::string service)
    {
        return nh_ptr->serviceClient<M>(name + "/" + service);
    }


    template<typename M, typename T>
    ros::Subscriber Arduino::register_subscriber(const std::string topic, uint32_t queue_size,
                                                 void (T::*fp)(const boost::shared_ptr<const M> &), T *obj)
    {
        return nh_ptr->subscribe(name + "/" + topic, queue_size, fp, obj);
    }

/**
 * ---------------------------------------------------------------------------------------------------------------------
 * conveyor node functions
 */

    ConveyorNode::ConveyorNode(std::shared_ptr<ros::NodeHandle> nh_ptr)
            : Arduino(device_names::CONVEYOR_NODE_NAME, nh_ptr)
    {
        if_sensor = register_subscriber<std_msgs::Bool>("GetInfraredSensorBlocked", 1, &ConveyorNode::if_callback,
                                                        this);
        belt_speed = register_subscriber<arduino_msgs::get_velocity>("GetConveyorbeltSpeed", 1,
                                                        &ConveyorNode::speed_callback, this);
        transportbelt = register_service_client<arduino_msgs::SetConveyorbeltSpeed>("SetConveyorbeltSpeed");
    }


    void ConveyorNode::if_callback(const std_msgs::BoolConstPtr &msg)
    {
        if_blocked = msg->data;
        if(remoteCallbackSet)
        {
        	if_remote_callback(msg);
        }
    }


    void ConveyorNode::speed_callback(const arduino_msgs::get_velocityConstPtr &msg)
    {
        speed = msg->velocity;
    }


    bool ConveyorNode::set_transportbelt_speed(int8_t speed) {
        if (speed < -100 || speed > 100)
            return false;

        arduino_msgs::SetConveyorbeltSpeed::Request req;
        arduino_msgs::SetConveyorbeltSpeed::Response res;

        req.speed = speed;

        if (transportbelt.call(req, res))
            return true;

        return false;
    }


    int8_t ConveyorNode::get_transportbelt_speed() const
    {
        return speed;
    }


    bool ConveyorNode::is_if_sensor_blocked() const {
        return if_blocked;
    }


    void ConveyorNode::register_if_callback(void (*fp)(const std_msgs::BoolConstPtr &))
    {
        if_remote_callback = fp;
        remoteCallbackSet = true;
    }

/**
 * ---------------------------------------------------------------------------------------------------------------------
 * sensor node functions
 */
    SensorNode::SensorNode(std::shared_ptr<ros::NodeHandle> nh_ptr)
            : Arduino(device_names::SENSOR_NODE_NAME, nh_ptr) {
        right_dist_sensor = register_subscriber<arduino_msgs::distance>("GetDistanceSensorRight", 1,
                                                                        &SensorNode::right_dist_callback, this);
        left_dist_sensor = register_subscriber<arduino_msgs::distance>("GetDistanceSensorLeft", 1,
                                                                       &SensorNode::left_dist_callback, this);
        color_sensor = register_subscriber<arduino_msgs::rgb_color>("GetRGBColor", 1,
                                                                    &SensorNode::color_sensor_callback, this);
    }


    void SensorNode::right_dist_callback(const arduino_msgs::distanceConstPtr &msg) {
        r_dist = msg->dist;
        r_status = msg->status;

        read_l_dist = true;
    }


    void SensorNode::left_dist_callback(const arduino_msgs::distanceConstPtr &msg) {
        l_dist = msg->dist;
        l_status = msg->status;

        read_l_dist = true;
    }


    void SensorNode::color_sensor_callback(const arduino_msgs::rgb_colorConstPtr &msg) {
        r = msg->r;
        g = msg->g;
        b = msg->b;
        c = msg->c;

        read_color = true;
    }


    bool SensorNode::read_r_dist_sensor(uint8_t &dist, uint8_t &status)
    {
    	dist = r_dist;
        status = r_status;

        read_r_dist = false;
        return true;
    }


    bool SensorNode::read_l_dist_sensor(uint8_t &dist, uint8_t &status)
    {

        dist = l_dist;
        status = l_status;

        read_l_dist = false;
        return true;
    }


    bool SensorNode::read_rgb_sensor(uint8_t &r, uint8_t &g, uint8_t &b, uint16_t &c) {
        if (read_color) {
            r = this->r;
            g = this->g;
            b = this->b;
            c = this->c;

            read_color = false;
            return true;
        }

        return false;
    }

    bool SensorNode::read_rgb_sensor(uint8_t &r, uint8_t &g, uint8_t &b) {
        uint16_t c;
        return read_rgb_sensor(r, g, b, c);
    }


/**
* ---------------------------------------------------------------------------------------------------------------------
* rfid node functions
*/

    RFIDNode::RFIDNode(std::shared_ptr<ros::NodeHandle> nh_ptr)
            : Arduino(device_names::RFID_NODE_NAME, nh_ptr) {
        // create service
        rfid = register_service_client<arduino_msgs::rfiduid>("GetRFIDUID");
        rfid_publisher = nh_ptr->advertise<arduino_msgs::rfid>("/arduino_rfid/uid", 1000);
    }

    bool RFIDNode::get_rfid_uid(uint8_t &b0, uint8_t &b1, uint8_t &b2, uint8_t &b3) {
        // call service and get results
        arduino_msgs::rfiduidRequest req;
        arduino_msgs::rfiduidResponse res;

        if(rfid.call(req, res)){
            b0 = res.b0;
            b1 = res.b1;
            b2 = res.b2;
            b3 = res.b3;

            msg.b0 = res.b0;
            msg.b1 = res.b1;
            msg.b2 = res.b2;
            msg.b3 = res.b3;
            rfid_publisher.publish(msg);
            ros::spinOnce();

            return true;
        }
        return false;
    }


}
