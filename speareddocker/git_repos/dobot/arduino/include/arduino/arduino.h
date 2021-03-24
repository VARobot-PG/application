//
// Created by lars on 12.03.19.
//

#ifndef PROJECT_ARDUINO_H
#define PROJECT_ARDUINO_H

#include <ros/ros.h>
#include <memory>
#include <boost/shared_ptr.hpp>
#include <std_msgs/Bool.h>
#include <arduino_msgs/SetConveyorbeltSpeed.h>
#include <arduino_msgs/distance.h>
#include <arduino_msgs/rgb_color.h>
#include <arduino_msgs/get_velocity.h>
#include <arduino_msgs/rfiduid.h>
#include <arduino_msgs/rfid.h>

namespace arduino
{
    namespace device_names
    {
        const std::string CONVEYOR_NODE_NAME = "arduino_conveyor";

        const std::string SENSOR_NODE_NAME = "arduino_sensor";

        const std::string RFID_NODE_NAME = "arduino_rfid";
    }

    //represents an abstract arduino object with its sensors and actuators
    class Arduino
    {
    private:
         std::shared_ptr<ros::NodeHandle> nh_ptr;
         std::string name;
    protected:
        /**
         * protected default constructor to prevent initializing this class without using derived ones
         */
        Arduino() {}
        /**
         * constructor to create this object
         * @param name name of the arduino node
         * @param nh_ptr ros node handle
         */
        Arduino(const std::string name, std::shared_ptr<ros::NodeHandle> nh_ptr);
        /**
         * register a new subscriber for this arduino node
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
         * register a new service client for this arduino node
         * @tparam M the message type
         * @param service the service name to register
         * @return the service client for thie service
         */
        template<typename M>
        ros::ServiceClient register_service_client(const std::string service);
    };

    //------------------------------------------------------------------------------------------------------------------

    /**
     * Represents the arduino controlling the conveyor belt and the infrared sensor.
     */
    class ConveyorNode : public Arduino
    {
    private:
        bool if_blocked;
        bool remoteCallbackSet = false;
        int8_t speed;
        ros::Subscriber if_sensor, belt_speed;
        ros::ServiceClient transportbelt;
        void (*if_remote_callback)(const std_msgs::BoolConstPtr &);

        void if_callback(const std_msgs::BoolConstPtr &msg);
        void speed_callback(const arduino_msgs::get_velocityConstPtr &msg);

    public:
       explicit ConveyorNode(std::shared_ptr<ros::NodeHandle> nh_ptr);
       /**
        * Sets the speed of the transportbelt. Possible values are -100 to 100.
        * @param speed the new speed of the transportbelt (-100 to 100)
        * @return true iff speed could be set
        */
       bool set_transportbelt_speed(const int8_t speed);
       /**
        * Gets the momentary speed of the transportbelt in range -100 to 100.
        * @return speed in range -100 to 100
        */
       int8_t get_transportbelt_speed() const;
       /**
        * Returns the state of the if sensor
        * @return true if if sensor is blocked, false otherwise
        */
       bool is_if_sensor_blocked() const;

        /**
         * register a function callback for the if sensor.
         * @tparam M
         * @param fp
         */
       void register_if_callback(void (*fp)(const std_msgs::BoolConstPtr &));
    };

    //------------------------------------------------------------------------------------------------------------------

    /**
     * Represents the arduino monitoring the distance sensors and the color sensor.
     */
    class SensorNode : public Arduino
    {
    private:
        ros::Subscriber right_dist_sensor, left_dist_sensor, color_sensor;
        uint8_t r, g, b;
        uint16_t c;
        uint8_t r_dist, l_dist, r_status, l_status;
        bool read_color = false, read_r_dist = false, read_l_dist = false;

        void right_dist_callback(const arduino_msgs::distanceConstPtr &msg);
        void left_dist_callback(const arduino_msgs::distanceConstPtr &msg);
        void color_sensor_callback(const arduino_msgs::rgb_colorConstPtr &msg);

    public:
        explicit SensorNode(std::shared_ptr<ros::NodeHandle> nh_ptr);
        /**
         * Reads the values of the right distance sensor into the provided variables.
         * @param dist output variable for distance
         * @param status output variable for status
         * @return true iff a distance could be read
         */
        bool read_r_dist_sensor(uint8_t &dist, uint8_t &status);
        /**
         * Reads the values of the left distance sensor into the provided variables.
         * @param dist output variable for distance
         * @param status output variable for status
         * @return true iff a distance could be read
         */
        bool read_l_dist_sensor(uint8_t &dist, uint8_t &status);
        /**
         * Reads the values of the rgb sensor into the provided variables.
         * @param r output variable for red value
         * @param g output variable for green value
         * @param b output variable for blue value
         * @param c output variable for clear value
         * @return true iff a color could be read
         */
        bool read_rgb_sensor(uint8_t &r, uint8_t &g, uint8_t &b, uint16_t &c);
        /**
         * Convencience function when clear value is not needed
         * @param r red value
         * @param g green value
         * @param b blue value
         * @return true iff a color could be read
         */
        bool read_rgb_sensor(uint8_t &r, uint8_t &g, uint8_t &b);
    };

    /**
      * Represents the arduino that offers the service to read RFID tags
     */
    class RFIDNode : public Arduino
    {
    private:
        ros::ServiceClient rfid;
        ros::Publisher rfid_publisher;
        arduino_msgs::rfid msg;

    public:
        explicit RFIDNode(std::shared_ptr<ros::NodeHandle> nh_ptr);
        /**
         * Reads the values of the RFID tag into the provides values. If nor RFID tag is present
         * on the tag scanner all variables are set to 0.
         */
        bool get_rfid_uid(uint8_t &b0, uint8_t &b1, uint8_t &b2, uint8_t &b3);
    };


}



#endif //PROJECT_ARDUINO_H
