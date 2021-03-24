//
// Created by speeters on 14.03.19.
//

#ifndef DOBOT_CONNECTOR_H
#define DOBOT_CONNECTOR_H

#include <ros/ros.h>
#include <ros/spinner.h>

#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <arduino_msgs/distance.h>
#include <arduino_msgs/rgb_color.h>
#include <arduino_msgs/rfid.h>
#include <arduino_msgs/get_velocity.h>
#include <detection_msgs/DetectedObjects.h>
#include <detection_msgs/DetectedGrid.h>


#include <iostream>
#include <string>
#include <map>
#include <functional>
#include <unistd.h>

#include "../lib/influxdb_cpp/src/influxdb_line.h"
#include "../lib/influxdb_cpp/src/influxdb_simple_async_api.h"

#include <functional>
#include <iostream>

namespace influxdb_connector{
    template <class T> class connector{
    private:
        influxdb::async_api::simple_db db;
        std::function<influxdb::api::line(T, std::string)> connector_func;
        std::string meas;

    public:

        connector(std::string url, std::string username, std::string password, std::string database, std::string measurement, std::function<influxdb::api::line(T, std::string)> conn_func)
                : db(url, database), meas(measurement), connector_func(conn_func){
            db.with_authentication(username, password);
        }

        void callback(const T msg){
            db.insert(connector_func(msg, meas));
        }

        ros::Subscriber subscribe(ros::NodeHandle node, std::string sname){
            return node.subscribe(sname, 1000, &connector::callback, this);
        }
    };
    namespace functions{
        influxdb::api::line geometry_msgs_pose(geometry_msgs::Pose, std::string);
        influxdb::api::line std_msgs_bool(std_msgs::Bool, std::string);
        influxdb::api::line std_msgs_float64(std_msgs::Float64, std::string);
        influxdb::api::line arduino_msgs_distance(arduino_msgs::distance, std::string);
        influxdb::api::line arduino_msgs_rgb_color(arduino_msgs::rgb_color, std::string);
        influxdb::api::line sensor_msgs_joint_state(sensor_msgs::JointState, std::string);
        influxdb::api::line detected_objects(detection_msgs::DetectedObjects msg, std::string measurement);
        influxdb::api::line detected_grid(detection_msgs::DetectedGrid msg, std::string measurement);
        influxdb::api::line arduino_msgs_rfid(arduino_msgs::rfid, std::string);
        influxdb::api::line arduino_msgs_get_velocity(arduino_msgs::get_velocity , std::string );
    }
}





#endif //DOBOT_CONNECTOR_H
