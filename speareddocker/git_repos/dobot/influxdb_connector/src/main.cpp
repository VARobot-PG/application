//
// Created by speeters on 11.04.19.
//


#include "../include/connector.h"
#include <dobot/dobot.h>
#include <dobot_msgs/GetEndEffectorSuctionCup.h>

std::string url = "https://vm-eng-ki4as-db:8086";
std::string db = "fun_with_flags_0";
std::string uname = "admin";
std::string pw = "2MG9zEvQ";


template<typename T> influxdb_connector::connector<T>* wrapper_conn(std::string meas, std::function<influxdb::api::line(T,std::string)> func){
    return new influxdb_connector::connector<T> (url, uname, pw, db, meas, func);
}

void truck_scenario(int argc, char** argv){
    ros::init(argc,argv, "influxdb_connector");
    ros::NodeHandle n;

    // connector functions
    std::function<influxdb::api::line(geometry_msgs::Pose, std::string)> func_geo_pose = influxdb_connector::functions::geometry_msgs_pose;
    std::function<influxdb::api::line(std_msgs::Bool, std::string)> func_std_bool = influxdb_connector::functions::std_msgs_bool;
    std::function<influxdb::api::line(std_msgs::Float64, std::string)> func_std_float64 = influxdb_connector::functions::std_msgs_float64;
    std::function<influxdb::api::line(sensor_msgs::JointState, std::string)> func_joint_state = influxdb_connector::functions::sensor_msgs_joint_state;
    std::function<influxdb::api::line(detection_msgs::DetectedObjects, std::string)> func_detected_objs = influxdb_connector::functions::detected_objects;
    std::function<influxdb::api::line(detection_msgs::DetectedGrid, std::string)> func_detected_grid = influxdb_connector::functions::detected_grid;
    std::function<influxdb::api::line(arduino_msgs::rfid, std::string)> func_arduino_rfid = influxdb_connector::functions::arduino_msgs_rfid;

    // arduino
    influxdb_connector::connector<std_msgs::Bool>* asinfra = wrapper_conn("arduino_conveyor_GetInfraredSensorBlocked", func_std_bool);

    auto s1 = asinfra->subscribe(n, "/arduino_conveyor/GetInfraredSensorBlocked");

    // dobot loader
    influxdb_connector::connector<geometry_msgs::Pose>* ldap = wrapper_conn<geometry_msgs::Pose>("dobot_loader_arm_Pose", func_geo_pose);
    influxdb_connector::connector<sensor_msgs::JointState>* ldjs = wrapper_conn("dobot_loader_joint_states", func_joint_state);
    influxdb_connector::connector<std_msgs::Bool>* ldi = wrapper_conn("dobot_loader_idle", func_std_bool);

    auto s2 = ldap->subscribe(n, "/" + dobot::dobot_names::DOBOT_LOADER + "/arm/Pose");
    auto s3 = ldjs->subscribe(n, "/" + dobot::dobot_names::DOBOT_LOADER + "/joint_states");
    auto s4 = ldi->subscribe(n, "/" + dobot::dobot_names::DOBOT_LOADER + "/idle");

    // dobot rail

    influxdb_connector::connector<geometry_msgs::Pose>* rdap = wrapper_conn<geometry_msgs::Pose>("dobot_rail_arm_Pose", func_geo_pose);
    influxdb_connector::connector<sensor_msgs::JointState>*rdjs = wrapper_conn("dobot_rail_joint_states", func_joint_state);
    influxdb_connector::connector<std_msgs::Bool>* rdi = wrapper_conn("dobot_rail_idle", func_std_bool);
    influxdb_connector::connector<std_msgs::Float64>* rdlrl = wrapper_conn("dobot_rail_linear_rail_pos", func_std_float64);

    auto s5 = rdap->subscribe(n, "/" + dobot::dobot_names::DOBOT_RAIL + "/arm/Pose");
    auto s6 = rdjs->subscribe(n, "/" + dobot::dobot_names::DOBOT_RAIL + "/joint_states");
    auto s7 = rdi->subscribe(n, "/" + dobot::dobot_names::DOBOT_RAIL + "/idle");
    auto s8 = rdlrl->subscribe(n, "/" + dobot::dobot_names::DOBOT_RAIL + "linear_rail/pos");

    // load grasp detection
    influxdb_connector::connector<detection_msgs::DetectedObjects>* lgdo = wrapper_conn<detection_msgs::DetectedObjects>("LoadGraspDetection_objects", func_detected_objs);
    influxdb_connector::connector<detection_msgs::DetectedObjects>* lgdos = wrapper_conn<detection_msgs::DetectedObjects>("LoadGraspDetection_objects_save", func_detected_objs);

    auto s9 = lgdo->subscribe(n,"/LoadGraspDetection/objects");
    auto s10 = lgdos->subscribe(n,"/LoadGraspDetection/objectsSave");

    // rail grasp detection
    influxdb_connector::connector<detection_msgs::DetectedObjects>* rgdo = wrapper_conn<detection_msgs::DetectedObjects>("RailGraspDetection_objects", func_detected_objs);
    influxdb_connector::connector<detection_msgs::DetectedObjects>* rgdos = wrapper_conn<detection_msgs::DetectedObjects>("RailGraspDetection_objects_save", func_detected_objs);

    auto s11 = rgdo->subscribe(n,"/RailGraspDetection/objects");
    auto s12 = rgdos->subscribe(n,"/RailGraspDetection/objectsSave");

    // grid
    influxdb_connector::connector<detection_msgs::DetectedGrid>* gddt = wrapper_conn<detection_msgs::DetectedGrid>("GridDetection_detected_grid", func_detected_grid);
    auto s13 = gddt->subscribe(n, "/GridDetection/detectedGrid");

    // rfid
    influxdb_connector::connector<arduino_msgs::rfid>* ar = wrapper_conn<arduino_msgs::rfid>("arduino_rfid_uid", func_arduino_rfid);
    auto s14 = ar->subscribe(n, "/arduino_rfid/uid");

    ros::AsyncSpinner spinner = ros::AsyncSpinner(0);
    spinner.start();
    while(ros::ok());
    spinner.stop();
}

void flag_scenario(int argc, char** argv){
    db = argv[1];
    ros::init(argc,argv, db);
    ros::NodeHandle n;

    influxdb::async_api::simple_db influx_db(url,db);
    influx_db.with_authentication(uname, pw);

    // connector functions
    std::function<influxdb::api::line(geometry_msgs::Pose, std::string)> func_geo_pose = influxdb_connector::functions::geometry_msgs_pose;
    std::function<influxdb::api::line(arduino_msgs::rgb_color, std::string)> func_color = influxdb_connector::functions::arduino_msgs_rgb_color;
    std::function<influxdb::api::line(std_msgs::Bool, std::string)> func_std_bool = influxdb_connector::functions::std_msgs_bool;
    std::function<influxdb::api::line(std_msgs::Float64, std::string)> func_std_float64 = influxdb_connector::functions::std_msgs_float64;
    std::function<influxdb::api::line(sensor_msgs::JointState, std::string)> func_joint_state = influxdb_connector::functions::sensor_msgs_joint_state;
    std::function<influxdb::api::line(detection_msgs::DetectedObjects, std::string)> func_detected_objs = influxdb_connector::functions::detected_objects;
    std::function<influxdb::api::line(detection_msgs::DetectedGrid, std::string)> func_detected_grid = influxdb_connector::functions::detected_grid;
    std::function<influxdb::api::line(arduino_msgs::rfid, std::string)> func_arduino_rfid = influxdb_connector::functions::arduino_msgs_rfid;
    std::function<influxdb::api::line(arduino_msgs::rgb_color, std::string)> func_arduino_color = influxdb_connector::functions::arduino_msgs_rgb_color;
    std::function<influxdb::api::line(arduino_msgs::get_velocity, std::string)> func_arduino_velocity = influxdb_connector::functions::arduino_msgs_get_velocity;
    std::function<influxdb::api::line(arduino_msgs::distance, std::string)> func_arduino_distance = influxdb_connector::functions::arduino_msgs_distance;

    // arduino
    influxdb_connector::connector<std_msgs::Bool>* asinfra = wrapper_conn("arduino_conveyor_GetInfraredSensorBlocked", func_std_bool);
    influxdb_connector::connector<arduino_msgs::rgb_color>* asrgb = wrapper_conn("arduino_sensor_GetRGBColor", func_arduino_color);

    auto s1 = asinfra->subscribe(n, "/arduino_conveyor/GetInfraredSensorBlocked");
    auto scolor = asrgb->subscribe(n, "/arduino_sensor/GetRGBColor");

    // dobot loader
    influxdb_connector::connector<geometry_msgs::Pose>* ldap = wrapper_conn<geometry_msgs::Pose>("dobot_loader_arm_Pose", func_geo_pose);
    influxdb_connector::connector<sensor_msgs::JointState>* ldjs = wrapper_conn("dobot_loader_joint_states", func_joint_state);
    influxdb_connector::connector<std_msgs::Bool>* ldi = wrapper_conn("dobot_loader_idle", func_std_bool);

    auto s2 = ldap->subscribe(n, "/" + dobot::dobot_names::DOBOT_LOADER + "/arm/Pose");
    auto s3 = ldjs->subscribe(n, "/" + dobot::dobot_names::DOBOT_LOADER + "/joint_states");
    auto s4 = ldi->subscribe(n, "/" + dobot::dobot_names::DOBOT_LOADER + "/idle");

    // dobot rail

    influxdb_connector::connector<geometry_msgs::Pose>* rdap = wrapper_conn<geometry_msgs::Pose>("dobot_rail_arm_Pose", func_geo_pose);
    influxdb_connector::connector<sensor_msgs::JointState>*rdjs = wrapper_conn("dobot_rail_joint_states", func_joint_state);
    influxdb_connector::connector<std_msgs::Bool>* rdi = wrapper_conn("dobot_rail_idle", func_std_bool);
    influxdb_connector::connector<std_msgs::Float64>* rdlrl = wrapper_conn("dobot_rail_linear_rail_pos", func_std_float64);

    auto s5 = rdap->subscribe(n, "/" + dobot::dobot_names::DOBOT_RAIL + "/arm/Pose");
    auto s6 = rdjs->subscribe(n, "/" + dobot::dobot_names::DOBOT_RAIL + "/joint_states");
    auto s7 = rdi->subscribe(n, "/" + dobot::dobot_names::DOBOT_RAIL + "/idle");
    auto s8 = rdlrl->subscribe(n, "/" + dobot::dobot_names::DOBOT_RAIL + "linear_rail/pos");

    // load grasp detection
    influxdb_connector::connector<detection_msgs::DetectedObjects>* lgdo = wrapper_conn<detection_msgs::DetectedObjects>("loadGraspDetection_objects", func_detected_objs);
    influxdb_connector::connector<detection_msgs::DetectedObjects>* lgdos = wrapper_conn<detection_msgs::DetectedObjects>("loadGraspDetection_objects_save", func_detected_objs);

    auto s9 = lgdo->subscribe(n,"/LoadGraspDetection/objects");
    auto s10 = lgdos->subscribe(n,"/LoadGraspDetection/objectsSave");

    // rail grasp detection
    influxdb_connector::connector<detection_msgs::DetectedObjects>* rgdo = wrapper_conn<detection_msgs::DetectedObjects>("railGraspDetection_objects", func_detected_objs);
    influxdb_connector::connector<detection_msgs::DetectedObjects>* rgdos = wrapper_conn<detection_msgs::DetectedObjects>("railGraspDetection_objects_save", func_detected_objs);

    auto s11 = rgdo->subscribe(n,"/RailGraspDetection/objects");
    auto s12 = rgdos->subscribe(n,"/RailGraspDetection/objectsSave");

    influxdb_connector::connector<arduino_msgs::get_velocity>* conv = wrapper_conn<arduino_msgs::get_velocity>("arduino_getConveyorbeltSpeed", func_arduino_velocity);
    auto a13 = conv -> subscribe(n, "/arduino_conveyor/GetConveyorbeltSpeed");

    influxdb_connector::connector<arduino_msgs::distance>* distl = wrapper_conn<arduino_msgs::distance>("arduino_sensor_getDistanceSensorLeft", func_arduino_distance);
    influxdb_connector::connector<arduino_msgs::distance>* distr = wrapper_conn<arduino_msgs::distance>("arduino_sensor_getDistanceSensorRight", func_arduino_distance);

    auto s14 = distl->subscribe(n, "/arduino_sensor/GetDistanceSensorLeft");
    auto s15 = distr->subscribe(n, "/arduino_sensor/GetDistanceSensorRight");


    ros::AsyncSpinner spinner = ros::AsyncSpinner(0);
    spinner.start();

    ros::Rate r(10);
    ros::ServiceClient lsuc = n.serviceClient<dobot_msgs::GetEndEffectorSuctionCup>("/Dobot_Loader/GetEndEffectorSuctionCup");
    ros::ServiceClient rsuc = n.serviceClient<dobot_msgs::GetEndEffectorSuctionCup>("/Dobot_Rail/GetEndEffectorSuctionCup");
    ros::ServiceClient lin = n.serviceClient<dobot_msgs::GetPoseL>("/Dobot_Rail/GetPoseL");

    dobot_msgs::GetEndEffectorSuctionCup msg;
    dobot_msgs::GetPoseL posel;


    // services for suction cup
    while(ros::ok()){
        ROS_INFO_STREAM("Service Loop");
        if(lsuc.call(msg)){
            influxdb::api::key_value_pairs tags = influxdb::api::key_value_pairs(std::string("msg_type"), std::string("dobot_msgs/GetEndEffectorSuctionCup"));
            influxdb::api::key_value_pairs fields = influxdb::api::key_value_pairs();

            fields.add(std::string("suck"), msg.response.suck);
            influx_db.insert(influxdb::api::line(std::string("dobot_loader_GetEndEffectorSuctionCup"), tags, fields, influxdb::api::default_timestamp()));
        }
        if(rsuc.call(msg)){
            influxdb::api::key_value_pairs tags = influxdb::api::key_value_pairs(std::string("msg_type"), std::string("dobot_msgs/GetEndEffectorSuctionCup"));
            influxdb::api::key_value_pairs fields = influxdb::api::key_value_pairs();

            fields.add(std::string("suck"), msg.response.suck);
            influx_db.insert(influxdb::api::line(std::string("dobot_rail_GetEndEffectorSuctionCup"), tags, fields, influxdb::api::default_timestamp()));
        }
        if(lin.call(posel)){
            influxdb::api::key_value_pairs tags = influxdb::api::key_value_pairs(std::string("msg_type"), std::string("dobot_msgs/GetPoseL"));
            influxdb::api::key_value_pairs fields = influxdb::api::key_value_pairs();

            fields.add(std::string("position"), posel.response.l);
            influx_db.insert(influxdb::api::line(std::string("dobot_rail_linear_rail"), tags, fields, influxdb::api::default_timestamp()));
        }
        r.sleep();
    }

    spinner.stop();
}



int main(int argc, char** argv) {
    flag_scenario(argc, argv);
}



