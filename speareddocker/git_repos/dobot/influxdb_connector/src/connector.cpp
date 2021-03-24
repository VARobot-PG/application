//
// Created by speeters on 14.03.19.
//


#include "../include/connector.h"

namespace influxdb_connector {
    namespace functions {
        /**
       * Connector which maps the message to the datastructure used by the influxdb_cpp api
       * @param msg message with data
       * @param will be filled with the measurement name
       * @return line that will be inserted
       */
        influxdb::api::line geometry_msgs_pose(geometry_msgs::Pose msg, std::string measurement) {
            // data
            double pos_x = msg.position.x;
            double pos_y = msg.position.y;
            double pos_z = msg.position.z;

            double or_x = msg.orientation.x;
            double or_y = msg.orientation.y;
            double or_z = msg.orientation.z;
            double or_w = msg.orientation.w;

            // create key value pairs for the tag set and the field set
            influxdb::api::key_value_pairs tags("msg_type", "geometry_msgs/Pose");
            influxdb::api::key_value_pairs fields("position_x", pos_x);
            fields.add("position_y", pos_y);
            fields.add("position_z", pos_z);
            fields.add("orientation_x", or_x);
            fields.add("orientation_y", or_y);
            fields.add("orientation_z", or_z);
            fields.add("orientation_w", or_w);

            // create line and return it
            return influxdb::api::line(measurement, tags, fields, influxdb::api::default_timestamp());
        }

        influxdb::api::line std_msgs_bool(std_msgs::Bool msg, std::string measurement) {
            //data
            bool data = msg.data;

            influxdb::api::key_value_pairs tags(std::string("msg_type"), std::string("std_msgs/Bool"));

            influxdb::api::key_value_pairs fields(std::string("data"), data ? 1 : 0);

            // create line and return it

            return influxdb::api::line(measurement, tags, fields, influxdb::api::default_timestamp());
        }

        influxdb::api::line std_msgs_float64(std_msgs::Float64 msg, std::string measurement){
            double data = msg.data;
            influxdb::api::key_value_pairs tags = influxdb::api::key_value_pairs(std::string("msg_type"), std::string("std_msgs/Float64"));
            influxdb::api::key_value_pairs fields = influxdb::api::key_value_pairs(std::string("data"), data);

            return influxdb::api::line(measurement, tags, fields, influxdb::api::default_timestamp());
        }

        influxdb::api::line arduino_msgs_distance(arduino_msgs::distance msg, std::string measurement) {
            //data
            int dist = msg.dist;
            int status = msg.status;

            influxdb::api::key_value_pairs tags(std::string("msg_type"), std::string("arduino_msgs/distance"));
            influxdb::api::key_value_pairs fields(std::string("distance"), dist);
            fields.add("status", status);

            return influxdb::api::line(measurement, tags, fields, influxdb::api::default_timestamp());
        }

        influxdb::api::line arduino_msgs_rgb_color(arduino_msgs::rgb_color msg, std::string measurement) {
            //data
            int r = msg.r;
            int g = msg.g;
            int b = msg.b;
            int c = msg.c;

            influxdb::api::key_value_pairs tags(std::string("msg_type"), std::string("arduino_msgs/rgb_color"));
            influxdb::api::key_value_pairs fields(std::string("r"), r);
            fields.add("g", g);
            fields.add("b", b);
            fields.add("c", c);

            return influxdb::api::line(measurement, tags, fields, influxdb::api::default_timestamp());
        }


        influxdb::api::line sensor_msgs_joint_state(sensor_msgs::JointState msg, std::string measurement){
            influxdb::api::key_value_pairs tags = influxdb::api::key_value_pairs(std::string("msg_type"), std::string("sensor_msgs/JointState"));
            influxdb::api::key_value_pairs fields = influxdb::api::key_value_pairs();
            // data
            for(int i = 0; i < 4; i++){
                std::string i_str = std::to_string(i);
                fields.add(std::string("name").append(i_str), msg.name[i]);
                fields.add(std::string("position").append(i_str), msg.position[i]);
                fields.add(std::string("velocity").append(i_str), msg.velocity[i]);
                fields.add(std::string("effort").append(i_str), msg.effort[i]);
            }
            return influxdb::api::line(measurement, tags, fields, influxdb::api::default_timestamp());
        }


        influxdb::api::line detected_objects(detection_msgs::DetectedObjects msg, std::string measurement){

            influxdb::api::line lines;



            for(detection_msgs::DetectedObject obj : msg.objects){
                if(!(obj.timestamp == ros::Time(0))){
                    influxdb::api::key_value_pairs tags = influxdb::api::key_value_pairs(std::string("msg_type"), std::string("detection_msgs/DetectedObjects"));
                    influxdb::api::key_value_pairs fields = influxdb::api::key_value_pairs();

                    fields.add(std::string("startpoint_x"), obj.startPoint.x);
                    fields.add(std::string("startpoint_y"), obj.startPoint.y);
                    fields.add(std::string("startpoint_z"), obj.startPoint.z);

                    fields.add(std::string("endpoint_x"), obj.endPoint.x);
                    fields.add(std::string("endpoint_y"), obj.endPoint.y);
                    fields.add(std::string("endpoint_z"), obj.endPoint.z);

                    fields.add(std::string("grasppoint_x"), obj.graspPoint.x);
                    fields.add(std::string("grasppoint_y"), obj.graspPoint.y);
                    fields.add(std::string("grasppoint_z"), obj.graspPoint.z);

                    fields.add(std::string("rotation_x"), obj.rotation.x);
                    fields.add(std::string("rotation_y"), obj.rotation.y);
                    fields.add(std::string("rotation_z"), obj.rotation.z);
                    fields.add(std::string("rotation_w"), obj.rotation.w);

                    fields.add(std::string("color_r"), obj.color[0]);
                    fields.add(std::string("color_g"), obj.color[1]);
                    fields.add(std::string("color_b"), obj.color[2]);

                    fields.add(std::string("probability"), obj.probability);

                    fields.add(std::string("msg_timestamp"), obj.timestamp.sec);
                    lines(measurement, tags, fields, influxdb::api::default_timestamp());
                }
            }
            return lines;
        }

        influxdb::api::line detected_grid(detection_msgs::DetectedGrid msg, std::string measurement){
            influxdb::api::key_value_pairs tags = influxdb::api::key_value_pairs(std::string("msg_type"), std::string("detection_msgs/DetectedGrid"));
            influxdb::api::key_value_pairs fields = influxdb::api::key_value_pairs();

            fields.add(std::string("basepoint_x") , msg.basePoint.x);
            fields.add(std::string("basepoint_y") , msg.basePoint.y);
            fields.add(std::string("basepoint_z") , msg.basePoint.z);

            fields.add(std::string("linevector_x") , msg.lineVector.x);
            fields.add(std::string("linevector_y") , msg.lineVector.y);

            fields.add(std::string("msg_timestamp") , msg.timestamp.sec);

            return influxdb::api::line(measurement, tags, fields, influxdb::api::default_timestamp());

        }

        influxdb::api::line arduino_msgs_rfid(arduino_msgs::rfid msg, std::string measurement){
            influxdb::api::key_value_pairs tags = influxdb::api::key_value_pairs(std::string("msg_type"), std::string("arduino_msgs/rfid"));
            influxdb::api::key_value_pairs fields = influxdb::api::key_value_pairs();

            fields.add(std::string("b0"), msg.b0);
            fields.add(std::string("b1"), msg.b1);
            fields.add(std::string("b2"), msg.b2);
            fields.add(std::string("b3"), msg.b3);

            return influxdb::api::line(measurement, tags, fields, influxdb::api::default_timestamp());
        }

        influxdb::api::line arduino_msgs_get_velocity(arduino_msgs::get_velocity msg, std::string measurement) {
            //data
            int dist = msg.velocity;

            influxdb::api::key_value_pairs tags(std::string("msg_type"), std::string("arduino_msgs/get_velocity"));
            influxdb::api::key_value_pairs fields(std::string("velocity"), dist);

            return influxdb::api::line(measurement, tags, fields, influxdb::api::default_timestamp());
        }
    }

}