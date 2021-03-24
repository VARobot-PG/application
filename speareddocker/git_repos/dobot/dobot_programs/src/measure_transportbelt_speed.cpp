//
// Created by lars on 04.04.19.
//

#include <ros/ros.h>
#include <memory>
#include <chrono>
#include <arduino/arduino.h>

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "MeasureTransportbeltSpeed");
    ros::NodeHandle nh;
    std::shared_ptr<ros::NodeHandle> nh_ptr = std::make_shared<ros::NodeHandle>(nh);

    arduino::ConveyorNode node(nh_ptr);
    ros::spinOnce();

    std::chrono::time_point<std::chrono::high_resolution_clock> start, end;

    start = std::chrono::high_resolution_clock::now();
    node.set_transportbelt_speed(15);

    //wait for block to reach if sensor
    while(ros::ok() && !node.is_if_sensor_blocked())
    {
        ros::spinOnce();
    }

    end = std::chrono::high_resolution_clock::now();

    int64_t duration = std::chrono::duration_cast<std::chrono::nanoseconds>(end-start).count();
    double duration_in_seconds = duration * pow(10.0, -9.0);

    std::cout << "velocity: " << 300 / duration_in_seconds << " mm/s" << std::endl;

    return 0;
}
