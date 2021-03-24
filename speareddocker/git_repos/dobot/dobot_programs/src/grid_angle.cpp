//
// Created by speeters on 14.08.19.
//

#include <ros/ros.h>
#include <tf_helper/tfHelper.h>
#include <dobot/dobot.h>
#include <math.h>

int main(int argc, char* argv[]){
    ros::init(argc, argv, "gridangle");
    std::shared_ptr<ros::NodeHandle> handle = std::make_shared<ros::NodeHandle>();

    std::shared_ptr<TfHelper> tfHelper = std::make_unique<TfHelper>();

    dobot::Dobot dob(dobot::dobot_names::DOBOT_RAIL);
    dob.initArm(handle, dobot::dobot_names::DOBOT_RAIL);
    ros::Duration(1).sleep();
    ros::spinOnce();




    ros::Rate r(1);
    while(ros::ok()){

        tf2::Quaternion quat;
        quat.setRPY(0,0,0);

        tfHelper->transformQuaternion("gridDetection","world",ros::Time(0), quat);
        tf2::Matrix3x3 m(quat);

        double roll,pitch,yaw;
        m.getRPY(roll, pitch, yaw);

        yaw = (yaw * (double) 180) / M_PI + 90;

        std::cout << (yaw) << std::endl;

        tf2::Vector3 vec(200,0,0);
        //dob.moveToPositionWithLBlocking(vec, yaw, 400);

        ros::spinOnce();
        r.sleep();
    }

    return 0;

}
