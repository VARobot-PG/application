//
// Created by lars on 3/28/19.
//
#include <ros/ros.h>
#include <ros/spinner.h>
#include <memory>
#include <dobot_msgs/block_rotation.h>
#include <chrono>
#include <cmath>
#include <vector>
#include <algorithm>
#include <numeric>

#include <arduino/arduino.h>
#include <dobot/dobot.h>

/**
 * holds all the data of a single measurement.
 * measurement consists of:
 * - time stamp of the measurement
 * - distance from sensor to block
 */
struct Measurement
{
    uint8_t dist;
    std::chrono::time_point<std::chrono::high_resolution_clock> time_point;

    Measurement(uint8_t dist, std::chrono::time_point<std::chrono::high_resolution_clock> &time_point)
            : dist(dist), time_point(time_point) {}

    /**
     * overloading operator< to be able to use std::sort
     * @param rhs - right hand side of operator <
     * @return true iff this measurement's distance is less than rhs's distance
     */
    bool operator<(const Measurement& rhs)
    {
        return dist < rhs.dist;
    }

    /**
     * overlading operator== to be able to use std::find
     * @param rhs - right hand side of the operator ==
     * @return true iff both Measurements are equal in time stamp and distance
     */
    bool operator==(const Measurement& rhs)
    {
        return dist == rhs.dist && time_point == rhs.time_point;
    }
};

/**
 * arduino sensor node
 */
std::unique_ptr<arduino::SensorNode> sensor_node;

/**
 * arduino conveyor node
 */
std::unique_ptr<arduino::ConveyorNode> conveyor_node;

/**
 * lower status code threshold. status codes 10-13 indicate error due to measurement time out.
 */
const uint8_t LOWER_DIST_UNBOUNDED = 10;


/**
 * the maximum range the sensor could possibly read in our scenario
 * used to detect erroneus measurements
 */
const uint8_t MAX_RANGE = 105;

/**
 * constant for deleting first n and last n measurements of a vector.
 * necessary due to adjustment time of the distance sensor.
 */
const size_t ADJUSTMENT_TIME = 4;

/**
 * refresh rate for publishing and measuring
 */
const int REFRESH_RATE = 60;

/**
 * if measurement is currently taken
 */
bool taking_measurement = false;

/**
 * callback for if sensor being called by conveyor_node
 * @param msg - the message being forwarded by the conveyor_node
 */
void if_callback(const std_msgs::BoolConstPtr &msg)
{
    if (!msg->data)
    {
        int8_t speed = taking_measurement ?
                       dobot::env::CONVEYORBELT_MEASUREMENT_SPEED_IN_PERCENT
                                          : dobot::env::CONVEYORBELT_SPEED_IN_PERCENT;
        if(conveyor_node->get_transportbelt_speed() != speed)
            conveyor_node->set_transportbelt_speed(speed);
    }
    else if(conveyor_node->get_transportbelt_speed() != 0)
        conveyor_node->set_transportbelt_speed(0);
}

/**
 * preprocesses a given measurement series by removing measurements taken while sensor was adjusting
 * and removing one side of the dice (since two sides were measured).
 * @param v - the measurement series to preprocess
 */
void preprocess_measurement(std::vector<Measurement> &v)
{
    uint8_t min_dist = std::numeric_limits<uint8_t>::max();
    int min_dist_i = 0;

    //delete first and last entries since sensor only produces crap then
    for(size_t i=0; i<ADJUSTMENT_TIME; i++)
    {
        v.erase(v.begin());
        v.erase(v.end());
    }

    //delete every erroneus measurement due to exceeding maximum sensing range in scenario
    for(size_t i=0; i<v.size(); i++)
    {
        if(v[i].dist > MAX_RANGE)
        {
            v.erase(v.begin()+i);
            i -= 1;
        }
    }

    //find min dist to erase slope of other side
    for(size_t i=0; i<v.size(); i++)
    {
        if(v[i].dist < min_dist)
        {
            min_dist = v[i].dist;
            min_dist_i = i;
        }
    }

    v.erase(v.begin()+min_dist_i+1, v.end());
}


/**
 * prototype declaration of value_of function.
 * Returns the value of a given type.
 * @tparam T - type to get the value from
 * @param m - instance of the type to get the value from
 * @return the value of this type as double
 */
template<typename T>
double value_of(T m);


/**
 * template specialization for type Measurement.
 * Returns the distance value of a measurement as double.
 * @param m - the measurement to get the value from
 * @return distance of the measurement
 */
template<>
double value_of<Measurement>(Measurement m)
{
    return m.dist;
}


/**
 * template sepcialization for type double.
 * Returns the the double value itself.
 * @param m - the value to return
 * @return the value of m
 */
template<>
double value_of<double>(double m)
{
    return m;
}


/**
 * computes the standard deviation of a given vector of objects.
 * @tparam T - the type of the objects in the vector
 * @param v - the vector of elements
 * @return the standard deviation of the series
 */
template<typename T>
double standard_deviation(const std::vector<T> &v)
{
    double sum = std::accumulate(v.begin(), v.end(), 0.0, [](double sum, const T& curr) { return sum + value_of<T>(curr);});
    double mean = sum / v.size();

    std::vector<double> diff(v.size());
    std::transform(v.begin(), v.end(), diff.begin(), [mean](const T& curr) { return value_of<T>(curr) - mean; });
    double sq_sum = std::inner_product(diff.begin(), diff.end(), diff.begin(), 0.0);

    return std::sqrt(sq_sum / v.size());
}


/**
 * filters all elements that differ greater than the standard deviation from the median.
 * @tparam T - type of the elements of v
 * @param v - series to filter
 */
template<typename T>
void filter(std::vector<T> &v)
{
    std::vector<T> h(v);
    std::sort(h.begin(), h.end());

    double median = 0;
    double std_dev = standard_deviation(v);

    //compute median
    if(h.size() % 2 != 0)
        median = value_of<T>(h[h.size()/2]);
    else
        median = (value_of<T>(h[h.size()/2]) + value_of<T>(h[h.size()/2+1])) / 2;

    //delete inaccurrate measurements
    for(auto m : h)
    {
        if(abs(value_of<T>(m) - median) > std_dev)
            v.erase(std::find(v.begin(), v.end(), m));
    }
}

/**
 * Computes the slope between measurement i and i+1 for all 0 <= i < n-1.
 * Returns the slopes in the slopes vector.
 * @param v - vector of measurements to compute the slopes for
 * @param slopes - output vector containing all slopes
 */
void compute_slopes(const std::vector<Measurement> &v, std::vector<double> &slopes)
{
    for(int i=0; i<v.size()-1; i++)
    {
        Measurement m0 = v[i], m1 = v[i+1];
        double delta_y = std::chrono::duration_cast<std::chrono::nanoseconds>(m1.time_point - m0.time_point).count() / pow(10,9);
        delta_y *= dobot::env::CONVEYORBELT_MEASUREMENT_SPEED;
        slopes.push_back((m1.dist - m0.dist) / delta_y);
    }
}

/**
 * computes the rotation of the block from the slopes of the left and right side.
 * @param slope_l - the slope of the left side of the block
 * @param slope_r - the slope of the right side of the block
 * @return the rotation of the block in degrees
 */
double compute_rotation_angle(const double slope_l, const double slope_r)
{
    //rotation in degrees
    double rotation_l = std::atan(slope_l) * 180 / M_PI;
    double rotation_r = std::atan(slope_r) * 180 / M_PI;

    ROS_INFO("left rotation: %f,\t right rotation: %f", rotation_l, rotation_r);

    //return mean
    return (rotation_l+rotation_r) / 2;
}

/**
 * computes the mean value of a vector of doubles.
 * @param v - the vector holding the values for mean computation
 * @return the mean of the vector
 */
double mean(std::vector<double> &v)
{
    return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
}

/**
 * Measures a new block. Returns false if no block was found.
 * @param left_side - a vector of Measurements decribing the left side of the block
 * @param right_side - a vector of Measurements desribing the right side of the block
 * @param min_dist_l - the minimm distance of the left side to the sensor
 * @param min_dist_r - the minimum distance of the right side to the sensor
 * @return true iff a block has been measured
 */
bool take_measurement(std::vector<Measurement>& left_side, std::vector<Measurement>& right_side,
                        uint8_t& min_dist_l, uint8_t& min_dist_r)
{
    uint8_t dist_l, status_l, dist_r, status_r;
    std::chrono::time_point<std::chrono::high_resolution_clock> time;

    sensor_node->read_l_dist_sensor(dist_l, status_l);
    sensor_node->read_r_dist_sensor(dist_r, status_r);
    time = std::chrono::high_resolution_clock::now();

    //no block found
    if(status_l >= LOWER_DIST_UNBOUNDED && status_r >= LOWER_DIST_UNBOUNDED)
        return false;

    taking_measurement = true;

    ROS_INFO("New Block detected!");

    left_side.push_back(Measurement(dist_l, time));
    right_side.push_back(Measurement(dist_r, time));

    ros::Rate r(REFRESH_RATE);
    do {
        sensor_node->read_l_dist_sensor(dist_l, status_l);
        sensor_node->read_r_dist_sensor(dist_r, status_r);
        time = std::chrono::high_resolution_clock::now();

        if(status_l < LOWER_DIST_UNBOUNDED)
        {
            if(dist_l < min_dist_l)
                min_dist_l = dist_l;

            left_side.push_back(Measurement(dist_l, time));
        }

        if(status_r < LOWER_DIST_UNBOUNDED)
        {
            if(dist_r < min_dist_r)
                min_dist_r = dist_r;

            right_side.push_back(Measurement(dist_r, time));
        }
        r.sleep();
    }while((status_l < LOWER_DIST_UNBOUNDED || status_r < LOWER_DIST_UNBOUNDED || min_dist_l == 255 || min_dist_r == 255)
            && ros::ok());

    taking_measurement = false;

    return true;
}

/**
 * computes the rotation of a block and fills it into the message.
 * @param msg - the message that will contain the rotation information
 */
void compute_rotation(dobot_msgs::block_rotation& msg)
{
    uint8_t min_dist_l = 255, min_dist_r = 255;
    std::vector<Measurement> left_side, right_side;

    //measure a new block if present. if no block is found, return from this method
    if(!take_measurement(left_side, right_side, min_dist_l, min_dist_r))
    {
        if(msg.new_measurement)
            ROS_WARN("------------------------SPACER--------------------");

        msg.new_measurement = false;
        return;
    }
    ROS_INFO("%zu, \t %zu",left_side.size(), right_side.size());

    for(auto d : left_side)
        std::cout << +d.dist << "\t";
    std::cout << std::endl;
    for(auto d : right_side)
        std::cout << +d.dist << "\t";
    std::cout << std::endl;
    std::vector<double> slopes_l;
    std::vector<double> slopes_r;
    preprocess_measurement(left_side);
    preprocess_measurement(right_side);
    filter(left_side);
    filter(right_side);
    for(auto d : left_side)
        std::cout << +d.dist << "\t";
    std::cout << std::endl;
    for(auto d : right_side)
        std::cout << +d.dist << "\t";
    std::cout << std::endl;
    compute_slopes(left_side, slopes_l);
    compute_slopes(right_side, slopes_r);
    filter(slopes_l);
    filter(slopes_r);
    double mean_l = mean(slopes_l);
    double mean_r = mean(slopes_r);

    for(auto d : slopes_l)
        std::cout << d << "\t";
    std::cout << std::endl;
    for(auto d : slopes_r)
        std::cout << d << "\t";

    std::cout << std::endl;

    std::cout << "mean slope_l: " << mean_l << std::endl;
    std::cout << "mean slope_r: " << mean_r << std::endl;

    msg.min_dist_l = min_dist_l;
    msg.min_dist_r = min_dist_r;
    //msg.max_dist_l = max_dist_l;
    //msg.max_dist_r = max_dist_r;
    msg.rotation = compute_rotation_angle(mean_l, mean_r);
    msg.new_measurement = true;

    ROS_INFO("Measured new block! min_dist_l = %u, max_dist_l = %u\n\t min_dist_r = %u, max_dist_r = %u\n\t rotation = %f,"
             " new_measurement = %u!", msg.min_dist_l, msg.max_dist_l, msg.min_dist_r, msg.max_dist_r,
             msg.rotation, msg.new_measurement);
}

/**
 * publishes the rotation message via the given publisher
 * @param pub - the publisher to publish the message
 * @param rotation_msg - the message to be published
 */
void publish_block_rotation(ros::Publisher& pub, dobot_msgs::block_rotation &rotation_msg)
{
    compute_rotation(rotation_msg);
    pub.publish(rotation_msg);
    //rotation_msg.new_measurement = false;
}

/**
 * main control loop
 * @param argc - unused
 * @param argv - unused
 * @return 0 iff execution ended normally
 */
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "BlockRotationDetector");
    ros::NodeHandle nh;
    std::shared_ptr<ros::NodeHandle> nh_ptr = std::make_shared<ros::NodeHandle>(nh);
    sensor_node = std::make_unique<arduino::SensorNode>(nh_ptr);
    conveyor_node = std::make_unique<arduino::ConveyorNode>(nh_ptr);
    conveyor_node->register_if_callback(if_callback);

    ros::Publisher rotation_pub = nh.advertise<dobot_msgs::block_rotation>("/BlockRotation", 1);
    dobot_msgs::block_rotation rotation_msg;

    //async spinner
    ros::AsyncSpinner spinner(0);
    spinner.start();

    ros::Rate r(REFRESH_RATE); //60Hz refresh rate
    //sleep for a second to wait for valid published results
    ros::Duration(1).sleep();

    while(ros::ok())
    {
        publish_block_rotation(rotation_pub, rotation_msg);
        r.sleep();
    }

    spinner.stop();

    ros::waitForShutdown();
}
