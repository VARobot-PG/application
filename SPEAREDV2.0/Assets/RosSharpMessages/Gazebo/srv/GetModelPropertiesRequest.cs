/* 
 * This message is auto generated by ROS#. Please DO NOT modify.
 * Note:
 * - Comments from the original code will be written in their own line 
 * - Variable sized arrays will be initialized to array of size 0 
 * Please report any issues at 
 * <https://github.com/siemens/ros-sharp> 
 */

using Newtonsoft.Json;

namespace RosSharp.RosBridgeClient.MessageTypes.Gazebo
{
    public class GetModelPropertiesRequest : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "gazebo_msgs/GetModelProperties";

        public string model_name;
        //  name of Gazebo Model

        public GetModelPropertiesRequest()
        {
            this.model_name = "";
        }

        public GetModelPropertiesRequest(string model_name)
        {
            this.model_name = model_name;
        }
    }
}
