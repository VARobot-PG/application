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
    public class GetWorldPropertiesResponse : Message
    {
        [JsonIgnore]
        public const string RosMessageName = "gazebo_msgs/GetWorldProperties";

        public double sim_time;
        //  current sim time
        public string[] model_names;
        //  list of models in the world
        public bool rendering_enabled;
        //  if X is used
        public bool success;
        //  return true if get successful
        public string status_message;
        //  comments if available

        public GetWorldPropertiesResponse()
        {
            this.sim_time = 0.0;
            this.model_names = new string[0];
            this.rendering_enabled = false;
            this.success = false;
            this.status_message = "";
        }

        public GetWorldPropertiesResponse(double sim_time, string[] model_names, bool rendering_enabled, bool success, string status_message)
        {
            this.sim_time = sim_time;
            this.model_names = model_names;
            this.rendering_enabled = rendering_enabled;
            this.success = success;
            this.status_message = status_message;
        }
    }
}
