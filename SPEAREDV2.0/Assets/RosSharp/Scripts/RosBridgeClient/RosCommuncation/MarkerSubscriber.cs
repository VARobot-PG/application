using RosSharp.RosBridgeClient.MessageTypes.Visualization;
using UnityEngine;
namespace RosSharp.RosBridgeClient
{
    public class MarkerSubscriber : UnitySubscriber<Marker>
    {
        public MarkerHandler myHandler;
        public MarkerSubscriber()
        {

        }
        protected override void Start()
        {
            base.Start();
        }


       

        protected override void ReceiveMessage(Marker marker)
        {
            myHandler.receiveMarker(marker);
        }
    }
}
