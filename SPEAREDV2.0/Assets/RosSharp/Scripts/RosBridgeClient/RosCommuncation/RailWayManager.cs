using System.Collections;
using System.Collections.Generic;
using UnityEngine;
namespace RosSharp.RosBridgeClient
{
    public class RailWayManager : MonoBehaviour
    {
        public bool debug_MOde = false;
        public RosConnector RosConnector;
        protected RosSocket RosSocket;
        public string ServiceEndpoint = "/Dobot_Rail/SetPTPWithLCmd";
        int i = 0;
        // Start is called before the first frame update
        void Start()
        {
            this.RosSocket = this.RosConnector.RosSocket;
         }

        // Update is called once per frame
        void Update()
        {
            if (debug_MOde)
            {
                i++;
                int j;

                if (i % 100 == 0)
                {
                    System.Random random = new System.Random(1);
                    j = random.Next(1, 999);
                    i = 0;
                    moveRail(j);
                }
            }
        }

        public void moveRail(int x)
        {
            //MessageTypes.Dobot.SetPTPLParamsRequest request=new MessageTypes.Dobot.SetPTPLParamsRequest(0.5f,0.5f,false);




            MessageTypes.Dobot.SetPTPWithLCmdRequest request = new MessageTypes.Dobot.SetPTPWithLCmdRequest(1, 217, 25, 175, 22, x, false);
            RosSocket.CallService<MessageTypes.Dobot.SetPTPWithLCmdRequest, MessageTypes.Dobot.SetPTPWithLCmdResponse>
                (ServiceEndpoint, handlerSetPTPWithLCmd, request);

        }

        public void handlerSetPTPWithLCmd(MessageTypes.Dobot.SetPTPWithLCmdResponse rep)
        {
            //for now do nothing
            Debug.Log("Answer received.");
        }



    }
}