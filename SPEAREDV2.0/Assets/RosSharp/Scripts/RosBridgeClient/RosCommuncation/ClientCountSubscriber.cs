
using RosSharp.RosBridgeClient.MessageTypes.Std;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
namespace  RosSharp.RosBridgeClient{
public class ClientCountSubscriber : UnitySubscriber<MessageTypes.Std.Int32>
{
    public TextMeshProUGUI textMeshPro;
        public int messageData;
        protected override void ReceiveMessage(Int32 message)
        {
            messageData = message.data;
        }

        void Update()
    {
        if (textMeshPro != null)
        {
            textMeshPro.text = "" + this.messageData;
        }
    }
}
}
