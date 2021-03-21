using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using RosSharp.RosBridgeClient.MessageTypes.Actionlib;

namespace RosSharp.RosBridgeClient
{
    public class GoalIDSubscriber : UnitySubscriber<MessageTypes.Actionlib.GoalID>
    {
        public TextMeshProUGUI textMeshPro;
        public string id;
        public MessageTypes.Std.Time stamp;
        public bool parsedValue;
        protected override void ReceiveMessage(GoalID message)
        {
            this.id = message.id;
            this.stamp = message.stamp;
            parseValues();
        }

        private void parseValues()
        {
            if (this.id.ToLower().Contains("true"))
                this.parsedValue = true;
            else
                this.parsedValue = false;
        }
        // Update is called once per frame
        void Update()
        {
            if (textMeshPro != null && this.Topic != null && this.id != null && this.stamp != null && this.stamp.nsecs != null && this.stamp.secs != null)
                textMeshPro.text = $"topic: {this.Topic} id: {this.id} stamp: nsecs {this.stamp.nsecs}  secs {this.stamp.secs}";
        }
    }
}