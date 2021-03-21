using RosSharp.RosBridgeClient;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class SuctionCupStatusSubscriber : MonoBehaviour
{
    public bool suctionStatus = false;
    public List<CollisionTester> collisionTesters = new List<CollisionTester>();
    public TextMeshProUGUI textMeshPro;
    public GoalIDSubscriber goalIDSubscriber;

    void Update()
    {
        if (textMeshPro != null)
        {
            textMeshPro.text = "" + this.suctionStatus;
        }
        this.suctionStatus = goalIDSubscriber.parsedValue;
        foreach (CollisionTester cT in collisionTesters)
            cT.sucker_message_arrived(suctionStatus);
    }
}


