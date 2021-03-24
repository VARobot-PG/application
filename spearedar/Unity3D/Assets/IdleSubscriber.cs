
using RosSharp.RosBridgeClient;
using UnityEngine;

public class IdleSubscriber : MonoBehaviour
{
    public bool idle = false;
    public GoalIDSubscriber goalIDSubscriber;
    public bool isIdle()
    {
        return idle;
    }
    void Update()
    {
        this.idle = goalIDSubscriber.parsedValue;   
    }

}
