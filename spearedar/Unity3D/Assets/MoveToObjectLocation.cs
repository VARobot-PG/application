using RosSharp.RosBridgeClient;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoveToObjectLocation : MonoBehaviour
{
    public LocationObject targetObject;
    public ResetDobot resetDobot;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    
    public void updateSliders()
    {
        resetDobot.resetDobotPosition(targetObject.transformedLocation);
    }
}
