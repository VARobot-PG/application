using RosSharp.RosBridgeClient;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ToggleMarkerHandler : MonoBehaviour
{
    MarkerHandler markerHandler;
    public bool defaultState = false;
   public void toggleMarkerHandler()
    {
        markerHandler.showGizmos = !markerHandler.showGizmos;
    }
    // Start is called before the first frame update
    /*
    void Start()
    {
        markerHandler.showGizmos = this.defaultState;
    }
    */
    // Update is called once per frame
    void Update()
    {
        
    }
}
