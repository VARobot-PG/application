using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class OnPlane : MonoBehaviour
{
    void Update()

    {
        float planeY = 0;
        Transform RealTargetSphere = transform; 

        if (Input.touchCount == 1 && Input.GetTouch(0).phase == TouchPhase.Moved)
        {
            // create ray from the camera and passing through the touch position:

            Ray ray = Camera.main.ScreenPointToRay(Input.GetTouch(0).position);

            // create a logical plane at this object's position
            // and perpendicular to world Y:

            Plane plane = new Plane (Vector3.up, Vector3.up * planeY); // ground plane

            float distance; // this will return the distance from the camera

            if (plane.Raycast(ray, out distance))

            { // if plane hit...
                RealTargetSphere.position = ray.GetPoint(distance); // get the point
           }
        }
    }

    private Plane Instantiate(Plane plane, Vector3 up, Vector3 vector3)
    {
        throw new NotImplementedException();
    }
}