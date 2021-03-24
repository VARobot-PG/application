using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlaceScenario : MonoBehaviour
{
    public GameObject scenario;
    private bool placed = false;
    void Update()
    {
        if(GameObject.Find("ScenarioPlacer") != null && !placed)
        {
            GameObject placer = GameObject.Find("ScenarioPlacer");
            scenario.transform.position = placer.transform.position;
            scenario.transform.rotation = placer.transform.rotation;
            scenario.SetActive(true);

            Transform plane = scenario.transform.Find("Plane_");
            if(plane == null)
            {
                Debug.Log("Plane_ is null");
                GameObject planeObject = GameObject.Find("Plane");
                Debug.Log("Plane is null: " + (plane == null));
                plane = planeObject.transform;
            }
            Debug.Log("Plane:" + plane);
            plane.eulerAngles = new Vector3(0, plane.eulerAngles.y, 0);
            //scenario.SetActive(true);
            placed = true;
        }
    }
}
