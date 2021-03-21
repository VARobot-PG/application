using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestInstantiateRobot1 : MonoBehaviour
{

    public GameObject Target;


    void Update()
    {
        
        {
            if (Input.touchCount > 0 && Input.GetTouch(0).phase == TouchPhase.Began)
            {
                Vector3 fingerPos = Input.GetTouch(0).position;
                fingerPos.z = 3;
                Vector3 objPos = Camera.main.ScreenToWorldPoint(fingerPos);

                Target.SetActive(true);

            }

        }
    }
        
}
