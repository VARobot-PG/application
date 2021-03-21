using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TestInstantiateRobot : MonoBehaviour
{
    public GameObject scenario;
    // Start is called before the first frame update

    private bool robottoggle = false;
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        if (Input.touchCount > 0)
        {
            // The screen has been touched so store the touch

            Touch touch = Input.GetTouch(0);

            if (touch.phase == TouchPhase.Stationary || touch.phase == TouchPhase.Moved)
            {

                if (!robottoggle)

                {
                    // If the finger is on the screen, move the object smoothly to the touch position

                    // Vector3 touchPosition = Camera.main.ScreenToWorldPoint(new Vector3(touch.position.x, touch.position.y, 0.0f));

                    // transform.position = Vector3.Lerp(transform.position, new Vector3(touchPosition.x, 0.0f, touchPosition.z), Time.deltaTime * 5.0f);

                    scenario.SetActive(true);

                Vector3 fingerPos = Input.GetTouch(0).position;
                fingerPos.z = 3;
                Vector3 objPos = Camera.main.ScreenToWorldPoint(fingerPos);

                    objPos.y = objPos.y -0.05f;


                    scenario.transform.position = objPos;

                    robottoggle = true;
                }
            }

        }
    }

    
}