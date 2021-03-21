using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ResetDobot : MonoBehaviour
{
    public Vector3 initialPosition = new Vector3(215, 0, 145);
    public Slider sliderPTP;
    public Slider sliderX;
    public Slider sliderY;
    public Slider sliderZ;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    public void resetDobotPosition()
    {
        resetDobotPosition(initialPosition);
    }
    public void resetDobotPosition(Vector3 newPosition)
    {

        sliderPTP.value = 1;
        sliderX.value = newPosition.x;
        sliderY.value = newPosition.y;
        sliderZ.value = newPosition.z;
    }
}
