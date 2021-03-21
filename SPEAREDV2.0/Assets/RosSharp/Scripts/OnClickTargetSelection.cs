using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class OnClickTargetSelection : MonoBehaviour
{
    public CoordinateTargetSelector coordinateTargetSelector;
    public InputField textField; //the text field where the coordinates should be

    public void onClickSelectTarget()
    {
        //Debug.Log("Is null: "+coordinateTargetSelector.selectedRosTransform);
        if (coordinateTargetSelector.selectedRosTransform != null)
        {
            Transform rosCoords = coordinateTargetSelector.selectedRosTransform;
            //Debug.Log("RosCoords: " + rosCoords.localPosition.ToString());
            //Debug.Log("Text: " + textField.text);
            textField.text = ""+rosCoords.localPosition.ToString();
        }
    }

    public void onClickSelectTargetNonROS()
    {
        //Debug.Log("Is null: "+coordinateTargetSelector.selectedRosTransform);
        if (coordinateTargetSelector.selectedRosTransform != null)
        {
            Transform ownCoords = GameObject.Find("RealTargetSphere").transform;
            textField.text = ownCoords.localPosition.ToString();
        }
    }
}
