using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class GlobalCoordinateSetter : MonoBehaviour
{ public TextMesh textField;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    /* On each update write the coordinates of the object this is attached to into the given text field. Coordinates are written as robot coordinates, not Unity coordinates. */
    void Update()
    {
        Vector3 officialPosition = this.transform.position;
        Vector3 unityPosition = this.transform.position;
        GameObject zero = GameObject.Find("AbsoluteZero");
        if (zero != null)
        {
            GameObject go = new GameObject("GO DragOnFloorReciever");
            go.transform.position = unityPosition;
            go.transform.parent = zero.transform;
            Vector3 tmpPos = go.transform.localPosition;
            officialPosition = new Vector3(tmpPos.z, tmpPos.y, tmpPos.x);
            Destroy(go);
        }
        else
        {
            Debug.Log("AbsoluteZero not found");
        }
        textField.text = String.Format("({0}, {1}, {2})", Math.Round(officialPosition.x, 2), Math.Round(officialPosition.y, 2), Math.Round(officialPosition.z, 2));
    }
}
