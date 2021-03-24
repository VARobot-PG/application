using Microsoft.MixedReality.Toolkit.UI;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ToggleBoundingBox : MonoBehaviour
{
    public BoundingBox boundingBox;


    public void ToggleBoundingBoxActiveState()
    {
        // Do something on specified distance for fire event
        if (boundingBox != null)
        {
            boundingBox.Active = !boundingBox.Active;
        }

    }
}
