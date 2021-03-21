using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AddCoordinateTargetSelector : MonoBehaviour
{
    
    void Start()
    {
        //find the (only) CoordinateTargetSelector by searching in the scene and add it to the taargetselector and OnClickTargetSelection
        TargetSelectorUI tsu = this.GetComponentInParent<TargetSelectorUI>();
        tsu.coordinateTargetSelector = Resources.FindObjectsOfTypeAll<CoordinateTargetSelector>()[0];
        OnClickTargetSelection octs = this.GetComponentInParent<OnClickTargetSelection>();
        octs.coordinateTargetSelector = Resources.FindObjectsOfTypeAll<CoordinateTargetSelector>()[0];
    }
}
