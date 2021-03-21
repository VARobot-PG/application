using Microsoft.MixedReality.Toolkit.UI;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;


public class ToggleManipulationHandler : MonoBehaviour
{
    public ManipulationHandler manipulationHandler;


    public void ToggleManipulationHandlerActiveState()
    {
        // Do something on specified distance for fire event
        if (manipulationHandler != null)
        {
            manipulationHandler.enabled = !manipulationHandler.enabled;
        }

    }
}
