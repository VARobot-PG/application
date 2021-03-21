using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ActivityToggle : MonoBehaviour
{
    /*
     * This script can be added to any button to make it able to control the visibility (GameObject.active) 
     * of a GameObject. It also has a function for toggling the color of a button in response to this visibility change. 
     * 
     * To use it (for toggling visibility): 
     * 1. Add the ActivityToggle script to your button
     * 2. Select is as source of the onclick method for the button
     * 3. Select the ToggleGameObject method as the method to execute
     * 4. Add the GameObject of which you want to toggle the vibility as 
     *    the objectToToggle for this script by dragging it into the respective field in the editor.
     *    
     * To use the color change:
     * Do 1. and 2. as above (with a new onclick method)
     * 3. Add the Image of the button of which you want to change the colour as the imageForColour 
     *    for this script by dragging it into the respective field in the editor.
     */
    public GameObject objectToToggle;
    public Image imageForColour;
    public Color activeButtonColour = Color.grey;   //colour if button is active
    public Color inactiveButtonColor = Color.white; //colour if button is not active

    public void ToggleGameObject()
    {
        try
        {
            objectToToggle.SetActive(!objectToToggle.activeInHierarchy);
        }
        catch (System.NullReferenceException)
        {
            Debug.LogWarning("You are trying to toggle the visibility of an object which is not referenced. You might have forgotten to add a GameObject as the objectToToggle in the ActivityToggle script.");
        }
    }

    public void toggleColor()
    {
        try
        {
            if (objectToToggle.activeInHierarchy)
            {
                imageForColour.color = activeButtonColour; //colour if button is active
            }
            else
            {
                imageForColour.color = inactiveButtonColor; //colour if button is not active
            }
        }
        catch (System.NullReferenceException)
        {
            Debug.LogWarning("You are change the colour of an object which is not referenced. You might have forgotten to add an Image as the imageForColour in the ActivityToggle script.");
        }
    }
}

