using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class TripleToggle : MonoBehaviour
{
    public GameObject screenUI;
    public GameObject arUI;
    public bool arOn;
    public bool visible = true;


    public Image imageForColourVisibility;      // the image which decides the colour for the visibility toggle button
    public Image imageForColourAR;              // the image which decides the colour for the AR toggle button
    public Color activeButtonColour = Color.grey;   //colour if button is active
    public Color inactiveButtonColor = Color.white; //colour if button is not active

    public void ToggleVisibility()
    {
        visible = !visible;
    }

    public void toggleAR()
    {
        arOn = !arOn;
        visible = true; //if ar is toggled, we want to automatically see the UI again
    }

    private void Update()
    {
        try
        {
            if (visible)
            {
                arUI.SetActive(arOn);
                screenUI.SetActive(!arOn);
            }
            else
            {
                arUI.SetActive(false);
                screenUI.SetActive(false);
            }
            toggleColorVisibility();
            toggleColorAR();
        }
        catch (System.NullReferenceException)
        {
            Debug.LogWarning("You are trying to toggle the visibility of an object which is not referenced. You might have forgotten to add a GameObject as the objectToToggle in the ActivityToggle script.");
        }
    }

    public void toggleColorVisibility()
    {
        if (visible)
        {
            imageForColourVisibility.color = activeButtonColour; //colour if button is active
        }
        else
        {
            imageForColourVisibility.color = inactiveButtonColor; //colour if button is not active
        }
    }
    public void toggleColorAR()
    {
        if (arOn)
        {
            imageForColourAR.color = activeButtonColour; //colour if button is active
        }
        else
        {
            imageForColourAR.color = inactiveButtonColor; //colour if button is not active
        }
    }
}
