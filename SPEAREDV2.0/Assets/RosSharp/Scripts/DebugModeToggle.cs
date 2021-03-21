using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class DebugModeToggle : MonoBehaviour
{

    public Image imageForColour;

    public Button firstButtonToBePressed;
    public Button secondButtonToBePressed;
    public Button thirdButtonToBePressed;
    public Button fourthButtonToBePressed;
    public Button fifthButtonToBePressed;
    public Button sixthButtonToBePressed;
    public Button seventhButtonToBePressed;


    public GameObject firstARMenuToHide;
    public GameObject firstScreenMenuToHide;
    public GameObject secondARMenuToHide;
    public GameObject secondScreenMenuToHide;
    public GameObject thirdARMenuToHide;
    public GameObject thirdScreenMenuToHide;
    public GameObject fourthARMenuToHide;
    public GameObject fourthScreenMenuToHide;
    public GameObject fifthARMenuToHide;
    public GameObject fifthScreenMenuToHide;
    public GameObject sixthARMenuToHide;
    public GameObject sixthScreenMenuToHide;
    public GameObject seventhARMenuToHide;
    public GameObject seventhScreenMenuToHide;

    public GameObject spheresToHide;

    public GameObject firstRunningModeButton;
    public GameObject secondRunningModeButton;

    void Awake()
    {
        if (firstButtonToBePressed == null)
            firstButtonToBePressed = GetComponent<Button>();
        if (secondButtonToBePressed == null)
            secondButtonToBePressed = GetComponent<Button>();
        if (thirdButtonToBePressed == null)
            thirdButtonToBePressed = GetComponent<Button>();
        if (fourthButtonToBePressed == null)
            fourthButtonToBePressed = GetComponent<Button>();
        if (fifthButtonToBePressed == null)
            fifthButtonToBePressed = GetComponent<Button>();
        if (sixthButtonToBePressed == null)
            sixthButtonToBePressed = GetComponent<Button>();
        if (seventhButtonToBePressed == null)
            seventhButtonToBePressed = GetComponent<Button>();
    }

    public void Invoke()
    {
        if (imageForColour.color == Color.white) // We want to hide all Menus
        {
            if (firstButtonToBePressed != null && firstButtonToBePressed.onClick != null)
            {
                if (firstARMenuToHide.activeInHierarchy || firstScreenMenuToHide.activeInHierarchy)
                {
                    firstButtonToBePressed.onClick.Invoke();
                }
            }
            if (secondButtonToBePressed != null && secondButtonToBePressed.onClick != null)
            {
                if (secondARMenuToHide.activeInHierarchy || secondScreenMenuToHide.activeInHierarchy)
                {
                    secondButtonToBePressed.onClick.Invoke();
                }
            }
            if (thirdButtonToBePressed != null && thirdButtonToBePressed.onClick != null)
            {
                if (thirdARMenuToHide.activeInHierarchy || thirdScreenMenuToHide.activeInHierarchy)
                {
                    thirdButtonToBePressed.onClick.Invoke();
                }
            }
            if (fourthButtonToBePressed != null && fourthButtonToBePressed.onClick != null)
            {
                if (fourthARMenuToHide.activeInHierarchy || fourthScreenMenuToHide.activeInHierarchy)
                {
                    fourthButtonToBePressed.onClick.Invoke();
                }
            }
            if (fifthButtonToBePressed != null && fifthButtonToBePressed.onClick != null)
            {
                if (fifthARMenuToHide.activeInHierarchy || fifthScreenMenuToHide.activeInHierarchy)
                {
                    fifthButtonToBePressed.onClick.Invoke();
                }
            }
            if (sixthButtonToBePressed != null && sixthButtonToBePressed.onClick != null)
            {
                if (sixthARMenuToHide.activeInHierarchy || sixthScreenMenuToHide.activeInHierarchy)
                {
                    sixthButtonToBePressed.onClick.Invoke();
                }
            }
            if (seventhButtonToBePressed != null && seventhButtonToBePressed.onClick != null)
            {
                if (seventhARMenuToHide.activeInHierarchy || seventhScreenMenuToHide.activeInHierarchy)
                {
                    seventhButtonToBePressed.onClick.Invoke();
                }
            }
            spheresToHide.SetActive(false);
            firstRunningModeButton.SetActive(true);
            secondRunningModeButton.SetActive(true);
        }
        else if (imageForColour.color == Color.grey) // We want to show all Menus
        {
            if (firstButtonToBePressed != null && firstButtonToBePressed.onClick != null)
            {
                if (!firstARMenuToHide.activeInHierarchy && !firstScreenMenuToHide.activeInHierarchy)
                {
                    firstButtonToBePressed.onClick.Invoke();
                }
            }
            if (secondButtonToBePressed != null && secondButtonToBePressed.onClick != null)
            {
                if (!secondARMenuToHide.activeInHierarchy && !secondScreenMenuToHide.activeInHierarchy)
                {
                    secondButtonToBePressed.onClick.Invoke();
                }
            }
            if (thirdButtonToBePressed != null && thirdButtonToBePressed.onClick != null)
            {
                if (!thirdARMenuToHide.activeInHierarchy && !thirdScreenMenuToHide.activeInHierarchy)
                {
                    thirdButtonToBePressed.onClick.Invoke();
                }
            }
            if (fourthButtonToBePressed != null && fourthButtonToBePressed.onClick != null)
            {
                if (!fourthARMenuToHide.activeInHierarchy && !fourthScreenMenuToHide.activeInHierarchy)
                {
                    fourthButtonToBePressed.onClick.Invoke();
                }
            }
            if (sixthButtonToBePressed != null && sixthButtonToBePressed.onClick != null)
            {
                if (!sixthARMenuToHide.activeInHierarchy && !sixthScreenMenuToHide.activeInHierarchy)
                {
                    sixthButtonToBePressed.onClick.Invoke();
                }
            }
            if (seventhButtonToBePressed != null && seventhButtonToBePressed.onClick != null)
            {
                if (!seventhARMenuToHide.activeInHierarchy && !seventhScreenMenuToHide.activeInHierarchy)
                {
                    seventhButtonToBePressed.onClick.Invoke();
                }
            }
            spheresToHide.SetActive(true);
            firstRunningModeButton.SetActive(false);
            secondRunningModeButton.SetActive(false);
        }

    }

    public void toggleColor()
    {
        if (imageForColour.color == Color.grey)
        {
            imageForColour.color = Color.white; //color if button is not active
        }
        else if (imageForColour.color == Color.white)
        {
            imageForColour.color = Color.grey; //color if button is active
        }
    }

}
