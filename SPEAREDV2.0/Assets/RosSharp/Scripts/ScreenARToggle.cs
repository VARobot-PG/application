using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ScreenARToggle : MonoBehaviour
{

    public Button firstARButtonToBePressed;
    public Button secondARButtonToBePressed;
    public Button thirdARButtonToBePressed;
    public Button fourthARButtonToBePressed;
    public Button fifthARButtonToBePressed;
    public Button sixthARButtonToBePressed;
    public Button seventhARButtonToBePressed;


    public GameObject firstARMenu;
    public GameObject firstScreenMenu;
    public GameObject secondARMenu;
    public GameObject secondScreenMenu;
    public GameObject thirdARMenu;
    public GameObject thirdScreenMenu;
    public GameObject fourthARMenu;
    public GameObject fourthScreenMenu;
    public GameObject fifthARMenu;
    public GameObject fifthScreenMenu;
    public GameObject sixthARMenu;
    public GameObject sixthScreenMenu;
    public GameObject seventhARMenu;
    public GameObject seventhScreenMenu;

    public void sendToARSpace()
    {
        if (!firstARMenu.activeInHierarchy && firstScreenMenu.activeInHierarchy)
        {
            firstARButtonToBePressed.onClick.Invoke();
        }
        if (!secondARMenu.activeInHierarchy && secondScreenMenu.activeInHierarchy)
        {
            secondARButtonToBePressed.onClick.Invoke();
        }
        if (!thirdARMenu.activeInHierarchy && thirdScreenMenu.activeInHierarchy)
        {
            thirdARButtonToBePressed.onClick.Invoke();
        }
        if (!fourthARMenu.activeInHierarchy && fourthScreenMenu.activeInHierarchy)
        {
            fourthARButtonToBePressed.onClick.Invoke();
        }
        if (!fifthARMenu.activeInHierarchy && fifthScreenMenu.activeInHierarchy)
        {
            fifthARButtonToBePressed.onClick.Invoke();
        }
        if (!sixthARMenu.activeInHierarchy && sixthScreenMenu.activeInHierarchy)
        {
            sixthARButtonToBePressed.onClick.Invoke();
        }
        if (!seventhARMenu.activeInHierarchy && seventhScreenMenu.activeInHierarchy)
        {
            seventhARButtonToBePressed.onClick.Invoke();
        }
    }
    public void sendToScreenSpace()
    {
        if (!firstScreenMenu.activeInHierarchy && firstARMenu.activeInHierarchy)
        {
            firstARButtonToBePressed.onClick.Invoke();
        }
        if (!secondScreenMenu.activeInHierarchy && secondARMenu.activeInHierarchy)
        {
            secondARButtonToBePressed.onClick.Invoke();
        }
        if (!thirdScreenMenu.activeInHierarchy && thirdARMenu.activeInHierarchy)
        {
            thirdARButtonToBePressed.onClick.Invoke();
        }
        if (!fourthScreenMenu.activeInHierarchy && fourthARMenu.activeInHierarchy)
        {
            fourthARButtonToBePressed.onClick.Invoke();
        }
        if (!fifthScreenMenu.activeInHierarchy && fifthARMenu.activeInHierarchy)
        {
            fifthARButtonToBePressed.onClick.Invoke();
        }
        if (!sixthScreenMenu.activeInHierarchy && sixthARMenu.activeInHierarchy)
        {
            sixthARButtonToBePressed.onClick.Invoke();
        }
        if (!seventhScreenMenu.activeInHierarchy && seventhARMenu.activeInHierarchy)
        {
            seventhARButtonToBePressed.onClick.Invoke();
        }
    }

}
