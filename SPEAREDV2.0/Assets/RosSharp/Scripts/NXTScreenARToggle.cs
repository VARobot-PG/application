using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class NXTScreenARToggle : MonoBehaviour
{
    public Button firstARButtonToBePressed;
    public Button secondARButtonToBePressed;
    public Button thirdARButtonToBePressed;
    public Button fourthARButtonToBePressed;


    public GameObject firstARMenu;
    public GameObject firstScreenMenu;
    public GameObject secondARMenu;
    public GameObject secondScreenMenu;
    public GameObject thirdARMenu;
    public GameObject thirdScreenMenu;
    public GameObject fourthARMenu;
    public GameObject fourthScreenMenu;
    

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
    }
}
