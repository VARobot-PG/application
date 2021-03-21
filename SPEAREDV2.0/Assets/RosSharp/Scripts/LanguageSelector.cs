using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class LanguageSelector : MonoBehaviour
{
    public Dropdown languageDropdown;
    public Text languageText;

    void Start()
    {
        languageDropdown.onValueChanged.AddListener(delegate {
            languageDropdownValueChangedHandler(languageDropdown);
        });
    }
    void Destroy()
    {
        languageDropdown.onValueChanged.RemoveAllListeners();
    }

    private void languageDropdownValueChangedHandler(Dropdown target)
    {
        if (target.value == 0)
        {
            languageText.GetComponentInChildren<Text>().text = "Please choose a robot / scenario to continue";
        }
        else
        {
            languageText.GetComponentInChildren<Text>().text = "Bitte wählen Sie einen Roboter / ein Szenario aus";
        }
    }

    public void SetDropdownIndex(int index)
    {
        languageDropdown.value = index;
    }

}