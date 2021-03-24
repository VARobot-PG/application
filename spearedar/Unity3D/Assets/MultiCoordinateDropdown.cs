using System;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class MultiCoordinateDropdown : IUICoordinateSelector
{
    public GameObject multiCoordinateDropdownPrefab;
    public override Vector4 getCoordinateValues(GameObject coordinateSelector)
    {
        float parsedValue;
        String coordinateValueToParse;
        Vector4 target;
        List<TMP_Dropdown> dropdowns;
        List<TMP_Dropdown.OptionData> coordinateDropdownOptions;
        dropdowns = new List<TMP_Dropdown>();
        target = Vector4.zero;
        coordinateValueToParse = "";
        coordinateSelector.GetComponentsInChildren<TMP_Dropdown>(dropdowns);
        coordinateDropdownOptions = new List<TMP_Dropdown.OptionData>();
        parsedValue = 0f;        
        foreach (TMP_Dropdown dropdown in dropdowns)
        {
            coordinateDropdownOptions = dropdown.options;
            if (coordinateDropdownOptions.Count >= dropdown.value + 1)
            {
                coordinateValueToParse = coordinateDropdownOptions[dropdown.value].text;
            }
            parsedValue = float.Parse(coordinateValueToParse);
            switch (dropdown.transform.parent.name)
            {
                case "X":
                    target.x = parsedValue;
                    break;
                case "Y":
                    target.y = parsedValue;
                    break;
                case "Z":
                    target.z = parsedValue;
                    break;
                case "R":
                    target.w = parsedValue;
                    break;
                default:
                    break;
            }
        }
        return target;
    }
    public override GameObject createCoordinatesSelector(MoveStatement moveStatement)
    {
        GameObject coordinateDisplay;
        List<TMP_Dropdown> dropdownFields;
        List<TMP_Dropdown.OptionData> options;
        coordinateDisplay = Instantiate(multiCoordinateDropdownPrefab);
        dropdownFields = new List<TMP_Dropdown>();
        coordinateDisplay.GetComponentsInChildren<TMP_Dropdown>(dropdownFields);
        int currentObject;
        foreach (TMP_Dropdown dropdownCoordinate in dropdownFields)
        {
            switch (dropdownCoordinate.transform.parent.name)
            {
                case "X":
                    options = dropdownCoordinate.options;
                    currentObject = 0;
                    foreach (TMP_Dropdown.OptionData optionData in options)
                    {
                        if ((float.Parse(optionData.text)).Equals(moveStatement.target.x))
                        {
                            dropdownCoordinate.value = currentObject;
                        }
                        currentObject += 1;
                    }
                    break;
                case "Y":
                    options = dropdownCoordinate.options;
                    currentObject = 0;
                    foreach (TMP_Dropdown.OptionData optionData in options)
                    {
                        if ((float.Parse(optionData.text)).Equals(moveStatement.target.y))
                        {
                            dropdownCoordinate.value = currentObject;
                        }
                        currentObject += 1;
                    }
                    break;
                case "Z":
                    options = dropdownCoordinate.options;
                    currentObject = 0;
                    foreach (TMP_Dropdown.OptionData optionData in options)
                    {
                        if ((float.Parse(optionData.text)).Equals(moveStatement.target.z))
                        {
                            dropdownCoordinate.value = currentObject;
                        }
                        currentObject += 1;
                    }
                    break;
                default:
                    break;
            }
        }
        return coordinateDisplay;
    }

    public override string nameIdentifier()
    {
        return multiCoordinateDropdownPrefab.name;
    }
}