using System.Collections.Generic;
using TMPro;
using UnityEngine;
public class SingleCoordinateDropdown : IUICoordinateSelector
{
    public GameObject singleCoordinateDropdownPrefab;
    public  override GameObject createCoordinatesSelector(MoveStatement moveStatement)
    {
        GameObject coordinateDisplay;
        List<TMP_Dropdown> dropdownFields;
        List<TMP_Dropdown.OptionData> options;
        coordinateDisplay = Instantiate(singleCoordinateDropdownPrefab);
        dropdownFields = new List<TMP_Dropdown>();
        coordinateDisplay.GetComponentsInChildren<TMP_Dropdown>(dropdownFields);
        int currentObject;
        Vector4 parsedDropdown = Vector4.zero;

        Vector4 targetInV4 = new Vector4(moveStatement.target.x, moveStatement.target.y, moveStatement.target.z, 0f);
        foreach (TMP_Dropdown dropdownCoordinate in dropdownFields)
        {
            options = dropdownCoordinate.options;
            currentObject = 0;
            foreach (TMP_Dropdown.OptionData optionData in options)
            {
                parsedDropdown = parseDropdownText(optionData.text);
                
                
                if (parsedDropdown.Equals(targetInV4))
                {
                    dropdownCoordinate.value = currentObject;
                }
                currentObject += 1;
            }
        }
        return coordinateDisplay;
    }

    private Vector4 parseDropdownText(string optionText)
    {
        Vector4 parsedDropdown = Vector4.zero;
        string[] splitDropdown;
        float parsedString = 0f;
        char[] trimChars = { '(', ')' };
        optionText = optionText.Trim(trimChars);
        splitDropdown = optionText.Split(',');
        for (int i = 0; i < splitDropdown.Length; i++)
        {
            parsedString = float.Parse(splitDropdown[i]);
            switch (i)
            {
                case 0:
                    parsedDropdown.x = parsedString;
                    break;
                case 1:
                    parsedDropdown.y = parsedString;
                    break;
                case 2:
                    parsedDropdown.z = parsedString;
                    break;
                case 3:
                    parsedDropdown.w = parsedString;
                    break;
                default:
                    break;
            }
        }
        return parsedDropdown;
    }
    public override Vector4 getCoordinateValues(GameObject coordinateSelector)
    {
        List<TMP_Dropdown> dropdowns = new List<TMP_Dropdown>();
        coordinateSelector.GetComponentsInChildren<TMP_Dropdown>(dropdowns);
        Vector4 target = Vector4.zero;
        foreach (TMP_Dropdown dropdown in dropdowns)
        {
            target = parseDropdownText(dropdown.options[dropdown.value].text);
        }
        return target;
    }

    public override string nameIdentifier()
    {
        return singleCoordinateDropdownPrefab.name;
    }
}