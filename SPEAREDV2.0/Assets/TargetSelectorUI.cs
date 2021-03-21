using Microsoft.MixedReality.Toolkit.UI;
using System;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class TargetSelectorUI : IUICoordinateSelector
{
    public GameObject targetSelectorPrefab;
    public CoordinateTargetSelector coordinateTargetSelector;
    public bool useROSCoordinates = true;
    public override GameObject createCoordinatesSelector(MoveStatement moveStatement)
    {
        GameObject coordinateDisplay;
        List<TextMeshPro> textFields;
        Button button;
        Interactable interactable;
        textFields = new List<TextMeshPro>();
        coordinateDisplay = Instantiate(targetSelectorPrefab);
        coordinateDisplay.GetComponentsInChildren<TextMeshPro>(textFields);
        Vector4 targetInV4 = new Vector4(moveStatement.target.x, moveStatement.target.y, moveStatement.target.z, 0f);
        button = coordinateDisplay.GetComponentInChildren<Button>();
        interactable = coordinateDisplay.GetComponentInChildren<Interactable>();
        foreach (TextMeshPro textField in textFields)
        {
            if (textField.name.Contains("TargetValue"))
            {

                textField.text = moveStatement.target.ToString();
                interactable.OnClick.AddListener(() =>
                {
                    if (useROSCoordinates)
                    {
                        if (coordinateTargetSelector.selectedRosTransform != null)
                        {
                            Transform rosCoords = coordinateTargetSelector.selectedRosTransform;
                            textField.text = rosCoords.localPosition.ToString();
                        }
                    }
                    else
                    {
                        Transform ownCoords = GameObject.Find("RealTargetSphere").transform;//   coordinateTargetSelector.selectedOwnTransform;
                        textField.text = ownCoords.localPosition.ToString();
                    }
                });
                /*
                button.onClick.AddListener(() =>
                {
                    if(coordinateTargetSelector.selectedRosTransform != null)
                    {
                        Transform rosCoords = coordinateTargetSelector.selectedRosTransform;
                        textField.text = rosCoords.localPosition.ToString();
                    }
                });
                */
            }
        }
        

        return coordinateDisplay;
    }

    public override Vector4 getCoordinateValues(GameObject coordinateSelector)
    {
     
        List<TextMeshPro> textFields;
        textFields = new List<TextMeshPro>();
        coordinateSelector.GetComponentsInChildren<TextMeshPro>(textFields);
        Vector4 resultTarget = Vector4.zero;
        foreach (TextMeshPro textField in textFields)
        {
            if (textField.name.Contains("TargetValue"))
            {
                resultTarget = parseText(textField.text);
            }
        }
        return resultTarget;
    }
    private Vector4 parseText(string text)
    {
        System.Globalization.CultureInfo culture;
        System.Globalization.NumberStyles style;
        
        Vector4 targetVector = Vector4.zero;
        string[] splitText;
        float parsedString = 0f;
        char[] trimChars = { '(', ')' };
        string adaptedText;
        culture = System.Globalization.CultureInfo.CreateSpecificCulture("en-GB");
        style = System.Globalization.NumberStyles.Number;
        adaptedText = text.Trim(trimChars);
        splitText = adaptedText.Split(',');
        for (int i = 0; i < splitText.Length; i++)
        {
            Single.TryParse(splitText[i], style, culture, out parsedString);
            switch (i)
            {
                case 0:
                    targetVector.x = parsedString;
                    break;
                case 1:
                    targetVector.y = parsedString;
                    break;
                case 2:
                    targetVector.z = parsedString;
                    break;
                case 3:
                    targetVector.w = parsedString;
                    break;
                default:
                    break;
            }
        }
        return targetVector;
    }
    public override string nameIdentifier()
    {
        return targetSelectorPrefab.name;
    }
}
