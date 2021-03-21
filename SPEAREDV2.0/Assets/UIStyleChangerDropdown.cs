using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class UIStyleChangerDropdown : MonoBehaviour
{
    public List<IUICoordinateSelector> uiCoordinateSelectors;
    public CodeToCanvasGenerator codeToCanvasGenerator;
    public TMP_Dropdown tmpDropdown;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    public void updateUIStyle()
    {
        updateUIStyle(tmpDropdown.value);
    }

    public void updateUIStyle(int value)
    {
        if(value < uiCoordinateSelectors.Count)
        codeToCanvasGenerator.commandUIRepresentation.changeParameterComponent(uiCoordinateSelectors[value]);
    }
}
