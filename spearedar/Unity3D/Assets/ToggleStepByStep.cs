using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ToggleStepByStep : MonoBehaviour
{
    public ExecuteGeneratedProgram executeGeneratedProgram;
    public bool defaultToggleOn = false;
    public void toggleStepByStep()
    {
        executeGeneratedProgram.ToggleExecutionLoop();
    }
    // Start is called before the first frame update
    void Start()
    {
        if (defaultToggleOn)
        {
            executeGeneratedProgram.EnableStepByStep();
        }
        else
        {
            executeGeneratedProgram.DisableStepByStep();
        }
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
