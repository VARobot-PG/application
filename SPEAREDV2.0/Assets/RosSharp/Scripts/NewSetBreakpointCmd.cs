using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class NewSetBreakpointCmd : MonoBehaviour
{
    public ExecuteNewGenProg executeGeneratedProgram;
    public void toggleBreakpointAtStatement()
    {
        Transform parentTransform = this.transform.parent;
        for (int i = 0; i < 4; i++)
        {
            if (parentTransform.name.StartsWith("Command"))
            {
                toggleBreakpointAtStatement(parentTransform);
                i = 4;
            }
            else
            {
                parentTransform = parentTransform.parent;
            }
        }
    }
    public void toggleBreakpointAtStatement(Transform commandNode)
    {
        if (commandNode.name.Contains("Command"))
        {
            string commandNumber = commandNode.name.Replace("Command", "");
            int currentStatement = 0;
            int.TryParse(commandNumber, out currentStatement);
            toggleBreakpointAtStatement(currentStatement);
        }
    }
    public void toggleBreakpointAtStatement(int currentStatement)
    {
        List<int> breakpoints = executeGeneratedProgram.breakpoints;
        if (breakpoints.Contains(currentStatement))
        {
            breakpoints.Remove(currentStatement);
        }
        else
        {
         breakpoints.Add(currentStatement);
        }
        executeGeneratedProgram.breakpoints = breakpoints;

    }
    // Start is called before the first frame update
    void Start()
    {
        executeGeneratedProgram = GameObject.FindObjectOfType<ExecuteNewGenProg>();
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
