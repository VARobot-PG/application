using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AddClawToggleCommand : MonoBehaviour
{
    public CodetoCanvasGenNew ctcGenerator;

    public void addClawToggleStatement()
    {
        Transform parentTransform = this.transform.parent;
        for (int i = 0; i < 4; i++)
        {
            if (parentTransform.name.StartsWith("Command"))
            {
                addClawToggleStatement(parentTransform);
                i = 4;
            }
            else
            {
                parentTransform = parentTransform.parent;
            }
        }
    }
    public void addClawToggleStatement(Transform commandNode)
    {
        if (commandNode.name.Contains("Command"))
        {
            string commandNumber = commandNode.name.Replace("Command", "");
            int currentStatement = 0;
            int.TryParse(commandNumber, out currentStatement);
            addClawToggleStatement(currentStatement);
        }
    }
    public void addClawToggleStatement(int currentStatement)
    {
        ctcGenerator.currentProgram = ctcGenerator.generateProgramFromTransform();
        Program currentProgram = ctcGenerator.currentProgram;
        List<Statement> statements = currentProgram.statements;
        ClawUp toggleClawStatement = new ClawUp();
        statements.Insert(currentStatement, toggleClawStatement);
        currentProgram.statements = statements;
        ctcGenerator.currentProgram = currentProgram;
        ctcGenerator.redraw();

    }
}
