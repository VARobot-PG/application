using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DeleteCodeCmdNew : MonoBehaviour
{
    public CodetoCanvasGenNew ctcGenerator;

    public void deleteStatement()
    {
        Transform parentTransform = this.transform.parent;
        for (int i = 0; i < 4; i++)
        {
            if (parentTransform.name.StartsWith("Command"))
            {
                deleteStatement(parentTransform);
                i = 4;
            }
            else
            {
                parentTransform = parentTransform.parent;
            }
        }
    }
    public void deleteStatement(Transform commandNode)
    {
        if (commandNode.name.Contains("Command"))
        {
            string commandNumber = commandNode.name.Replace("Command", "");
            int currentStatement = 0;
            int.TryParse(commandNumber, out currentStatement);
            deleteStatement(currentStatement);
        }
    }
    public void deleteStatement(int currentStatement)
    {
        ctcGenerator.currentProgram = ctcGenerator.generateProgramFromTransform();
        Program currentProgram = ctcGenerator.currentProgram;
        List<Statement> statements = currentProgram.statements;        
        if (statements.Count > currentStatement)
            statements.RemoveAt(currentStatement);
        currentProgram.statements = statements;
        ctcGenerator.currentProgram = currentProgram;
        ctcGenerator.redraw();

    }
}
