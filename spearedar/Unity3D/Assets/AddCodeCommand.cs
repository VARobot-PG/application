using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AddCodeCommand : MonoBehaviour
{
    public CodeToCanvasGenerator ctcGenerator;

    public void addStatement()
    {
        Transform parentTransform = this.transform.parent;
        for (int i = 0; i < 4; i++)
        {
            if (parentTransform.name.StartsWith("Command"))
            {
                addStatement(parentTransform);
                i = 4;
            }
            else
            {
                parentTransform = parentTransform.parent;
            }
        }
    }
    public void addStatement(Transform commandNode)
    {
        if (commandNode.name.Contains("Command"))
        {
            string commandNumber = commandNode.name.Replace("Command", "");
            int currentStatement = 0;
            int.TryParse(commandNumber, out currentStatement);
            addStatement(currentStatement);
        }
    }
    public void addStatement(int currentStatement)
    {
        ctcGenerator.currentProgram = ctcGenerator.generateProgramFromTransform();
        Program currentProgram = ctcGenerator.currentProgram;
        List<Statement> statements = currentProgram.statements;
        MoveToL moveToL = new MoveToL();
        moveToL.target = new Vector3(215f, 0f, 145f);
        statements.Insert(currentStatement, moveToL);
        currentProgram.statements = statements;
        ctcGenerator.currentProgram = currentProgram;
        ctcGenerator.redraw();

    }
}
