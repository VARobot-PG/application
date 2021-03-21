using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AddSuctionToggleCommand : MonoBehaviour
{
    public CodeToCanvasGenerator ctcGenerator;

    public void addSuctionToggleStatement()
    {
        Transform parentTransform = this.transform.parent;
        for (int i = 0; i < 4; i++)
        {
            if (parentTransform.name.StartsWith("Command"))
            {
                addSuctionToggleStatement(parentTransform);
                i = 4;
            }
            else
            {
                parentTransform = parentTransform.parent;
            }
        }
    }
    public void addSuctionToggleStatement(Transform commandNode)
    {
        if (commandNode.name.Contains("Command"))
        {
            string commandNumber = commandNode.name.Replace("Command", "");
            int currentStatement = 0;
            int.TryParse(commandNumber, out currentStatement);
            addSuctionToggleStatement(currentStatement);
        }
    }
    public void addSuctionToggleStatement(int currentStatement)
    {
        ctcGenerator.currentProgram = ctcGenerator.generateProgramFromTransform();
        Program currentProgram = ctcGenerator.currentProgram;
        List<Statement> statements = currentProgram.statements;
        ToggleSuction toggleSuctionStatement = new ToggleSuction();
        statements.Insert(currentStatement, toggleSuctionStatement);
        currentProgram.statements = statements;
        ctcGenerator.currentProgram = currentProgram;
        ctcGenerator.redraw();

    }
}
