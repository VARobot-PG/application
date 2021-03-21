using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AddCodeCmdNew : MonoBehaviour
{
    public CodetoCanvasGenNew ctcGenerator;

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
        MoveToL1 moveToL = new MoveToL1();
        moveToL.target = new Vector3(215f, 0f, 145f);
        statements.Insert(currentStatement, moveToL);
        currentProgram.statements = statements;
        ctcGenerator.currentProgram = currentProgram;
        ctcGenerator.redraw();

    }

    //BruteMove
    public void addBruteStatement()
    {
        Transform parentTransform = this.transform.parent;
        for (int i = 0; i < 4; i++)
        {
            if (parentTransform.name.StartsWith("Command"))
            {
                addBruteStatement(parentTransform);
                i = 4;
            }
            else
            {
                parentTransform = parentTransform.parent;
            }
        }
    }
    public void addBruteStatement(Transform commandNode)
    {
        if (commandNode.name.Contains("Command"))
        {
            string commandNumber = commandNode.name.Replace("Command", "");
            int currentStatement = 0;
            int.TryParse(commandNumber, out currentStatement);
            addBruteStatement(currentStatement);
        }
    }
    public void addBruteStatement(int currentStatement)
    {
        ctcGenerator.currentProgram = ctcGenerator.generateProgramFromTransform();
        Program currentProgram = ctcGenerator.currentProgram;
        List<Statement> statements = currentProgram.statements;
        BruteMoveToJ1 moveToL = new BruteMoveToJ1();
        moveToL.target = new Vector3(215f, 0f, 145f);
        statements.Insert(currentStatement, moveToL);
        currentProgram.statements = statements;
        //debug 
        Debug.Log("Program by adding Brute Move :"+statements);
        ctcGenerator.currentProgram = currentProgram;
        ctcGenerator.redraw();

    }
}
