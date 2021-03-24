using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoveCodeCommand : MonoBehaviour
{
    public bool isDirectionUpwards = false;
    public CodeToCanvasGenerator ctcGenerator;

    public void moveStatement()
    {
        Transform parentTransform = this.transform.parent;
        for (int i = 0; i < 4; i++)
        {
            if (parentTransform.name.StartsWith("Command"))
            {
                moveStatement(parentTransform);
                i = 4;
            }
            else
            {
                parentTransform = parentTransform.parent;
            }
        }
    }
    public void moveStatement(Transform commandNode)
    {
        if (commandNode.name.Contains("Command"))
        {
            string commandNumber = commandNode.name.Replace("Command", "");
            int currentStatement = 0;
            int.TryParse(commandNumber, out currentStatement);
            moveStatement(currentStatement);
        }
    }
    public void moveStatement(int currentStatement)
    {
        ctcGenerator.currentProgram = ctcGenerator.generateProgramFromTransform();
        Program currentProgram = ctcGenerator.currentProgram;
        List<Statement> statements = currentProgram.statements;
        int directionFactor = 0;
        if (isDirectionUpwards)
        {
            directionFactor = 1;
        }
        if(statements.Count > (currentStatement - directionFactor + 1) && currentStatement - directionFactor >= 0)
        statements.Reverse(currentStatement - directionFactor, 2); // swap currentStatement - directionFactor with (currentStatement - directionFactor) +1
        currentProgram.statements = statements;
        ctcGenerator.currentProgram = currentProgram;
        ctcGenerator.redraw();
        
    }
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
