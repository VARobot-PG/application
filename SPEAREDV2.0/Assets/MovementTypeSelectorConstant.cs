using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MovementTypeSelectorConstant : IUIMovementTypeSelector
{
    public override GameObject generateMovmentTypeSelectorFromStatement(MoveStatement moveStatement)
    {
        GameObject gameObject = new GameObject();
        gameObject.name = "MovementTypeConstant_" + moveStatement.movementValueForCanvas;
        return gameObject;
    }

    public override MoveStatement getMoveStatementFromUI(GameObject movementTypeUI)
    {

        string[] splittedString = movementTypeUI.name.Split('_');
        
        MoveStatement moveStatement = new MoveToJ();
        if (splittedString.Length > 1)
        {
            int movementValue;
            int.TryParse(splittedString[1], out movementValue);
            switch (movementValue)
            {
                case 0:
                    moveStatement = new MoveToL();
                    break;
                case 1:
                    moveStatement = new MoveToJ();
                    break;
                case 2:
                    moveStatement = new JumpTo();
                    break;
                case 3:
                    moveStatement = new MoveArc();
                    break;
                default:
                    break;
            }
        }
        return moveStatement;
    }

    public override string nameIdentifier()
    {
        return "MovementTypeConstant_";
    }

    
}
