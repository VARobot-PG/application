using UnityEngine;

public abstract class NewUIMovementTypeSelector : MonoBehaviour
{
    public abstract string nameIdentifier();
    public abstract MoveStatement1 getMoveStatementFromUI(GameObject movementTypeUI);
    //Brute Move Command
    public abstract BruteMove getBruteMoveFromUI(GameObject movementTypeUI);
    public abstract GameObject generateMovmentTypeSelectorFromStatement(MoveStatement1 moveStatement);
    public abstract GameObject generateBruteMovmentTypeSelectorFromStatement(BruteMove bruteStatement);

}