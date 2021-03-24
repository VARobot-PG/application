using UnityEngine;

public abstract class IUIMovementTypeSelector : MonoBehaviour
{
    public abstract string nameIdentifier();
    public abstract MoveStatement getMoveStatementFromUI(GameObject movementTypeUI);
    public abstract GameObject generateMovmentTypeSelectorFromStatement(MoveStatement moveStatement);
}