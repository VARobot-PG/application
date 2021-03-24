using UnityEngine;

public abstract  class ICommandUIRepresentation : MonoBehaviour
{
    public abstract GameObject constructCommandUIComponent(string commandUIName, Statement statement);
    public abstract Statement generateStatementFromUI(GameObject commandContainer);

    public abstract Rect getCommandObjectBoundaries();
    public abstract void changeCommandTypeComponent(IUIMovementTypeSelector uImovementTypeSelector);
    public abstract void changeParameterComponent(IUICoordinateSelector uICoordinateSelector);

}
