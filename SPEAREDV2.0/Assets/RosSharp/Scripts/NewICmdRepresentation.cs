using UnityEngine;

public abstract  class NewICmdRepresentation : MonoBehaviour
{
    public abstract GameObject constructCommandUIComponent(string commandUIName, Statement statement);
    public abstract Statement generateStatementFromUI(GameObject commandContainer);

    public abstract Rect getCommandObjectBoundaries();
    public abstract void changeCommandTypeComponent(NewUIMovementTypeSelector uImovementTypeSelector);
    public abstract void changeParameterComponent(NewUICoordinateSelector uICoordinateSelector);

}
