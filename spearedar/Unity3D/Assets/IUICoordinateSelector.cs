using UnityEngine;

public abstract  class IUICoordinateSelector : MonoBehaviour
{

    public abstract GameObject createCoordinatesSelector(MoveStatement moveStatement);
    public abstract Vector4 getCoordinateValues(GameObject coordinateSelector);
    public abstract string nameIdentifier();
}