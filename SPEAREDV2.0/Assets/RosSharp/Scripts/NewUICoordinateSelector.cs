using UnityEngine;

public abstract  class NewUICoordinateSelector : MonoBehaviour
{

    public abstract GameObject createCoordinatesSelector(MoveStatement1 moveStatement);
    //for Brute Move command
    public abstract GameObject createBruteCoordinatesSelector(BruteMove bruteStatement);
    public abstract Vector4 getCoordinateValues(GameObject coordinateSelector);
    public abstract string nameIdentifier();
}