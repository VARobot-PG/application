using UnityEngine;
using System.Collections;

using UnityEngine.UI;

public class PollNameUpdate : MonoBehaviour
{
    public InputField InputField;

    public void Start()
    {
        InputField.text = NXTManager.Instance.NXTName;
    }

    public void UpdateRobotName()
    {
        NXTManager.Instance.NXTName = InputField.text;
    }
}
