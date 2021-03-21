using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.Events;

public class VoiceCommandBinding : MonoBehaviour
{
    public UnityEvent onVoiceCommand;
    public string command;
    private SpeechCommandController controller;

    public void Start()
    {
        Debug.Log("Start add cmd");
        controller = SpeechCommandController.Instance;
        VoiceCMD cmd = ScriptableObject.CreateInstance<VoiceCMD>();
        cmd.command = command;
        cmd.onVoiceCommand = onVoiceCommand;
        controller.AddVoiceCommand(cmd);
    }
}
