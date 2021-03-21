using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Events;

public class VoiceCMD : ScriptableObject
{
    public UnityEvent onVoiceCommand;
    public string command;
}
