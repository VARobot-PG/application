using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.Events;

[RequireComponent(typeof(AndroidBridge))]
public class SpeechCommandController : MonoBehaviour
{
    private Dictionary<string,VoiceCMD> voicecommands;
    public static SpeechCommandController Instance;
    private AndroidBridge bridge;
    public void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            voicecommands = new Dictionary<string, VoiceCMD>();
        }
    }
    public void Start()
    {
        bridge = this.GetComponent<AndroidBridge>();
    }
    public void AddVoiceCommand(VoiceCMD voiceCommand)
    {
        //debugger.text += "Command added: " + voiceCommand.command;
        voicecommands.Add(voiceCommand.command,voiceCommand);
    }
    public void checkForKeyWord(string input)
    {
        input = input.ToLower();
        foreach (var cmd in voicecommands.Keys)
        {
            //debugger.text += "Command: " + cmd + "\n";
            //debugger.text += "input: " + input + "\n";
            if (input.Contains(cmd.ToLower()))
            {
                //debugger.text += "Text recognized\n";
                voicecommands[cmd].onVoiceCommand.Invoke();        
                break;
            }
        }
        //debugger.text += "Text not recognized\n";
    }
}
