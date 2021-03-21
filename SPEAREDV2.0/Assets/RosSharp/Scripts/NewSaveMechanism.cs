using System.Collections;
using System.Collections.Generic;
using System.IO;
using System.Runtime.Serialization.Formatters.Binary;
using UnityEngine;
using UnityEngine.UI;

public class NewSaveMechanism : MonoBehaviour
{

    public Transform Cmds;
    private List<Transform> codeEditorLines;

    public Button addMovementButton;
    public Button addBruteMovementButton;

    public List<Transform> getCodeEditorLines()
    {
        codeEditorLines = new List<Transform>();
        foreach (Transform child in Cmds)
        {
            codeEditorLines.Add(child);
        }
        return codeEditorLines;
    }

    public void ClearRobotCmds()
    {
        foreach (Transform child in Cmds)
        {
            GameObject.Destroy(child.gameObject);
        }
    }

    private Save CreateSaveGameObject()
    {
        Save save = new Save();
        codeEditorLines = getCodeEditorLines();
        foreach (Transform CodeEditorLine in codeEditorLines)
        {
            Text[] tmp = CodeEditorLine.GetComponentsInChildren<Text>();
            save.RobotCmds.Add(tmp[1].text);
            // save.RobotCmds.Add(CodeEditorLine.GetComponentInChildren<Text>().text); // Returns 3*ON
        }
        return save;
    }

    public void SaveGame()
    {
        // 1
        Save save = CreateSaveGameObject();

        // 2
        BinaryFormatter bf = new BinaryFormatter();
        FileStream file = File.Create(Application.persistentDataPath + "/gamesave.save");
        bf.Serialize(file, save);
        file.Close();

        // 3
        Debug.Log("Game Saved");
    }

    public void LoadGame()
    {
        // 1
        if (File.Exists(Application.persistentDataPath + "/gamesave.save"))
        {
            ClearRobotCmds();

            // 2
            BinaryFormatter bf = new BinaryFormatter();
            FileStream file = File.Open(Application.persistentDataPath + "/gamesave.save", FileMode.Open);
            Save save = (Save)bf.Deserialize(file);
            file.Close();

            // 3
            for (int i = 0; i < save.RobotCmds.Count; i++)
            {
                string[] tmp = save.RobotCmds[i].Split(',');
                if (tmp[1].Replace(" ","") == "0.0" || tmp[1].Replace(" ", "") == "0" || tmp[1].Replace(" ", "") == "0,0")
                {
                    addBruteMovementButton.onClick.Invoke();
                    List<Transform> codeLines = getCodeEditorLines();
                    /*
                    Text[] tmp = codeLines[codeLines.Count - 1].GetComponentsInChildren<Text>();
                    tmp[1].text = save.RobotCmds[i];
                    */
                    // codeLines[codeLines.Count - 1].GetComponentsInChildren<Text>()[1].text = save.RobotCmds[i];
                    // Debug.Log(save.RobotCmds[i]);
                    // Debug.Log(codeLines[codeLines.Count - 1].GetComponentsInChildren<Text>()[1].text);
                    var lo = codeLines[codeLines.Count - 1].GetComponent<NewLineOperations>();
                    lo.SetArg(save.RobotCmds[i]);
                }
                else
                {
                    addMovementButton.onClick.Invoke();
                    List<Transform> codeLines = getCodeEditorLines();
                    /*
                    Text[] tmp = codeLines[codeLines.Count - 1].GetComponentsInChildren<Text>();
                    tmp[1].text = save.RobotCmds[i];
                    */
                    // codeLines[codeLines.Count - 1].GetComponentsInChildren<Text>()[1].text = save.RobotCmds[i];
                    var lo = codeLines[codeLines.Count - 1].GetComponent<NewLineOperations>();
                    lo.SetArg(save.RobotCmds[i]);
                }
            }

            // 4
            Debug.Log("Game Loaded");
        }
        else
        {
            Debug.Log("No game saved!");
        }
    }

}
