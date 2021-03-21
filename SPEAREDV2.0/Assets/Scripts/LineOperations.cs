using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class LineOperations : MonoBehaviour
{
    public Button RemoveButtonGO;
    public InputField CommandArgs;
    public TMP_Dropdown Command;
    // Start is called before the first frame update
    void Start()
    {
        var buttons = gameObject.GetComponentsInChildren<Button>();
        foreach (var btn in buttons)
        {
            if (btn.name.Equals("Touch_Delete1"))
            {
                btn.onClick.AddListener(() =>
                {
                    Destroy(gameObject);
                });
            }
        }
    }
    public void ChangeCommand(int option)
    {
        Command.value = option;
        if (option == 0)
        {
            CommandArgs.text = "(78, 200, 145)";
        }
        else
        {
            CommandArgs.text = "true";
        }
    }
    public void SetArg(string args)
    {
        CommandArgs.text = args;
    }
    public Statement serialize2ROSCode()
    {
        Statement statement;
        if (Command.value == 0)
        {
            statement = new MoveToJ();
            var commaseparatedVector = CommandArgs.text.Substring(1).Substring(0, CommandArgs.text.Length - 2);
            var vectorValues = commaseparatedVector.Split(',');

            (statement as MoveToJ).target = new Vector3(Single.Parse(vectorValues[0], new CultureInfo("en-US").NumberFormat), Single.Parse(vectorValues[1], new CultureInfo("en-US").NumberFormat), Single.Parse(vectorValues[2], new CultureInfo("en-US").NumberFormat));
        }
        else
        {
            statement = new ToggleSuction();
            (statement as ToggleSuction).isSuctionEnabled = CommandArgs.text.Trim().ToLower().Equals("true");
        }
        return statement;

    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
