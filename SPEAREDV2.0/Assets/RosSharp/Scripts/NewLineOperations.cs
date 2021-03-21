using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class NewLineOperations : MonoBehaviour
{
    public Button RemoveButtonGO;
    public InputField CommandArgs;
    public TMP_Dropdown Command;
    public Button SelectButton;
    public OnClickTargetSelection targetSelection;
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
            CommandArgs.text = "(-0.7, -0.5, 1.5)";
            var dragOnFloor = this.GetComponentInParent<DragOnFloorReciever>();
            SelectButton.onClick.AddListener(targetSelection.onClickSelectTargetNonROS);
            SelectButton.onClick.AddListener(dragOnFloor.onClickUseTargetSelector);
        }
        else if (option == 2)
        {
            CommandArgs.text = "up";

            SelectButton.GetComponentInChildren<Text>().text = "switch";
            SelectButton.onClick.AddListener(() =>
            {
                switch (CommandArgs.text)
                {
                    case "up":
                        CommandArgs.text = "down";
                        break;
                    case "down":
                        CommandArgs.text = "up";
                        break;
                }
            });

        }
        else
        {
            var dragOnFloor = this.GetComponentInParent<DragOnFloorReciever>();
            SelectButton.onClick.AddListener(targetSelection.onClickSelectTargetNonROS);
            SelectButton.onClick.AddListener(dragOnFloor.onClickUseTargetSelector);
            CommandArgs.text = "(78, 0, 200)";
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
            statement = new MoveToJ1();
            var commaseparatedVector = CommandArgs.text.Substring(1).Substring(0, CommandArgs.text.Length - 2);
            var vectorValues = commaseparatedVector.Split(',');

            (statement as MoveToJ1).target = new Vector3(Single.Parse(vectorValues[0], new CultureInfo("en-US").NumberFormat), Single.Parse(vectorValues[1], new CultureInfo("en-US").NumberFormat), Single.Parse(vectorValues[2], new CultureInfo("en-US").NumberFormat));
        }
        else if (Command.value == 2)
        {
            statement = new ClawUp();
            (statement as ClawUp).isClawUp = CommandArgs.text.Trim().ToLower().Equals("up");
        }
        else
        {
            statement = new BruteMoveToJ1();
            var commaseparatedVector = CommandArgs.text.Substring(1).Substring(0, CommandArgs.text.Length - 2);
            var vectorValues = commaseparatedVector.Split(',');

            (statement as BruteMoveToJ1).target = new Vector3(Single.Parse(vectorValues[0], new CultureInfo("en-US").NumberFormat), Single.Parse(vectorValues[1], new CultureInfo("en-US").NumberFormat), Single.Parse(vectorValues[2], new CultureInfo("en-US").NumberFormat));
        }
        return statement;

    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
