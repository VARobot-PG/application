using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class SwitchingRobots : MonoBehaviour
{

	public GameObject RobotCmds;
	public GameObject NewRobotCmds;

	// reference to robot commands dropdown game object
	public GameObject dropdownRobotCommands;

	// referenses to controlled game objects
	public GameObject avatar1, avatar2;

	// variable contains which avatar is on and active
	int whichAvatarIsOn = 1;

	// Use this for initialization
	void Start()
	{

		// anable first avatar and disable another one
		avatar1.gameObject.SetActive(true);
		avatar2.gameObject.SetActive(false);

		// initialize touch input
		RobotCmds.SetActive(true);
		NewRobotCmds.SetActive(false);

		// initialize robot commands list
		bool boolAddNew = true;
		int countNew = dropdownRobotCommands.GetComponent<Dropdown>().options.Count - 1;
		// add new robot commands if not existing
		for (int i = countNew; i >= 0; i--)
		{
			if (dropdownRobotCommands.GetComponent<Dropdown>().options[i].text == "New Movement" || dropdownRobotCommands.GetComponent<Dropdown>().options[i].text == "New Suction Toggle")
			{
				boolAddNew = false;
				break;
			}
		}
		if (boolAddNew == true)
		{
			dropdownRobotCommands.GetComponent<Dropdown>().options.Add(new Dropdown.OptionData() { text = "New Movement" });
			dropdownRobotCommands.GetComponent<Dropdown>().options.Add(new Dropdown.OptionData() { text = "New Suction Toggle" });
		}
		// remove old robot commands
		countNew = dropdownRobotCommands.GetComponent<Dropdown>().options.Count - 1;
		for (int i = countNew; i >= 0; i--)
		{
			if (dropdownRobotCommands.GetComponent<Dropdown>().options[i].text == "NewRobotDraftCommand") dropdownRobotCommands.GetComponent<Dropdown>().options.RemoveAt(i);
		}


	}

	// public method to switch avatars by pressing UI button
	public void SwitchAvatar()
	{

		// processing whichAvatarIsOn variable
		switch (whichAvatarIsOn)
		{

			// if the first avatar is on
			case 1:

				// then the second avatar is on now
				whichAvatarIsOn = 2;

				// disable the first one and anable the second one
				avatar1.gameObject.SetActive(false);
				avatar2.gameObject.SetActive(true);
				break;

			// if the second avatar is on
			case 2:

				// then the first avatar is on now
				whichAvatarIsOn = 1;

				// disable the second one and anable the first one
				avatar1.gameObject.SetActive(true);
				avatar2.gameObject.SetActive(false);
				break;
		}

	}

	// public method to switch robot commands by pressing UI button
	public void SwitchCommands()
	{

		// processing whichAvatarIsOn variable
		switch (whichAvatarIsOn)
		{

			case 1:
				
				bool boolAddNew = true;
				int countNew = dropdownRobotCommands.GetComponent<Dropdown>().options.Count - 1;

				// add new robot commands if not existing
				for (int i = countNew; i >= 0; i--)
				{
					if (dropdownRobotCommands.GetComponent<Dropdown>().options[i].text == "New Movement" || dropdownRobotCommands.GetComponent<Dropdown>().options[i].text == "New Suction Toggle")
					{
						boolAddNew = false;
						break;
					}
				}
				if (boolAddNew == true)
				{
					dropdownRobotCommands.GetComponent<Dropdown>().options.Add(new Dropdown.OptionData() { text = "New Movement" });
					dropdownRobotCommands.GetComponent<Dropdown>().options.Add(new Dropdown.OptionData() { text = "New Suction Toggle" });
				}

				// remove old robot commands
				countNew = dropdownRobotCommands.GetComponent<Dropdown>().options.Count - 1;
				for (int i = countNew; i >= 0; i--)
				{
					if (dropdownRobotCommands.GetComponent<Dropdown>().options[i].text == "NewRobotDraftCommand") dropdownRobotCommands.GetComponent<Dropdown>().options.RemoveAt(i);
				}

				// END
				break;

			case 2:

				bool boolAdd = true;
				int count = dropdownRobotCommands.GetComponent<Dropdown>().options.Count - 1;

				// add new robot commands if not existing
				for (int i = count; i >= 0; i--)
				{
					if (dropdownRobotCommands.GetComponent<Dropdown>().options[i].text == "NewRobotDraftCommand")
					{
						boolAdd = false;
						break;
					}
				}
				if (boolAdd == true)
				{
					dropdownRobotCommands.GetComponent<Dropdown>().options.Add(new Dropdown.OptionData() { text = "NewRobotDraftCommand" });
				}

				// remove old robot commands
				count = dropdownRobotCommands.GetComponent<Dropdown>().options.Count - 1;
				for (int i = count; i >= 0; i--)
				{
					if (dropdownRobotCommands.GetComponent<Dropdown>().options[i].text == "New Movement") dropdownRobotCommands.GetComponent<Dropdown>().options.RemoveAt(i);
					if (dropdownRobotCommands.GetComponent<Dropdown>().options[i].text == "New Suction Toggle") dropdownRobotCommands.GetComponent<Dropdown>().options.RemoveAt(i);
				}

				// END
				break;

		}

	}

	// public method to switch touch input by pressing UI button
	public void SwitchTouchInput()
	{

		// processing whichAvatarIsOn variable
		switch (whichAvatarIsOn)
		{

			case 1:

				RobotCmds.SetActive(true);
				NewRobotCmds.SetActive(false);
				break;

			case 2:

				RobotCmds.SetActive(false);
				NewRobotCmds.SetActive(true);
				break;

		}

	}

	// TODO - SwitchCodePanel

}
