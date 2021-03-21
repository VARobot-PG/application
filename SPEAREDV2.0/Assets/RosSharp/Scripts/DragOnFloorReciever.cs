using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;

public class DragOnFloorReciever : MonoBehaviour
{
    public GameObject targetSelectorPrefab;
    public InputField textField; //the text field where the coordinates should be
    public TextMeshPro text;

    private GameObject selector;
    bool currentlySelecting = false;
    public void onClickUseTargetSelector()
    {
        //if (!currentlySelecting) {
            currentlySelecting = true;

            /*Vector3 textVector = new Vector3(0, 0, 0);

            if(textField != null)
            {
                //Text t = FindText();
                string t = textField.text;
                Debug.Log(t.ToString());
                textVector = StringToVector3(t);
            }
            else
            {
                //Text t = FindText();
                string t = text.text;
                Debug.Log(t.ToString());
                textVector = StringToVector3(t); ;
            }*/

            float height = GameObject.Find("Blue").transform.position.y;
        //GameObject gmo = Instantiate(targetSelectorPrefab, new Vector3(textVector.x, height, textVector.z), Quaternion.identity);
        //float height = 0.05f;
        if (selector == null)
        {
            selector = Instantiate(targetSelectorPrefab, Camera.main.transform.position + Camera.main.transform.forward, Quaternion.identity);
            selector.GetComponent<DragOnFloor>().floorHeight = height;
            //selector = gmo;
            Debug.Log(selector.name);
        }
        else
        {
            selector.SetActive(true);
        }
        //}
        /*else
        {
            currentlySelecting = false;
            GameObject targetSelector = GameObject.Find("TargetSelector(Clone)");
            if (textField != null)
            {
                textField.text = targetSelector.transform.position.ToString("F1");

            }
            else
            {
                text.text = targetSelector.transform.position.ToString("F1");
            }
            Destroy(targetSelector);
        }*/
    }

    private void Update()
    {
        if (textField != null)
        {
            if (this.selector != null)
            {
                textField.text = this.selector.transform.position.ToString("F1");
            }
        }
        else
        {
            if (text != null && this.selector != null)
            {
                text.text = this.selector.transform.position.ToString("F1");
            }
        }
    }

    public static Vector3 StringToVector3(string sVector)
    {
        // Remove the parentheses
        if (sVector.StartsWith("(") && sVector.EndsWith(")"))
        {
            sVector = sVector.Substring(1, sVector.Length - 2);
        }

        // split the items
        string[] sArray = sVector.Split(',');

        // store as a Vector3
        Vector3 result = new Vector3(
            float.Parse(sArray[0]),
            float.Parse(sArray[1]),
            float.Parse(sArray[2]));

        return result;
    }

    private Text FindText()
    {
        Debug.Log("FindText, this.name= " + this.name);
        GameObject go = null;
        if (this.name == "CodeLineButton(Clone)")
        {
            go = this.transform.Find("TargetValue").gameObject;
        }
        else
        {
            if (this.name == "BruteCodeLineButton(Clone)")
            {
                go = this.transform.Find("TargetValue").gameObject;
            }
            else
            {
                if (this.name == "NewCodeEditorLine(Clone)")
                {
                    go = this.transform.Find("Touch_SuctionON").Find("Text").gameObject;
                }
            }
        }
        Text t = go.GetComponent<Text>();
        Debug.Log("FindText, text is:" + t.name);
        return t;
    }
}
