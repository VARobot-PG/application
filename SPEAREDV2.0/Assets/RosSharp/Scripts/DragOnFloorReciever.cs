using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
using UnityEngine.SceneManagement;

public class DragOnFloorReciever : MonoBehaviour
{
    public GameObject targetSelectorPrefab;
    public InputField textField; //the text field where the coordinates should be
    public TextMeshPro text;
    float height;

    private GameObject selector;
    public void onClickUseTargetSelector()
    {
        //if (!currentlySelecting) {

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


        if (GameObject.Find("Plane_") != null)
        {
            height = GameObject.Find("Plane_").transform.position.y;
        }
        else
        {
            if (GameObject.Find("Plane") != null)
            {
                height = GameObject.Find("Plane").transform.position.y;
            }
            else
            {
                height = 0;
            }
        }


        //GameObject gmo = Instantiate(targetSelectorPrefab, new Vector3(textVector.x, height, textVector.z), Quaternion.identity);
        //float height = 0.05f;
        if (selector == null)
        {
            selector = Instantiate(targetSelectorPrefab, Camera.main.transform.position + Camera.main.transform.forward*0.5f, Quaternion.identity);
            selector.GetComponent<DragOnFloor>().floorHeight = height;
            selector.GetComponent<DragOnFloor>().selecting = true;
            //selector = gmo;
            Debug.Log(selector.name);
        }
        else
        {
            //selector.SetActive(true);
            selector.GetComponent<DragOnFloor>().SetMaterialPurple();
            selector.GetComponent<DragOnFloor>().selecting = true;
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

    /* On each Update, get the selection sphere's position, convert it to robot coordinates and write it into the given text field (as long as the selector is selecting) */
    private void Update()
    {
        Vector3 officialPosition = this.selector.transform.position;
        if (this.selector != null)
        {
            Vector3 unityPosition = this.selector.transform.position;
            GameObject zero = GameObject.Find("AbsoluteZero");
            if (zero != null)
            {
                GameObject go = new GameObject("GO DragOnFloorReciever"); 
                go.transform.position = unityPosition;
                go.transform.parent = zero.transform;
                Vector3 tmpPos = go.transform.localPosition;
                officialPosition = new Vector3(tmpPos.z, tmpPos.y, tmpPos.x);
                Destroy(go);
            }
            else
            {
                Debug.Log("AbsoluteZero not found");
            }
        }
        
        if (textField != null)
        {
            if (this.selector != null && selector.GetComponent<DragOnFloor>().selecting)
            {
                textField.text = officialPosition.ToString("F2");
            }
        }
        else
        {
            if (text != null && this.selector != null && selector.GetComponent<DragOnFloor>().selecting)
            {
                text.text = officialPosition.ToString("F2");
            }
        }
    }

    private void OnDestroy()
    {
        Destroy(selector);
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
