using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class IdleTextWriter : MonoBehaviour
{
    public IdleSubscriber idleSubscriber;
    public TextMeshProUGUI textField;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        textField.text = ""+idleSubscriber.idle;
    }
}
