using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class GlobalCoordinateSetter : MonoBehaviour
{ public TextMesh textField;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        textField.text = String.Format("({0}, {1}, {2})", Math.Round(transform.position.x, 1), Math.Round(transform.position.y, 1), Math.Round(transform.position.z, 1));
    }
}
