using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PositionFreezer : MonoBehaviour
{
    public GameObject[] objectsToFreeze;
    public float freezeTime = 1;

    // Start is called before the first frame update
    void Start()
    {
        StartCoroutine(Freeze(freezeTime));
    }

    private IEnumerator Freeze(float time)
    {
        foreach(GameObject g in objectsToFreeze)
        {
            g.GetComponent<Rigidbody>().freezeRotation = true;
        }
        Debug.Log("Froze object rotations");
        yield return new WaitForSeconds(time);
        foreach (GameObject g in objectsToFreeze)
        {
            g.GetComponent<Rigidbody>().freezeRotation = false;
        }
        Debug.Log("Unfroze object rotations");

    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
