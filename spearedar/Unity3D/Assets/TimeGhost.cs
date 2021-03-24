using System.Collections;
using System.Collections.Generic;
using System;
using UnityEngine;


//implements TimeGhosts if the object passed to Dobots
//TimeGhosts are visualizations of the objects past state that are not interactive anymore
// meaning they don't collide, they don't react to inputs
// Ghosts as implemented here are only visualizations 
// they can be shown with showTimeGhosts function and hidden again with the
// hideAllGhosts function
// it is recommended to pause the time 
public class TimeGhost : MonoBehaviour
{
    public GameObject dobot;
    //this array holds the reference to the past states
    protected GameObject[] stateArray = new GameObject[25];
    //this list holds references to all ghosts that are currently shown
    // it is necessary as the ghosts in the stateArray will be overwritten
    // a maximum of stateArray.Lenght-1 updates
    protected System.Collections.ArrayList currentGhosts = null;
    //if enabled TimeGhosts are saved everyFrame
    private bool GeisterZeit = true;
    //if enabled TimeGhosts are shown every 1100 frames and hidden after 600 frames
    public bool debug = false;

    //sets how often a ghost is saved.
    public float save_frequency = 0.25f;

    protected int statePointer = -1;
    private int x = 00;
    //toggle indicating wether ghosts are active or not
    private bool spookyTime = false;
    private string GhostcreatorCoroutine = "GhostCreator";

    // Start is called before the first frame update
    void Start()
    {
        //StartCoroutine(GhostcreatorCoroutine);
    }

    private IEnumerator GhostCreator()
    {
        while (true)
        {

                saveState(dobot);
                if (debug)
                {
                    x++;
                    if (x % 1100 == 0)
                    {
                        showTimeGhosts(1000);
                        x = 0;
                    }
                    else if (x % 600 == 0)
                    {
                        hideAllGhosts();
                    }
                
            }

            yield return new WaitForSeconds(save_frequency);
        }
    }

    // Update is called once per frame
    void Update()
    {
        //todo outsource this to a coroutine that runs every 0.25 seconds or so
        //every frame is just to much
        //paused or only do this while game time actually progresses
        
    }

    //todo beim wiederherstellen kann man wharscheinlich mashs einfach 

    public void showTimeGhosts(int limit=25,int stepSize=5)
    {
        if (GeisterZeit)
        {
            StopGhostCreation();
            currentGhosts = new System.Collections.ArrayList(limit / stepSize);
            for (int i = 0; i < limit; i++)
            {
                if (i % stepSize == 0)
                {
                    GameObject curr = getState(i);
                    float opacity = 0.5f * (1f - ((i + 1f) / (float)limit));
                    fadeOut(curr, null, opacity);
                    removeAllNonGhostScripts(curr);
                    freezeGhostPosition(curr);
                    DestroyColliders(curr);
                    curr.SetActive(true);
                    currentGhosts.Add(curr);
                }

            }
            spookyTime = true;
        }
    }


    
    public void hideAllGhosts()
    {
        //first let's disable all ghosts
        if (currentGhosts != null)
        {
            foreach (GameObject current in currentGhosts)
            {
                current.SetActive(false);
                Destroy(current);
            }
        }
        //delete reference to ghosts
        // in case the references to these ghosts are still in the stateArray they can be recreated
        // otherwise the GC will take care of them and this call frees lot's
        // of memory
        currentGhosts = null;
        spookyTime = false;
        StartGhostCreation();
    }


    public void freezeGhostPosition(GameObject currentGhost) {
        Rigidbody[] transforms = currentGhost.GetComponentsInChildren<Rigidbody>();
        foreach(Rigidbody t in transforms)
        {
            t.constraints = RigidbodyConstraints.FreezeAll;
           
        }
    }
    
    public void fadeOut(GameObject parent, Color? fadedTone=null, float? opacity=null)
    {
        Renderer[] renderers = parent.GetComponentsInChildren<Renderer>();
        Color faded = fadedTone ?? new Color(1f, 1f, 1f);
         
        faded.a = opacity ?? 0.3f;
        foreach (Renderer r in renderers)
        {
            Material[] materials = r.materials;
            foreach (Material m in materials)
            {
                m.color = faded;
            }
        }
    }

    public void removeAllNonGhostScripts(GameObject currentGhost)
    {
        UnityEngine.Object[] scripts = (UnityEngine.Object[])(currentGhost.GetComponentsInChildren<MonoBehaviour>());
        foreach (UnityEngine.Object current in scripts)
        {
            if(!current.GetType().Name.ToLower().Contains("urdf"))
            Destroy(current);
        }
        scripts = (UnityEngine.Object[])(currentGhost.GetComponentsInChildren<MonoBehaviour>());
        foreach (UnityEngine.Object current in scripts)
        {
                Destroy(current);
        }

    }

    //destroys all collider of currentGhost so the objects really behave like Ghosts and can pass through everything
    public void DestroyColliders(GameObject currentGhost)
    {
        Collider[] childColliders = currentGhost.GetComponentsInChildren<Collider>();
        foreach (Collider collider in childColliders)
        {
            Destroy(collider);
        }
        
        foreach(UnityEngine.Object currentObject in currentGhost.GetComponentsInChildren<HingeJoint>())
        {
            Destroy(currentObject);
        }
        foreach (UnityEngine.Object currentObject in currentGhost.GetComponentsInChildren<ConfigurableJoint>())
        {
            Destroy(currentObject);
        }
        foreach (UnityEngine.Object currentObject in currentGhost.GetComponentsInChildren<SphereCollider>())
        {
            Destroy(currentObject);
        }
        foreach (UnityEngine.Object currentObject in currentGhost.GetComponentsInChildren<BoxCollider>())
        {
            Destroy(currentObject);
        }
        foreach (UnityEngine.Object currentObject in currentGhost.GetComponentsInChildren<Rigidbody>())
        {
            Destroy(currentObject);
        }

    }
    //
    protected void saveState(GameObject currentObj)
    {
        statePointer++;
        if (statePointer > stateArray.Length - 1)
        {
            statePointer = 0;
        }
        //todo we think instantiate clones the current version including it's orientation
        GameObject clone = Instantiate(currentObj);
        //deactivate the clone so it doesn't collide with the original object
        clone.SetActive(false);
        removeAllNonGhostScripts(clone);
        DestroyColliders(clone);
        // move clone to the same position as the original
        clone.transform.position = currentObj.transform.position;
         //scale clone the same way
         clone.transform.localScale = currentObj.transform.localScale;
        //destroy the object at the old location
        if (stateArray[statePointer] != null)
        {
        
            GameObject old = stateArray[statePointer];
            //Destroy the old object only if it is currently not shown
            // we determine if it is currentlyt shown by checking wether it
            // it is in the currenGhosts list (which might also be null)
            if (currentGhosts == null)
            {
                Destroy(old);
            }else{
                if (!currentGhosts.Contains(old))
                {
                    Destroy(old);
                }
            }
            
        }
        stateArray[statePointer] = clone;

    }

    protected GameObject getState(int position=0)
    {
        GameObject result = null;

        if (Math.Abs( position) > stateArray.Length)
        {
            throw new ArgumentException("Can't go back that far, sorry :( Try a lower value.");
        }
        if(position < 0)
        {
            throw new ArgumentException("Input values smaller than 0 mean, that this code should predict the future." +
                " I just can't do this. Sorry, dude I ain't no fortune teller.");
        }

        int targetPos = statePointer - position;
        if (targetPos < 0)
        {
            // if target pos is smaller 0, this means that we wrapped around the array in memory aand need to start in the back
            targetPos = stateArray.Length + targetPos;
        }
        result = stateArray[targetPos];
        return result;
    }
    public bool ToggleGeisterzeit()
    {
        if (GeisterZeit)
        {
           return StopGhostCreation();
        }
        else
        {
            return StartGhostCreation();
        }
    }

    private bool StopGhostCreation()
    {
        GeisterZeit = false;
        StopCoroutine(GhostcreatorCoroutine);
        return GeisterZeit;
    }

    private bool StartGhostCreation()
    {
        GeisterZeit = true;
        StartCoroutine(GhostcreatorCoroutine);
        return GeisterZeit;
    }
}
