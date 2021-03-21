using Microsoft.MixedReality.Toolkit;
using Microsoft.MixedReality.Toolkit.Diagnostics;
using UnityEngine;

public class ProfilerToggle : MonoBehaviour
{
    public bool visible;

    public void toggleVisibility()
    {
        visible = !visible;
        CoreServices.DiagnosticsSystem.ShowProfiler = visible;
    }

    /*
     * Code for toggeling AR to on screen for the profiler, results in slightly tilted on screen profiler due to difficulcies in accessing it.
     * 
    private void toggleAR()
    {
        bool arOn = true; //if this code should be used, aron will be the variable for toggling ar and should be global (and not necessarily true)
        try
        {
            CoreServices.DiagnosticsSystem.ShowProfiler = visible;
            GameObject diagnostics = GameObject.Find("Diagnostics");
            MixedRealityToolkitVisualProfiler profiler = (MixedRealityToolkitVisualProfiler)diagnostics.GetComponent("MixedRealityToolkitVisualProfiler");

            if (arOn)
            {
                profiler.WindowAnchor = TextAnchor.LowerCenter;
                profiler.WindowFollowSpeed = 5;
                profiler.WindowScale = 1;
                profiler.WindowOffset = new Vector2(0.1f, 0.1f);
            }
            else
            {
                profiler.WindowAnchor = TextAnchor.UpperRight;
                profiler.WindowFollowSpeed = 100;
                profiler.WindowScale = 1.4f;
                profiler.WindowOffset = new Vector2(0.04f, 0.16f);
            }
        }
        catch (System.NullReferenceException)
        {
            Debug.LogWarning("You are trying to toggle the visibility of an object which is not referenced. You might have forgotten to add a GameObject as the objectToToggle in the ActivityToggle script.");
        }
    }
    */
}