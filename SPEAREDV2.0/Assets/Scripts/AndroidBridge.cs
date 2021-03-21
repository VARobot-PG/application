using UnityEngine;
using UnityEngine.UI;
using static AndroidBridgeUtils;

public class AndroidBridge : MonoBehaviour, IAndroidBridge
{
    [SerializeField] private Button startListeningBtn = null;
    private SpeechCommandController ctrl;

    void Start()
    {
        this.gameObject.name = ANDROIDBRIDGE_GO_NAME;

        startListeningBtn.onClick.AddListener(StartListening);
        ctrl = this.GetComponent<SpeechCommandController>();
    }

    private void StartListening()
    {
        AndroidRunnableCall("StartListening");
    }

    private void SetContinuousListening(bool isContinuous)
    {
        AndroidCall("SetContinuousListening", isContinuous);
    }

    public void OnResult(string recognizedResult)
    {
        this.ctrl.checkForKeyWord(recognizedResult.Replace("~"," "));
    }

}
