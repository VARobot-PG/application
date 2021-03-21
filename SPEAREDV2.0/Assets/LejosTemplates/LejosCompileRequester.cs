using System.Collections;
using System.Collections.Generic;
using System.Text;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.Networking;

public class CompiledEvent : UnityEvent<byte[]> { }

public class LejosCompileRequester : MonoBehaviour
{
    public string CompilationServiceUrl = "http://localhost:3000/api/compile";
    public UnityEvent<byte[]> compiled = new CompiledEvent();
    public IEnumerator RequestCompilation(string code)
    {
        WWWForm form = new WWWForm();
        form.AddField("code", code );
        form.AddField("namespace", "test2");
        form.AddField("classname", "HalloWelt");
        var request = UnityWebRequest.Post(CompilationServiceUrl, form);
        yield return request.SendWebRequest();
        if (request.isNetworkError || request.isHttpError)
        {
            Debug.Log(request.error);
            Debug.Log(request.responseCode);
        }
        else
        {
            compiled.Invoke(request.downloadHandler.data);
        }
    } 
}
