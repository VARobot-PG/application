using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.UI;
using RosSharp;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using UnityEngine;

public class CoordinateTargetSelector : MonoBehaviour
{
    public GameObject selectedTarget;
    public Transform selectedRosTransform;
    public Material selectedColorPrefab;
    public Material unselectedColorPrefab;
    
    public Transform rootSphereTransform;
    public Transform exampleSphere1; // 2 solutions
    public Transform exampleSphere2;
    public Vector3 exampleROSVector;
    public Transform currentROSCoords;
    public bool useTransform;
    public Transform baseLink;
    public Transform handLink;
    public Vector3 offset;
    public bool changeOffset;
    public bool useOffset;
    String pathRos = "Assets/.junk/coords_ros";
    String pathUG = "Assets/.junk/coords_unity_global";
    String pathUL = "Assets/.junk/coords_unity_local";
    string fileEnding = ".txt";
    Vector3 oldVector = Vector3.zero;
    public float scale = 1000.0f;
    // Start is called before the first frame update
    void Start()
    {
        this.rootSphereTransform.gameObject.SetActive(true);
        this.addOnClickHandlerAndBoundingBoxes();
      //  this.addOnClickHandlerAndBoundingBoxes("RealTargetSphere");
        oldVector = Vector3.zero;

    }

    public void writeRosNodesToFile()
    {

        String timeStamp = DateTime.Now.ToString();
        //String outputUnityGlobal = "Unityglobal";
        //String outputROS = "ROS";
        //String outputUnityLocal = "UnityLocal";
        String outputUnityGlobal = "";
        String outputROS = "";
        String outputUnityLocal = "";
        timeStamp = timeStamp.Replace(":", "-");
        timeStamp = timeStamp.Replace(".", "_");
        timeStamp = timeStamp.Replace(" ", "_");
        FileStream createdFile = File.Create(pathRos + timeStamp + fileEnding);
        createdFile.Close();
        createdFile = File.Create(pathUG + timeStamp + fileEnding);
        createdFile.Close();
        createdFile = File.Create(pathUL + timeStamp + fileEnding);
        createdFile.Close();
        foreach (Transform coordTransform in rootSphereTransform.GetComponentsInChildren<Transform>())
        {
            if (coordTransform.name.Contains("CoordinateSphere"))
            {
                outputUnityGlobal += $"{coordTransform.position.x.ToString("F9")} {coordTransform.position.x.ToString("F9")} {coordTransform.position.z.ToString("F9")}\n";
                outputUnityLocal += $"{coordTransform.localPosition.x.ToString("F9")} {coordTransform.localPosition.y.ToString("F9")} {coordTransform.localPosition.z.ToString("F9")}\n";
            }


        }
        foreach (Transform coordTransform in rootSphereTransform.GetComponentsInChildren<Transform>())
        {
            if (coordTransform.name.Contains("ROS"))
            {
                //outputROS += coordTransform.localPosition.ToString("F9");
                outputROS += $"{coordTransform.localPosition.x.ToString("F9")} {coordTransform.localPosition.y.ToString("F9")} {coordTransform.localPosition.z.ToString("F9")}\n";
                //Debug.Log($"Converted ROS2Unity : {coordTransform.localPosition.ToString()} -> Local: {transformVectorRos2Unity(coordTransform,true).ToString("F4")}");
            }

        }
        outputUnityGlobal = outputUnityGlobal.Replace(",", ".");
        outputROS = outputROS.Replace(",", ".");
        outputUnityLocal = outputUnityLocal.Replace(",", ".");
        StreamWriter writer = new StreamWriter(pathRos + timeStamp + fileEnding, true);
        writer.Write(outputROS);
        writer.Close();
        writer = new StreamWriter(pathUG + timeStamp + fileEnding, true);
        writer.Write(outputUnityGlobal);
        writer.Close();
        writer = new StreamWriter(pathUL + timeStamp + fileEnding, true);
        writer.Write(outputUnityLocal);
        writer.Close();
        Debug.Log(outputUnityGlobal);
        Debug.Log(outputUnityLocal);
        Debug.Log(outputROS);
    }

    // Update is called once per frame
    void Update()
    {

        updateTestTransform();
    }
    public void addOnClickHandlerAndBoundingBoxes()
    {
        addOnClickHandlerAndBoundingBoxes("CoordinateSphere");
    }
        public void addOnClickHandlerAndBoundingBoxes(string transformName)
    {

        foreach (Transform coordTransform in rootSphereTransform.GetComponentsInChildren<Transform>())
            {
            if (coordTransform.name.Contains(transformName))
            {
                
                BoundingBox boundingBox = coordTransform.AddComponentIfNotExists<BoundingBox>();
                boundingBox.ScaleHandleSize = 0f;
                boundingBox.RotationHandleSize = 0f;
                boundingBox.ShowScaleHandles = false;
                boundingBox.ShowRotationHandleForX = false;
                boundingBox.ShowRotationHandleForZ = false;
                boundingBox.ShowRotationHandleForY = false;
                boundingBox.BoundingBoxActivation = BoundingBox.BoundingBoxActivationType.ActivateByProximityAndPointer;
                boundingBox.ShowWireFrame = false;
                PointerHandler pointerHandler = coordTransform.AddComponentIfNotExists<PointerHandler>();
                pointerHandler.OnPointerClicked.AddListener((mixedRealityEvent) =>
                {
                    switchTarget(coordTransform.gameObject);
                });

            }
                
            }
        }

  
    public void switchTarget(GameObject newTarget)
    {
        MeshRenderer meshRenderer;
        if (selectedTarget != null)
        {
      //      meshRenderer = selectedTarget.GetComponent<MeshRenderer>();
       //     meshRenderer.material = unselectedColorPrefab;
        }
        selectedTarget = newTarget;
        //meshRenderer = selectedTarget.GetComponent<MeshRenderer>();
        //meshRenderer.material = selectedColorPrefab;
        
        foreach(Transform childTransforms in selectedTarget.transform)
        {
            if (childTransforms.name.Contains("ROSCoords"))
            {
                selectedRosTransform = childTransforms;
            }
        }


    }
    public void transformVectorRos2Unity(bool resultLocal = true)
    {
        Debug.Log(this.transformVectorRos2Unity(selectedRosTransform, resultLocal).ToString("3f"));
    }
    public Vector3 transformVectorRos2Unity(Transform rosTransform, bool resultLocal)
    {
        if(resultLocal)
        return transformVectorRos2UnityLocal(rosTransform.localPosition);
        else
            return transformVectorRos2UnityGlobal(rosTransform.localPosition);
    }
    public Vector3 transformVectorRos2UnityLocal(Vector3 rosVector)
    {
        return testROS2UnityLocal(rosVector);
        /*
        Vector3 targetUnityVector = Vector3.zero;
        /*200
         Vector3 translationVectorRos2Unity = new Vector3(-0.5494738593f, 0.1381076802f -0.0088757968f);


            //-0.0096525863 - 0.9997534069 0.0199988240
            //0.0026585418 0.0199740272 0.9997969646
            //0.9999498786 - 0.0097037942 - 0.0024650852
            Vector4 col1 = new Vector4(-0.0096525863f, -0.9997534069f, 0.0199988240f);
            Vector4 col2 = new Vector4(0.0026585418f, 0.0199740272f, 0.9997969646f);
            Vector4 col3 = new Vector4(0.9999498786f, -0.0097037942f, -0.0024650852f);
            float scaleRos2Unity = 0.0007740286f;
            */
        /*
        // 400
        
        //-0.5365136955 0.1249480934 - 0.0242361201
        Vector3 translationVectorRos2Unity = new Vector3(-0.5365136955f, 0.1249480934f, -0.0242361201f);


        // -0.0011362229 -0.9999991950 -0.0005647168
        //-0.0161410875 - 0.0005463037 0.9998695749
        //0.9998690786 - 0.0011451899 0.0161404538
        Vector4 col1 = new Vector4(-0.0011362229f, -0.0161410875f, 0.9998690786f);
        Vector4 col2 = new Vector4(-0.9999991950f, -0.0005463037f, -0.0011451899f);
        Vector4 col3 = new Vector4(-0.0005647168f, 0.9998695749f, 0.0161404538f);


        Matrix4x4 rotationMatrixRos2Unity = new Matrix4x4(col1, col2, col3, Vector4.zero);
        //0.0008408325
        float scaleRos2Unity = 0.0008408325f;
        */
        // 1200 points
        //-0.5377505481 0.1281032867 -0.02156320241
       // Vector3 translationVectorRos2Unity = new Vector3(-0.5377505481f, 0.1281032867f, -0.02156320241f);


        // -0.0011362229 -0.9999991950 -0.0005647168
        //-0.0161410875 - 0.0005463037 0.9998695749
        //0.9998690786 - 0.0011451899 0.0161404538
     //   vector4 col1 = new vector4(-0.0011362229f, -0.0161410875f, 0.9998690786f);
      //  Vector4 col2 = new Vector4(-0.9999991950f, -0.0005463037f, -0.0011451899f);
      //  Vector4 col3 = new Vector4(-0.0005647168f, 0.9998695749f, 0.0161404538f);


        //Matrix4x4 rotationMatrixRos2Unity = new Matrix4x4(col1, col2, col3, Vector4.zero);
        //0.0008330475
        //float scaleRos2Unity = 0.0008330475f;

        //targetUnityVector = translationVectorRos2Unity + scaleRos2Unity * rotationMatrixRos2Unity.MultiplyPoint3x4(rosVector);
        //return targetUnityVector;
        
    }
    public Vector3 transformVectorRos2UnityGlobal(Vector3 rosVector)
    {
        Vector3 targetUnityVector = Vector3.zero;
        /*
        
        Vector3 translationVectorRos2Unity = new Vector4(0.5243008213f, -0.0528922945f, 0.9902718240f);
        // with 20 points
        //0.3510736742 0.9361419315 - 0.0196356671
        //0.0026580118 0.0199740251 0.9997969660
        //- 0.9363440662 0.3510545862 - 0.0045240781
        Vector4 col1 = new Vector4(0.3510736742f, 0.0026580118f, -0.9363440662f);
        Vector4 col2 = new Vector4(0.9361419315f, 0.0199740251f, 0.3510545862f);
        Vector4 col3 = new Vector4(-0.0196356671f, 0.9997969660f, -0.0045240781f);
        float scaleRos2Unity = 0.0007740288f;
        */
        /*
        // with 400 points
        // 0.5068687531 -0.0660519129 1.0002730825
        Vector3 translationVectorRos2Unity = new Vector4(0.5068687531f, -0.0660519129f, 1.0002730825f);
        //0.3430432835 0.9393001070 0.0060509958
        //- 0.0161411067 - 0.0005462667 0.9998695746
        //- 0.9391809039 0.3430962117 - 0.0149739501
        Vector4 col1 = new Vector4(0.3430432835f, -0.0161411067f, -0.9391809039f);
        Vector4 col2 = new Vector4(0.9393001070f, -0.0005462667f, 0.3430962117f);
        Vector4 col3 = new Vector4(0.0060509958f, 0.9998695746f, -0.0149739501f);        
        Matrix4x4 rotationMatrixRos2Unity = new Matrix4x4(col1, col2, col3, Vector4.zero);
        // 0.0008408326
        float scaleRos2Unity = 0.0008408326f;
        */

        //1200 points



        // 0.5089452248 -0.0628967147 0.9981844123
        Vector3 translationVectorRos2Unity = new Vector4(0.5089452248f, - 0.0628967147f, 0.9981844123f);
        //0.3434982918 0.9391514041 - 0.0018877596
        //0.0018210036 0.0013440252 0.9999974388
        //- 0.9391515359 0.3435008497 0.0012485278
        Vector4 col1 = new Vector4(0.3434982918f, 0.0018210036f, -0.9391515359f);
        Vector4 col2 = new Vector4(0.9391514041f, 0.0013440252f, 0.3435008497f);
        Vector4 col3 = new Vector4(-0.0018877596f, 0.9999974388f, 0.0012485278f);
        Matrix4x4 rotationMatrixRos2Unity = new Matrix4x4(col1, col2, col3, Vector4.zero);
        // 0.0008330475
        float scaleRos2Unity = 0.0008330475f;
        targetUnityVector = translationVectorRos2Unity + scaleRos2Unity * rotationMatrixRos2Unity.MultiplyPoint3x4(rosVector);
        return targetUnityVector;
    }
    public Vector3 transformVectorUnityGlobal2ROS(Vector3 unityVector)
    {
        /* 20
        Vector3 translationVectorUnity2Ros = new Vector3(930.3284524098f, -1038.5184308119f, 89.0990106716f);
        //0.3510736742 0.0026580118 - 0.9363440662
        //0.9361419315 0.0199740251 0.3510545862
        //- 0.0196356671 0.9997969660 - 0.0045240781
        Matrix4x4 rotationMatrixUnity2Ros = new Matrix4x4(new Vector4(0.3510736742f, 0.9361419315f, -0.0196356671f), new Vector4(0.0026580118f, 0.0199740251f, 0.9997969660f), new Vector4(-0.9363440662f, 0.3510545862f, -0.0045240781f), Vector4.zero);
        float sclaeUnity2Ros = 1245.7490968839f;
        */
        /*
        // 400 points
        //878.4965302065 - 929.3878908311 92.5931559356
        Vector3 translationVectorUnity2Ros = new Vector3(878.4965302065f, -929.3878908311f, 92.5931559356f);

        //0.3430432835 - 0.0161411067 - 0.9391809039
        //0.9393001070 - 0.0005462667 0.3430962117
        //0.0060509958 0.9998695746 - 0.0149739501
        Vector4 col1 = new Vector4(0.3430432835f, 0.9393001070f, 0.0060509958f);
        Vector4 col2 = new Vector4(-0.0161411067f, -0.0005462667f, 0.9998695746f);
        Vector4 col3 = new Vector4(-0.9391809039f, 0.3430962117f, -0.0149739501f);
        Matrix4x4 rotationMatrixUnity2Ros = new Matrix4x4(col1, col2, col3, Vector4.zero);
        
            

            Vector3 targetRosVector = Vector3.zero;

        //1141.4931181775
        float sclaeUnity2Ros = 1141.4931181775f;
        */

        //1200



        //883.3160330311 -937.7158478066 76.4611239278                   
        Vector3 translationVectorUnity2Ros = new Vector3(883.3160330311f, - 937.7158478066f, 76.4611239278f);

        //0.3434982918 0.0018210036 - 0.9391515359
//0.9391514041 0.0013440252 0.3435008497
//- 0.0018877596 0.9999974388 0.0012485278
        Vector4 col1 = new Vector4(0.3434982918f, 0.0018210036f, -0.9391515359f);
        Vector4 col2 = new Vector4(0.9391514041f, 0.0013440252f, 0.3435008497f);
        Vector4 col3 = new Vector4(-0.0018877596f, 0.9999974388f, 0.0012485278f);
        Matrix4x4 rotationMatrixUnity2Ros = new Matrix4x4(col1, col2, col3, Vector4.zero);
        //1150.0514958345
            float sclaeUnity2Ros = 1150.0514958345f;

        Vector3 targetRosVector = Vector3.zero;
        targetRosVector = translationVectorUnity2Ros + sclaeUnity2Ros * rotationMatrixUnity2Ros.MultiplyPoint3x4(unityVector);
        return targetRosVector;
    }
    public Vector3 transformVectorUnityLocal2ROS(Vector3 unityVector)
    {
        //20 points
        /*
        Vector3 translationVectorUnity2Ros = new Vector3(8.3455987256f, -683.2060663986f, -153.5346915035f);
        // -0.0096525863f 0.0026585418f 0.9999498786f
        // -0.9997534069f 0.0199740272f -0.0097037942f
        // 0.0199988240f 0.9997969646f -0.0024650852f
        Vector4 col1 = new Vector4(-0.0096525863f, 0.0026585418f, 0.9999498786f);
        Vector4 col2 = new Vector4(-0.9997534069f, 0.0199740272f, -0.0097037942f);
        Vector4 col3 = new Vector4(0.0199988240f, 0.9997969646f, -0.0024650852f);
        Matrix4x4 rotationMatrixUnity2Ros = new Matrix4x4(col1, col2, col3, Vector4.zero);
        Vector3 targetRosVector = Vector3.zero;
        float sclaeUnity2Ros = 1245.7492795758f;
        */
        /*
        // 400 points
        //35.1007360869 -606.5107733880 -138.8992247508
        Vector3 translationVectorUnity2Ros = new Vector3(35.1007360869f, -606.5107733880f, -138.8992247508f);
      //  -0.0011362229 - 0.0161410875 0.9998690786
    //- 0.9999991950 - 0.0005463037 - 0.0011451899
    //- 0.0005647168 0.9998695749 0.0161404538
        Vector4 col1 = new Vector4(-0.0011362229f, -0.9999991950f, -0.0005647168f);
        Vector4 col2 = new Vector4(-0.0161410875f, -0.0005463037f, 0.9998695749f);
        Vector4 col3 = new Vector4(0.9998690786f, 0.0011451899f, 0.0161404538f);
        Matrix4x4 rotationMatrixUnity2Ros = new Matrix4x4(col1, col2, col3, Vector4.zero);
        Vector3 targetRosVector = Vector3.zero;
        //1141.4931858724
        float sclaeUnity2Ros =  1141.4931858724f;
        */


        //1200 points

        /*
         -0.0015740044 0.0018209972 0.9999971032 
          -0.9999978542 0.0013440467 -0.0015764531 
         0.0013469135 0.9999974388 -0.0018188777 
         
         
         * 
         */
        //29.6837553784 -612.4655291149 -142.0819409999 

        Vector3 translationVectorUnity2Ros = new Vector3(29.6837553784f, - 612.4655291149f, - 142.0819409999f);
                //-0.0015740044 0.0018209972 0.9999971032
                //- 0.9999978542 0.0013440467 - 0.0015764531
                //0.0013469135 0.9999974388 - 0.0018188777
        Vector4 col1 = new Vector4(-0.0015740044f, -0.9999978542f, 0.0013469135f);
        Vector4 col2 = new Vector4(0.0018209972f, 0.0013440467f, 0.9999974388f);
        Vector4 col3 = new Vector4(0.9999971032f, -0.0015764531f, -0.0018188777f);
        Matrix4x4 rotationMatrixUnity2Ros = new Matrix4x4(col1, col2, col3, Vector4.zero);
        Vector3 targetRosVector = Vector3.zero;
        //1150.0515346607
        float sclaeUnity2Ros = 1150.0515346607f;
        targetRosVector = translationVectorUnity2Ros + sclaeUnity2Ros * rotationMatrixUnity2Ros.MultiplyPoint3x4(unityVector);
        return targetRosVector;
    }
    public void updateTestTransform()
    {
        if (useTransform)
        {
            this.exampleROSVector = currentROSCoords.position;
        }
        
        if (oldVector.Equals(exampleROSVector))
        {
         
        }
        else
        {
            oldVector = exampleROSVector;
            exampleSphere1.localPosition = testROS2UnityLocal(exampleROSVector); 
        }
    }
    public Vector3 testROS2UnityLocal(Vector3 rosVector)
    {
        float armExtension = 65.5f; //refer to dobot/include/dobot.h

        Vector3 rosScaled = rosVector / scale;
        Vector3 unityjustScaled = new Vector3(-rosScaled.y, rosScaled.z, rosScaled.x); 

        return unityjustScaled;


    }
    public Vector3 testUnityLocal2ROS(Vector3 unityLocalVector)
    {
        Vector3 unityLocalVectorScaled = unityLocalVector * scale;
        Vector3 rosVector = new Vector3(unityLocalVectorScaled.z, -unityLocalVectorScaled.x, unityLocalVectorScaled.y);
        return rosVector;
        //robotnameTransform
        //  RosSharp.RosBridgeClient.MessageTypes.Geometry.Transform
        //Transform robotTransform = 
        //Transform inverseRobotTransform = robotTransform.inv


    }

}

