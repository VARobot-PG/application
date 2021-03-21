using RosSharp;
using System.Collections;
using System.Collections.Generic;
using TMPro;
using UnityEngine;

public class LocationObject : MonoBehaviour
{
     Vector3 numerators = new Vector3(215, 1, 145);
     Vector3 denominators = new Vector3(0.07089876f, 0.1785709f, 0.6962211f);
     Vector3 fixedValueModifier;
    public Transform objectTransform;
    public Vector3 transformedLocation;
    public TextMeshProUGUI text;
     float scale;
     Vector3 targetScaled;
     Vector3 targetOffset;
     Vector3 pos3Scaled;
     float distanceDobot;
     float distanceUnity;
     Vector3 targetSwapped;
     Vector3 target;
    public Vector3 objectTransformPosition;
     float distanceDobotX;
     float distanceDobotY;
     float distanceDobotZ;
     float distanceUnityX;
     float distanceUnityY;
     float distanceUnityZ;
     float scaleX;
     float scaleY;
     float scaleZ;
     Vector3 unity2RosV3;
     Vector3 scaledDobotV3;
     Vector3 offset;
     Vector3 unityCoords;
     Vector3 unity2DobotScale;
    public Vector3 invertedScaleUN_DO;
    public Vector3 coords1;
    public Vector3 coords2;
    public Vector3 unityCoords1;
    public Vector3 invertedScaleDO_UN;
    public Vector3 invertedScaleDO;
    public Vector3 invertedScaleUN;
    public Vector3 scaledDO;
    public Vector3 scalenUN;
    public Vector3 minus_unityCoords1;
    public Vector3 minus_coords1;
    Vector3 unityCoords2;
    Vector3 unityCoords3;
    public float gizmoRadius;
    // Start is called before the first frame update
    private void OnDrawGizmos()
    {
        Gizmos.color = Color.green;
        Gizmos.DrawSphere(unityCoords1,  gizmoRadius);
        Gizmos.color = Color.red;
        Gizmos.DrawSphere(unityCoords2,  gizmoRadius);
        Gizmos.color = Color.yellow;
        Gizmos.DrawSphere(unityCoords3,  gizmoRadius);
    }
    void Start()
    {
        
         unityCoords1 = new Vector3(0.07345818f, 0.2593342f, 0.6931722f);
         unityCoords2 = new Vector3(0.07798947f, 0.162906f, 0.7468626f);
         unityCoords3 = new Vector3(0.2011681f, 0.4018536f, 0.890424f);
         Vector3 dobotCoords1 = new Vector3(215, 4, 145);
        Vector3 dobotCoordsSwappedXYZ1 =dobotCoords1.Ros2Unity();
        //dobotCoordsSwappedXYZ1 = -dobotCoordsSwappedXYZ1;
        Vector3 dobotCoords2 = new Vector3(176, 24, 79);
        Vector3 dobotCoordsSwappedXYZ2 = dobotCoords2.Ros2Unity();
        Vector3 dobotCoords3 = new Vector3(78, 200, 241);
        Vector3 dobotCoordsSwappedXYZ3 = dobotCoords3.Ros2Unity();
        //dobotCoordsSwappedXYZ2 = -dobotCoordsSwappedXYZ2;
        distanceDobot = Vector3.Distance(dobotCoordsSwappedXYZ1, dobotCoordsSwappedXYZ3);
        distanceDobotX = dobotCoordsSwappedXYZ3.x - dobotCoordsSwappedXYZ1.x;
        distanceDobotY = dobotCoordsSwappedXYZ3.z - dobotCoordsSwappedXYZ1.y;
        distanceDobotZ = dobotCoordsSwappedXYZ3.z - dobotCoordsSwappedXYZ1.y;
        distanceUnityX = unityCoords3.x - unityCoords1.x;
        distanceUnityY = unityCoords3.y - unityCoords1.y;
        distanceUnityZ = unityCoords3.z - unityCoords1.z;
        distanceUnity = Vector3.Distance(unityCoords1, unityCoords3);
        scaleX = distanceDobotX / distanceUnityX;
        scaleY = distanceDobotY / distanceUnityY;
        scaleZ = distanceDobotZ / distanceUnityZ;
        invertedScaleDO_UN = new Vector3(scaleX, scaleY, scaleZ);
        invertedScaleUN_DO = new Vector3(distanceUnityX / distanceDobotX, distanceUnityY / distanceDobotY, distanceUnityZ / distanceDobotZ);
        invertedScaleDO = new Vector3(1 / distanceDobotX, 1 / distanceDobotY, 1 / distanceDobotZ);
        invertedScaleUN = new Vector3(1 / distanceUnityX, 1 / distanceUnityY, 1 / distanceUnityZ);
        scaledDO = new Vector3( distanceDobotX,  distanceDobotY,  distanceDobotZ);
        scalenUN = new Vector3(distanceUnityX, distanceUnityY,  distanceUnityZ);
        coords1 = dobotCoordsSwappedXYZ2;
        coords1.Scale(invertedScaleUN_DO);
        coords2= unityCoords2 - coords1;
        minus_coords1 = -coords1;
        minus_unityCoords1 = -unityCoords1;
        scale = distanceDobot / distanceUnity;
 

        pos3Scaled = new Vector3(unityCoords3.x * scaleX, unityCoords3.y * scaleY, unityCoords3.z * scaleZ);
        targetOffset = dobotCoordsSwappedXYZ3 - pos3Scaled;

        unityCoords = new Vector3(0.07344335f, 0.2593994f, 0.6932021f).Unity2Ros();
        Vector3 unity2RosInverse = new Vector3(1 / unityCoords.x, 1 / unityCoords.y, 1 / unityCoords.z);
        dobotCoords1 = new Vector3(215, 0, 145);
        unity2DobotScale = Vector3.Scale(dobotCoords1, unity2RosInverse);
        Vector3 unity0_200_0 = new Vector3(0.1231232f, 0.12f, 0.9398047f);
        Vector3 ros2unity_0_200_0 = new Vector3(0, 200, 0).Ros2Unity();
        Quaternion rotation = Quaternion.identity;
        GameObject templategameObject_ros2unity_0_200_0 = new GameObject("ros2unity_0_200_0");
        GameObject gameObject_ros2unity_0_200_0 = Instantiate(templategameObject_ros2unity_0_200_0, ros2unity_0_200_0, rotation);
        //gameObject_ros2unity_0_200_0.transform.parent = transform.root.parent;
        //gameObject_ros2unity_0_200_0.transform.position = ros2unity_0_200_0;
        GameObject templategamegameObject_0_200_0_Unity = new GameObject("Unity_0_200_0");
        GameObject gameObject_0_200_0_Unity = Instantiate(templategamegameObject_0_200_0_Unity, unity0_200_0, rotation, gameObject_ros2unity_0_200_0.transform);
//        gameObject_0_200_0_Unity.transform.position = unity0_200_0;
        //gameObject.transform.parent = gameObject_ros2unity_0_200_0.transform;
    }


    // Update is called once per frame
    void Update()
    {
        

        objectTransformPosition = objectTransform.position;
        unity2RosV3 = TransformExtensions.Unity2Ros(objectTransformPosition);
        unity2RosV3.Scale(unity2DobotScale);
        Matrix3x3 matrix = new Matrix3x3(new float[] { 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f, 1f });
        Matrix3x3 transposedMatrix = matrix.Transpose();

        
        
        //targetScaled = new Vector3(objectTransformPosition.x * scaleX, objectTransformPosition.y * scaleY, objectTransformPosition.z * scaleZ);
        //target = targetScaled + targetOffset;
        //targetSwapped = new Vector3(target.z, target.x, target.y);
        //target.x = ( objectTransform.position.x - fixedValueModifier.x ) * numerators.x / denominators.x;
        //target.y = (objectTransform.position.y - fixedValueModifier.y) * numerators.y / denominators.y;
        //target.z = (objectTransform.position.z - fixedValueModifier.z) * numerators.z / denominators.z;
        transformedLocation = unity2RosV3;
        text.SetText(transformedLocation.ToString());
    }
}
