using UnityEngine;
using System.Collections.Generic;
using RosSharp.RosBridgeClient.MessageTypes.Visualization;

namespace RosSharp.RosBridgeClient
{
    public class MarkerHandler : MonoBehaviour
    {
        private List<Marker> markers_traj = new List<Marker>();
        private List<Marker> markers_plan = new List<Marker>();
        private List<GameObject> spawnedObjects = new List<GameObject>();
        private List<Marker> drawnMarkers = new List<Marker>();
        public float gizmoRadius = 0.0001f;
        public bool showGizmos = true;
        public GameObject coordinateSpherePrefabPlan;
        public GameObject coordinateSpherePrefabTraj;
        public CoordinateTargetSelector coordinateTargetSelector;
        public Transform generatedMarkers;
        public MarkerHandler()
        {

        }
        private void FixedUpdate()
        {

            if (showGizmos)
            {
                GameObject prefabObject = coordinateSpherePrefabPlan;
                foreach (Marker currMarker in markers_plan)
                {
                    if(!drawnMarkers.Contains(currMarker))
                    drawMarkerSpheres(currMarker, prefabObject);

                }
                prefabObject = coordinateSpherePrefabTraj;
                foreach (Marker currMarker in markers_traj)
                {
                    if (!drawnMarkers.Contains(currMarker))
                        drawMarkerSpheres(currMarker, prefabObject);
                }
            }
        }
        private void drawMarkerSpheres(Marker targetMarker, GameObject prefabObject)
        {
            Vector3 markerVector = new Vector3((float)targetMarker.pose.position.x, (float)targetMarker.pose.position.y, (float)targetMarker.pose.position.z);
            markerVector = markerVector * 1000f;
            Vector3 scale = new Vector3(gizmoRadius, gizmoRadius, gizmoRadius);
            GameObject endObject = Instantiate(prefabObject);
            endObject.transform.SetParent(generatedMarkers, false);
            foreach (Transform child in endObject.transform)
            {
                if (child.name.StartsWith("ROSCoords"))
                {
                    child.localPosition = markerVector;
                }
            }
            endObject.transform.localPosition = coordinateTargetSelector.testROS2UnityLocal(markerVector);
            endObject.transform.localScale = scale;

            // spawnedObjects.Add(startObject);
            spawnedObjects.Add(endObject);
            drawnMarkers.Add(targetMarker);
        }
      

        public void clearMarkers()
        {
            this.markers_plan = new List<Marker>();
            this.markers_traj = new List<Marker>();
            this.drawnMarkers = new List<Marker>();
            foreach (GameObject current in spawnedObjects)
            {
                Destroy(current);
            }
            //now that all objects in the list have been deleted and removed from
            // unity engine we can throw away the list
            spawnedObjects = new List<GameObject>();
        }

        public void switchState()
        {
            this.showGizmos = !this.showGizmos;
        }

        public bool receiveMarker(Marker marker)
        {
            //Debug.Log("Recieved marker with seq_id: " + marker.header.seq + " ns: " + marker.ns);
            bool res = true;
            try
            {
                if (marker.ns.Equals("goalPlan"))
                {
                    markers_plan.Add(marker);
                }
                else if (marker.ns.Equals("goalTraj"))
                {
                    markers_traj.Add(marker);
                }
            }
            catch
            {
                res = false;
            }
            return res;
        }
    }
}
