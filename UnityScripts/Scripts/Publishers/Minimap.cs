using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.UnityRoboticsDemo;

public class Minimap : MonoBehaviour
{
    private ROSConnection ros;
    public Camera minimapCamera;
    public GameObject waypointMarkerPrefab;
    public LineRenderer lineRenderer;
    public GameObject arrowHead;
    public Transform worldParent;
    public float worldSize;
    public float minimapSize;
    public string topicName = "pos_rot";

    private GameObject waypointMarker;
    private Vector3 originPosition;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PosRotMsg>(topicName);
    }

    private void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            // reset line renderer
            lineRenderer.SetPosition(0, Vector3.zero);
            lineRenderer.SetPosition(1, Vector3.zero);
            Vector3 mousePosition = Input.mousePosition;
            originPosition = minimapCamera.ScreenToWorldPoint(new Vector3(mousePosition.x, mousePosition.y, minimapCamera.nearClipPlane));

            Vector3 worldPosition = originPosition * (worldSize / minimapSize);

            if (waypointMarker != null)
            {
                Destroy(waypointMarker);
            }

            waypointMarker = Instantiate(waypointMarkerPrefab, worldPosition, Quaternion.identity);
            waypointMarker.transform.SetParent(worldParent);

            worldPosition.z = 3;
            // set the initial position of the line renderer
            lineRenderer.SetPosition(0, worldPosition);
            lineRenderer.startColor = Color.red;
            lineRenderer.endColor = Color.blue;
        }
        else if (Input.GetMouseButton(0))
        {
            Vector3 mousePosition = Input.mousePosition;
            Vector3 currentPos = minimapCamera.ScreenToWorldPoint(new Vector3(mousePosition.x, mousePosition.y, -5)) * (worldSize / minimapSize);

            // update the second position of the line renderer
            currentPos.z = 3;
            lineRenderer.SetPosition(1, currentPos);
            
            Vector3 ArrowDirection = (lineRenderer.GetPosition(1) - lineRenderer.GetPosition(0)).normalized;
            float zRotation = Mathf.Atan2(ArrowDirection.y, ArrowDirection.x) * Mathf.Rad2Deg;

            arrowHead.transform.position = currentPos;
            arrowHead.transform.rotation = Quaternion.Euler(0f, 0f, zRotation);
        }
        else if (Input.GetMouseButtonUp(0))
        {
            PosRotMsg cubePos = new PosRotMsg(
                -originPosition.y,
                -originPosition.x,
                originPosition.z,
                arrowHead.transform.rotation.x,
                arrowHead.transform.rotation.y,
                arrowHead.transform.rotation.z,
                arrowHead.transform.rotation.w
            );

            ros.Publish(topicName, cubePos);
        
        }
        
    }
}