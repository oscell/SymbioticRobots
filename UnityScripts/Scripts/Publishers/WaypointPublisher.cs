using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.UnityRoboticsDemo;

public class WaypointPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "pos_rot";

    // The game object
    public GameObject cubePrefab;

    // The instantiated cube
    private GameObject cubeInstance;

    void Start()
    {
        // start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<PosRotMsg>(topicName);
    }



    void Update()
    {
        if (Input.GetMouseButtonDown(0))
        {
            Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            RaycastHit hit;
            if (Physics.Raycast(ray, out hit))
            {
                // Destroy the previous cube if it exists
                if (cubeInstance != null)
                {
                    Destroy(cubeInstance);
                }

                // Instantiate a new cube at the clicked position
                cubeInstance = Instantiate(cubePrefab, hit.point, Quaternion.identity);

                PosRotMsg cubePos = new PosRotMsg(
                    -cubeInstance.transform.position.y,
                    -cubeInstance.transform.position.x,
                    cubeInstance.transform.position.z,
                    cubeInstance.transform.rotation.x,
                    cubeInstance.transform.rotation.y,
                    cubeInstance.transform.rotation.z,
                    cubeInstance.transform.rotation.w
                );

                // Publish the new cube's position and rotation
                ros.Publish(topicName, cubePos);
            }
        }
    }
}
