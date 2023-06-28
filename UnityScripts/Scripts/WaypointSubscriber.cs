using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;

public class WaypointSubscriber : MonoBehaviour
{
    public GameObject waypointObject; // The object that will move to the new waypoint

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<PoseStampedMsg>("/move_base_simple/goal", HandleGoalMessage);
    }

    void HandleGoalMessage(PoseStampedMsg goalMessage)
    {
        var position = goalMessage.pose.position;
        var orientation = goalMessage.pose.orientation;

        // Convert ROS orientation (Quaternion) to Unity Quaternion and then to Euler angles for rotation
        Quaternion rosOrientation = new Quaternion((float)orientation.x, (float)orientation.y, (float)orientation.z, (float)orientation.w);
        Vector3 eulerRotation = rosOrientation.eulerAngles;

        // ROS uses right-handed coordinate system (Z up), Unity uses left-handed (Y up).
        // Here we assume that the waypoint is only in X and Y plane in ROS, which corresponds to X and Z in Unity.
        waypointObject.transform.position = new Vector3((float)position.y, -(float)position.x, -1);
        waypointObject.transform.rotation = Quaternion.Euler(-eulerRotation.z + 270, 90, 90);
    }
}
