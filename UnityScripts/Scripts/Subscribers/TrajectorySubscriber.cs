using UnityEngine;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Geometry;
using RosMessageTypes.Nav;
using RosMessageTypes.Std;

public class TrajectorySubscriber : MonoBehaviour
{
    public LineRenderer trajectoryLine;  // Assign a LineRenderer in the Unity editor
    private List<PoseStampedMsg> poseList = new List<PoseStampedMsg>();

    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<PathMsg>("/move_base/NavfnROS/plan", HandleTrajectoryMessage);
    }

    void HandleTrajectoryMessage(PathMsg trajectoryMessage)
    {
        poseList.Clear();  // Clear the list for the new message

        foreach (PoseStampedMsg poseStamped in trajectoryMessage.poses)
        {
            poseList.Add(poseStamped);  // Add each pose to the list
        }

        UpdateTrajectoryLine();
    }

    void UpdateTrajectoryLine()
    {
        trajectoryLine.positionCount = poseList.Count;  // Set the number of line segments

        // Loop through the list and set each line segment position
        for (int i = 0; i < poseList.Count; i++)
        {
            var position = poseList[i].pose.position;

            // Convert ROS coordinate system (right-handed, Z up) to Unity (left-handed, Y up)
            Vector3 unityPosition = new Vector3(-(float)position.y, -(float)position.x, 3);
            trajectoryLine.SetPosition(i, unityPosition);
        }
    }
}
