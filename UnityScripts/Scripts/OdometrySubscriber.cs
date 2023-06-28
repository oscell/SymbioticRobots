using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;

public class OdometrySubscriber : MonoBehaviour
{
    public GameObject robot;
    void Start()
    {
        ROSConnection.GetOrCreateInstance().Subscribe<OdometryMsg>("/odometry/filtered", HandleOdometryMessage);
    }

    void HandleOdometryMessage(OdometryMsg odometryMessage)
    {
        var position = odometryMessage.pose.pose.position;
        var orientation = odometryMessage.pose.pose.orientation;

        Quaternion rosOrientation = new Quaternion((float)orientation.x, (float)orientation.y, (float)orientation.z, (float)orientation.w);
        Vector3 eulerRotation = rosOrientation.eulerAngles;


        robot.transform.position = new Vector3(-(float)position.y, -(float)position.x, 1);
	robot.transform.rotation = Quaternion.Euler(0,0,180-eulerRotation.z);

    }
}
