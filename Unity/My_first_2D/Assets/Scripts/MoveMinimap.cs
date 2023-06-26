using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MoveMinimap : MonoBehaviour
{
    public Vector3 minimapPosition;
    public Vector3 minimapScale;

    // Reference to all game objects in the minimap
    public List<GameObject> minimapObjects;

    void Start()
    {
        // Initialize minimap position and scale
        UpdateMinimap(minimapPosition, minimapScale);
    }

    public void UpdateMinimap(Vector3 newPosition, Vector3 newScale)
    {
        // Set the new position and scale for each game object in the minimap
        foreach (GameObject obj in minimapObjects)
        {
            obj.transform.localPosition = newPosition;
            obj.transform.localScale = newScale;
        }
    }
}

