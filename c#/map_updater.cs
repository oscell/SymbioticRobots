using System.IO;
using UnityEngine;

public class MapUpdater : MonoBehaviour
{
    public string livemap_path = "/home/oscar/Desktop/SymbioticRobots/Unity/My_first_2D/Assets/Map/mymap.png";
    public string fullmap_path = "/home/oscar/Desktop/SymbioticRobots/Unity/My_first_2D/Assets/Map/full_map.png";

    public SpriteRenderer mapRenderer;
    public bool updateMode = true;  // Add this line

    private System.DateTime lastModifiedTime;

    void Start()
    {
        LoadImageOnce();  // Load the image at least once

        if (updateMode)  // If updateMode is true, start the UpdateMap repeating function
        {
            InvokeRepeating(nameof(UpdateMap), 0f, 1f); // Call UpdateMap immediately and then every 1 second
        }
    }

    void LoadImageOnce()
    {
        if (File.Exists(fullmap_path))
        {
            byte[] fileData = File.ReadAllBytes(fullmap_path);
            Texture2D tex = new Texture2D(2, 2);
            tex.LoadImage(fileData); //..this will auto-resize the texture dimensions.

            // Convert the Texture2D to a Sprite
            Sprite sprite = Sprite.Create(tex, new Rect(0, 0, tex.width, tex.height), new Vector2(0.5f, 0.5f));
            mapRenderer.sprite = sprite;

            lastModifiedTime = File.GetLastWriteTime(fullmap_path);  // Update last modified time
        }
        else
        {
            Debug.LogError("Cannot find file at: " + fullmap_path);
        }
    }

    void UpdateMap()
    {
        if (File.Exists(livemap_path))
        {
            System.DateTime newModifiedTime = File.GetLastWriteTime(livemap_path);

            if (newModifiedTime != lastModifiedTime)
            {
                LoadImageOnce();  // Call LoadImageOnce function to load the updated image

                lastModifiedTime = newModifiedTime; // Update last modified time
            }
        }
        else
        {
            Debug.LogError("Cannot find file at: " + livemap_path);
        }
    }
}
