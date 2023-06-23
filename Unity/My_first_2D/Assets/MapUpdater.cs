using System.IO;
using UnityEngine;

public class MapUpdater : MonoBehaviour
{
    public string filePath = "C:/Path/To/Your/Image.png";
    public SpriteRenderer mapRenderer;
    private System.DateTime lastModifiedTime;

    void Start()
    {
        InvokeRepeating(nameof(UpdateMap), 0f, 1f); // Call UpdateMap immediately and then every 1 second
    }

    void UpdateMap()
    {
        if (File.Exists(filePath))
        {
            System.DateTime newModifiedTime = File.GetLastWriteTime(filePath);

            if (newModifiedTime != lastModifiedTime)
            {
                byte[] fileData = File.ReadAllBytes(filePath);
                Texture2D tex = new Texture2D(2, 2);
                tex.LoadImage(fileData); //..this will auto-resize the texture dimensions.

                // Convert the Texture2D to a Sprite
                Sprite sprite = Sprite.Create(tex, new Rect(0, 0, tex.width, tex.height), new Vector2(0.5f, 0.5f));
                mapRenderer.sprite = sprite;

                lastModifiedTime = newModifiedTime; // Update last modified time
            }
        }
        else
        {
            Debug.LogError("Cannot find file at: " + filePath);
        }
    }
}
