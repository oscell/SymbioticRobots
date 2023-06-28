using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosMessageTypes.Nav;

public class MapSubscriber : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "/map";

    public Material mapMaterial;
    public Renderer renderer;
    public GameObject mapObject;

void Start()
{
    ros = ROSConnection.GetOrCreateInstance();
    ros.Subscribe<OccupancyGridMsg>(topicName, HandleOccupancyGridMessage);
    ros.Subscribe<MapMetaDataMsg>("/map_metadata", HandleMapMetaDataMessage);
}


void HandleOccupancyGridMessage(OccupancyGridMsg message)
{
    // Create a new Texture2D object
    Texture2D texture = new Texture2D((int)message.info.width, (int)message.info.height);

    // Iterate over the OccupancyGrid data and set the pixels of the Texture2D
    for (int i = 0; i < message.data.Length; i++)
    {
        // Convert occupancy data to grayscale (assuming 0 = free, 100 = occupied, -1 = unknown)
        float grayscale = message.data[i] == -1 ? 0.5f : message.data[i] / 100.0f;
        Color color = new Color(grayscale, grayscale, grayscale, 0.5f);  // Added an alpha value of 0.5f

        // Calculate the x and y coordinates of the pixel
        int x = i % texture.width;
        int y = i / texture.width;

        // Set the pixel color in the texture
        texture.SetPixel(x, y, color);
    }

    // Apply the changes to the texture
    texture.Apply();

    // Create a new material using the texture
    mapMaterial.mainTexture = texture;

    // Ensure the material's shader supports transparency
    mapMaterial.shader = Shader.Find("Standard");
    mapMaterial.SetFloat("_Mode", 3);  // Set rendering mode to Transparent
    mapMaterial.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.One);
    mapMaterial.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
    mapMaterial.SetInt("_ZWrite", 0);
    mapMaterial.DisableKeyword("_ALPHATEST_ON");
    mapMaterial.DisableKeyword("_ALPHABLEND_ON");
    mapMaterial.EnableKeyword("_ALPHAPREMULTIPLY_ON");
    mapMaterial.renderQueue = 3000;

    // Apply the material to the renderer
    renderer.material = mapMaterial;
}


void HandleMapMetaDataMessage(MapMetaDataMsg message)
{
    uint width = message.width;
    uint height = message.height;

    float x_scale = 2f;
    float y_scale = 2f;
    float x_off = 0;
    float y_off = 0;

    if (width == 1504)
    {
        x_scale = 3f;
        y_off = 5f;
    }

    if (height == 1504)
    {
        y_scale = 3f;
        x_off = 5f;
    }

    // Assuming mapObject is the GameObject that should be scaled
    mapObject.transform.localScale = new Vector3(x_scale, mapObject.transform.localScale.y, y_scale);
    mapObject.transform.position = new Vector3(x_off-0.1f, -y_off+0.1f, 0);
}

}
