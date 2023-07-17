using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SpriteChanger : MonoBehaviour
{
    public Sprite sprites; // Array to store your sprites
    public SpriteRenderer spriteRenderer; // Sprite Renderer component
    public float changeInterval = 3.0f; // Interval time in seconds

    void Start()
    {

            StartCoroutine(ChangeSprite());
    }

    IEnumerator ChangeSprite()
    {
        while (true)
        {
            // Set the sprite
            spriteRenderer.sprite = sprites;

            // Wait for the specified interval
            yield return new WaitForSeconds(changeInterval);
        }
    }
}

