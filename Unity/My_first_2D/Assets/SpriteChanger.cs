using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SpriteChanger : MonoBehaviour
{
    public Sprite[] sprites; // Array to store your sprites
    public SpriteRenderer spriteRenderer; // Sprite Renderer component
    public float changeInterval = 3.0f; // Interval time in seconds

    private int currentSpriteIndex = 0; // Keep track of the current sprite

    void Start()
    {
        if (sprites.Length > 0 && spriteRenderer != null)
        {
            // Start the ChangeSprite coroutine
            StartCoroutine(ChangeSprite());
        }
    }

    IEnumerator ChangeSprite()
    {
        while (true)
        {
            // Set the sprite
            spriteRenderer.sprite = sprites[currentSpriteIndex];

            // Increment the index
            currentSpriteIndex++;

            // If we've gone past the end of the array, loop back to the start
            if (currentSpriteIndex >= sprites.Length)
            {
                currentSpriteIndex = 0;
            }

            // Wait for the specified interval
            yield return new WaitForSeconds(changeInterval);
        }
    }
}

