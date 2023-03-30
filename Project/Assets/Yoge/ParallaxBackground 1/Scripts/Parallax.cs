 using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Parallax : MonoBehaviour
{
    [SerializeField] private Vector2 parallaxEffect; //For horizontal and vertical parallax;
    private Transform cameraTransform;
    private Vector3 lastCameraPosition;
    private float textureSizeX;
   // private float textureSizeY; // infinite tiling of textures for vertical parallax, not used here, just for the case;


    void Start()
    {
        cameraTransform = Camera.main.transform;
        lastCameraPosition = cameraTransform.position;
        Sprite sprite = GetComponent<SpriteRenderer>().sprite;
        Texture2D texture = sprite.texture;
        textureSizeX = texture.width / sprite.pixelsPerUnit;
        // textureSizeY = texture.height / sprite.pixelsPerUnit; // infinite tiling of textures for vertical parallax, not used here, just for the case;
    }

    // Do not use FixedUpdate, cause it will lag badly;
    void LateUpdate() 
    {
        Vector3 movementDelta = cameraTransform.position - lastCameraPosition;
        transform.position += new Vector3 (movementDelta.x * parallaxEffect.x, movementDelta.y * parallaxEffect.y);
        lastCameraPosition = cameraTransform.position;

        if (Mathf.Abs(cameraTransform.position.x - transform.position.x) >= textureSizeX) 
        {
            float offsetPostitionX = (cameraTransform.position.x - transform.position.x) % textureSizeX;
            transform.position = new Vector3(cameraTransform.position.x + offsetPostitionX, transform.position.y);
        }

        /*// infinite tiling of textures for vertical parallax, not used here, just for the case;
        if (Mathf.Abs(cameraTransform.position.y - transform.position.y) >= textureSizeY)
        {
            float offsetPostitionY = (cameraTransform.position.y - transform.position.y) % textureSizeY;
            transform.position = new Vector3(transform.position.x,  cameraTransform.position.y + offsetPostitionY);
        }*/
    }
}
