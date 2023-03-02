using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShootTester : MonoBehaviour
{
    public Rigidbody bullet;
    public float maximumShootSpeed = 10f;
    public float maximumDrawLength = 3f;
    Vector2 mouseDownPos, shootDir = Vector2.zero;
    void Update() 
    {
        if (Input.GetMouseButtonDown(0))
        {
            mouseDownPos = GetMousePos();
        }
        if (Input.GetMouseButton(0))
        {
            shootDir = Vector2.ClampMagnitude(GetMousePos() - mouseDownPos, maximumDrawLength);
        }
        if (Input.GetMouseButtonUp(0))
        {
            bullet.position = mouseDownPos;
            float nowShootSpeed = Mathf.Lerp(0f, maximumShootSpeed, Mathf.InverseLerp(0f, maximumDrawLength, shootDir.magnitude));
            bullet.velocity = shootDir.normalized * nowShootSpeed; 

            shootDir = Vector2.zero;
        }
    }
    Vector2 GetMousePos()
    {
        return Camera.main.ScreenToWorldPoint(Input.mousePosition);
    }
    void OnDrawGizmos() 
    {
        if (shootDir.magnitude > 0f)
        {
            Gizmos.color = Color.red;
            Gizmos.DrawLine(mouseDownPos, mouseDownPos + shootDir);
        }
    }
}
