using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class SpeedTester : MonoBehaviour
{
    public float moveSpeed = 4f;

    // Update is called once per frame
    void FixedUpdate()
    {
        transform.position += moveSpeed * Vector3.right * Time.fixedDeltaTime;
    }
}
