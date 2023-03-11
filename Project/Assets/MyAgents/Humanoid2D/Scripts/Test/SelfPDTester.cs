using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using PhysicalCharacter2D;

public class SelfPDTester : MonoBehaviour
{
    public Transform refObj;
    public Rigidbody selfRb;
    public float posFrequency = 100f;
    public float posDamping = 1f;
    public float rotFrequency = 100f;
    public float rotDamping = 1f;
    public float addForceScale = 1000f;
    public float addTorqueScale = 1000f;

    float refLastPos;

    Vector2 mouseDownPos;

    // Start is called before the first frame update
    void Start()
    {
        refLastPos = refObj.position.y;
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        float deltaTime = Time.fixedDeltaTime;
        RbPDFollowRef(refObj, selfRb, deltaTime);
        MouseDragAddVerticalForce();
    }

    void RbPDFollowRef(Transform refObj, Rigidbody rb, float deltaTime)
    {
        // Vertical PD Follow
        float targetPos = refObj.position.y;
        float targetVel = (targetPos - refLastPos) / deltaTime;
        float targetForce = CommonFunctions.CalculatePDForce1DStable(targetPos, targetVel, rb.position.y, rb.velocity.y, posFrequency, posDamping, deltaTime);
        rb.AddForce(new Vector3(0f, targetForce));
        refLastPos = targetPos;

        // Rotation PD Follow
        Vector3 targetTorque = CommonFunctions.CalculatePDTorque(refObj.rotation, rb.transform.rotation, rb, rotFrequency, rotDamping, deltaTime);
        rb.AddTorque(targetTorque);
    }
    void MouseDragAddVerticalForce()
    {
        Vector2 mousePos = Camera.main.ScreenToWorldPoint(Input.mousePosition);
        if (Input.GetMouseButtonDown(0))
        {
            mouseDownPos = mousePos;
        }
        if (Input.GetMouseButton(0))
        {
            Vector2 dragOffset = mousePos - mouseDownPos;

            selfRb.AddForce(new Vector3(0f, dragOffset.y * addForceScale));

            selfRb.AddTorque(new Vector3(0f, 0f, dragOffset.x * addTorqueScale));
        }
    }
}
