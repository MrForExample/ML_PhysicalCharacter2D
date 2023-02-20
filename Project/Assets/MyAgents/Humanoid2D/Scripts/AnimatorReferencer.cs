using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;

public class AnimatorReferencer : MonoBehaviour
{
    Animator animator;
    public DecisionRequester decisionRequester;
    public float maxAnimationLength = 1f;

    [Header("Body Parts")]
    public Transform hips;
    public Transform spine;
    public Transform head;
    public Transform thighL;
    public Transform shinL;
    public Transform footL;
    public Transform thighR;
    public Transform shinR;
    public Transform footR;
    public Transform armL;
    public Transform forearmL;
    public Transform handL;
    public Transform armR;
    public Transform forearmR;
    public Transform handR;

    [HideInInspector] public Dictionary<string, BodyReferencer> bodyReferencersDict = new Dictionary<string, BodyReferencer>();

    public class BodyReferencer
    {
        public Transform bodyTransform;
        public Quaternion bodyLastLocalQuaternion;
        public BodyReferencer(Transform newBodyTransform)
        {
            bodyTransform = newBodyTransform;
        }
        public void recordLastLocalQuaternion()
        {
            bodyLastLocalQuaternion = bodyTransform.localRotation;
        }
    }
    // Start is called before the first frame update
    void Start()
    {
        animator = GetComponent<Animator>();

        bodyReferencersDict.Add(hips.name, new BodyReferencer(hips));
        bodyReferencersDict.Add(spine.name, new BodyReferencer(spine));
        bodyReferencersDict.Add(head.name, new BodyReferencer(head));
        bodyReferencersDict.Add(thighL.name, new BodyReferencer(thighL));
        bodyReferencersDict.Add(shinL.name, new BodyReferencer(shinL));
        bodyReferencersDict.Add(footL.name, new BodyReferencer(footL));
        bodyReferencersDict.Add(thighR.name, new BodyReferencer(thighR));
        bodyReferencersDict.Add(shinR.name, new BodyReferencer(shinR));
        bodyReferencersDict.Add(footR.name, new BodyReferencer(footR));
        bodyReferencersDict.Add(armL.name, new BodyReferencer(armL));
        bodyReferencersDict.Add(forearmL.name, new BodyReferencer(forearmL));
        bodyReferencersDict.Add(handL.name, new BodyReferencer(handL));
        bodyReferencersDict.Add(armR.name, new BodyReferencer(armR));
        bodyReferencersDict.Add(forearmR.name, new BodyReferencer(forearmR));
        bodyReferencersDict.Add(handR.name, new BodyReferencer(handR));

        ReferenceStateInitialization();
    }

    void ReferenceStateInitialization()
    {
        animator.Update(Random.Range(0f, maxAnimationLength));
        RecordAllBodiesLastLocalQuaternion();
        animator.Update(Time.fixedDeltaTime * decisionRequester.DecisionPeriod);
    }
    public void RecordAllBodiesLastLocalQuaternion()
    {
        foreach (var bodyPart in bodyReferencersDict.Values)
            bodyPart.recordLastLocalQuaternion();
    }
    public Quaternion CalculateQuaternionDifference(Quaternion previousRotation, Quaternion currentRotation)
    {
        return currentRotation * Quaternion.Inverse(previousRotation);
    }
    public Vector3 CalculateAngularVelocity(Quaternion previousRotation, Quaternion currentRotation, float deltaTime)
    {
        Quaternion deltaRotation = CalculateQuaternionDifference(previousRotation, currentRotation);
        
        float angle  = 0.0f;
        Vector3 axis = Vector3.zero;
        
        deltaRotation.ToAngleAxis(out angle, out axis);
        
        angle *= Mathf.Deg2Rad;
        
        return axis * angle * (1.0f / deltaTime);
    }
}
