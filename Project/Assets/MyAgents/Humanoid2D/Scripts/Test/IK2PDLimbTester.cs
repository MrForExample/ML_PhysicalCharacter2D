using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgentsExamples;
using PhysicalCharacter2D;

public class IK2PDLimbTester : MonoBehaviour
{
    [Header("Body Parts")]
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

    [Header("Limb Poles")]
    public Transform poleLegL;
    public Transform poleLegR;
    public Transform poleArmL;
    public Transform poleArmR;

    [HideInInspector]
    public JointDriveController m_JdController;
    Transform[,] limbArrays;
    Vector3[] limbEndTargets = new Vector3[4];
    List<PhysicalLimb> allPhysicalLimbs = new List<PhysicalLimb>();
    int currentEndTargetIndex = -1;

    [Header("Debug")]
    public float controlRadius = 1f;
    public bool isDebug = true;
    [Range(-1f, 1f)]
    public float dir_a = 0f;
    [Range(-1f, 1f)]
    public float len_a = 0f;

    void Start()
    {
        //Setup each body part
        m_JdController = GetComponent<JointDriveController>();
        m_JdController.SetupBodyPart(thighL);
        m_JdController.SetupBodyPart(shinL);
        m_JdController.SetupBodyPart(footL);
        m_JdController.SetupBodyPart(thighR);
        m_JdController.SetupBodyPart(shinR);
        m_JdController.SetupBodyPart(footR);
        m_JdController.SetupBodyPart(armL);
        m_JdController.SetupBodyPart(forearmL);
        m_JdController.SetupBodyPart(handL);
        m_JdController.SetupBodyPart(armR);
        m_JdController.SetupBodyPart(forearmR);
        m_JdController.SetupBodyPart(handR);

        limbArrays = new Transform[4,3]{
            {footL, shinL, thighL},
            {footR, shinR, thighR},
            {handL, forearmL, armL},
            {handR, forearmR, armR}
        };

        var poleArrays = new Transform[4]{poleLegL, poleLegR, poleArmL, poleArmR};

        PhysicalBody[] physicalBodies = new PhysicalBody[3];
        for (int iLimb = 0; iLimb < 4; iLimb++)
        {
            ConfigurableJoint lastJoint = null;
            for (int iJoint = 0; iJoint < 3; iJoint++)
            {
                var body = m_JdController.bodyPartsDict[limbArrays[iLimb, iJoint]];
                physicalBodies[iJoint] = new PhysicalBody(body.rb, body.joint, lastJoint);
                lastJoint = body.joint;
            }
            allPhysicalLimbs.Add(new PhysicalLimb(physicalBodies, poleArrays[iLimb], isDebug));
        }
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        //ControlTargetByMouse();
        //Target2IK2PD();
        Params2IK2PD();
    }

    void ControlTargetByMouse()
    {
        Vector2 mousePos = Camera.main.ScreenToWorldPoint(Input.mousePosition);
        if (!Input.GetMouseButton(0) || currentEndTargetIndex < 0)
        {
            float minRadius = controlRadius;
            currentEndTargetIndex = -1;
            for (int i = 0; i < allPhysicalLimbs.Count; i++)
            {
                Vector2 endPos = CommonFunctions.GetNowJointPosition(allPhysicalLimbs[i].end_body.joint);
                float nowRadius = (endPos - mousePos).magnitude;
                if (nowRadius < minRadius)
                {
                    minRadius = nowRadius;
                    currentEndTargetIndex = i;
                }

                limbEndTargets[i] = CommonFunctions.GetNowJointPosition(allPhysicalLimbs[i].end_body.joint);
            }
        }
        else
        {
            limbEndTargets[currentEndTargetIndex] = mousePos;
            allPhysicalLimbs[currentEndTargetIndex].LimbIKMoveToTargrtPoint(limbEndTargets[currentEndTargetIndex], Time.fixedDeltaTime);
        }
    }

    void Target2IK2PD()
    {
        for (int i = 0; i < allPhysicalLimbs.Count; i++)
        {
            allPhysicalLimbs[i].LimbIKMoveToTargrtPoint(limbEndTargets[i], Time.fixedDeltaTime);
        }
    }

    void Params2IK2PD()
    {
        for (int i = 0; i < allPhysicalLimbs.Count; i++)
        {
            allPhysicalLimbs[i].LimbIKMoveToTargrtPoint(allPhysicalLimbs[i].CalculateTargetEndPoint(dir_a, len_a), Time.fixedDeltaTime);
        }
    }

    void OnDrawGizmos()
    {
        if (isDebug)
        {
            foreach (var limb in allPhysicalLimbs)
                limb.DebugDraw();

            /*
            if (currentEndTargetIndex >= 0)
            {
                if (Input.GetMouseButton(0))
                    allPhysicalLimbs[currentEndTargetIndex].DebugDraw();

                Gizmos.color = Color.red;
                Gizmos.DrawSphere(limbEndTargets[currentEndTargetIndex], 0.1f);
            }
            */
        }
    }
}
