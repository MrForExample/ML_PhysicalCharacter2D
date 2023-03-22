using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;
using Unity.MLAgents;
using Unity.MLAgentsExamples;

namespace PhysicalCharacter2D
{
    /// <summary>
    /// Used to store relevant information for acting and learning for each body part in agent.
    /// </summary>
    [System.Serializable]
    public class BodyPartPDFeedback
    {
        [Header("Body Part Info")] [Space(10)] public ConfigurableJoint joint;
        public Rigidbody rb;
        [HideInInspector] public Vector3 startingPos;
        [HideInInspector] public Quaternion startingRot;

        [Header("Ground & Target Contact")]
        [Space(10)]
        public GroundContact groundContact;

        public TargetContact targetContact;

        [FormerlySerializedAs("thisJDController")]
        [HideInInspector] public JointDriveControllerPDFeedback thisJdController;

        [Header("Current Joint Settings")]
        [HideInInspector] public float currentStrength;

        Quaternion joint_init_axis_rotation;
        Quaternion world_to_joint_space;
        Quaternion joint_to_world_space;
        Quaternion last_joint_target_local_rotation = Quaternion.identity;
        Quaternion joint_add_target_rotation = Quaternion.identity;
        float joint_add_range;

        [Header("Other Debug Info")]
        [Space(10)]
        public Vector3 currentJointForce;

        public float currentJointForceSqrMag;
        public Vector3 currentJointTorque;
        public float currentJointTorqueSqrMag;
        public AnimationCurve jointForceCurve = new AnimationCurve();
        public AnimationCurve jointTorqueCurve = new AnimationCurve();

        public BodyPartPDFeedback(Transform t)
        {
            rb = t.GetComponent<Rigidbody>();
            joint = t.GetComponent<ConfigurableJoint>();
            startingPos = t.position;
            startingRot = t.rotation;

            if (joint != null)
            {
                joint_init_axis_rotation = joint.transform.localRotation;
                // Calculate the rotation expressed by the joint's axis and secondary axis and get its space transform matrix
                var right = joint.axis;
                var up = joint.secondaryAxis;
                var forward = Vector3.Cross(right, up).normalized;
                world_to_joint_space = Quaternion.LookRotation(forward, up);
                joint_to_world_space = Quaternion.Inverse(world_to_joint_space);

                last_joint_target_local_rotation = joint.targetRotation;

                joint_add_range = (joint.highAngularXLimit.limit - joint.lowAngularXLimit.limit) / 2f;
            }
        }

        /// <summary>
        /// Reset body part to initial configuration.
        /// </summary>
        public void Reset(BodyPartPDFeedback bp)
        {
            bp.rb.transform.position = bp.startingPos;
            bp.rb.transform.rotation = bp.startingRot;
            bp.rb.velocity = Vector3.zero;
            bp.rb.angularVelocity = Vector3.zero;
            if (bp.groundContact)
            {
                bp.groundContact.touchingGround = false;
            }

            if (bp.targetContact)
            {
                bp.targetContact.touchingTarget = false;
            }
        }

        public void SetJointPDTarget(Quaternion target_local_rotation, float deltaTime)
        {
            // Matrix change of basis, Transform difference of local rotation back into joint space
            Quaternion joint_target_local_rotation = joint_to_world_space * (Quaternion.Inverse(target_local_rotation) * joint_init_axis_rotation) * world_to_joint_space;
            // Set target rotation to our newly calculated rotation
            joint.targetRotation = joint_target_local_rotation * joint_add_target_rotation;

            // Set target angular velocity according to last and current real joint target rotation
            Vector3 now_target_angular_velocity = CommonFunctions.CalculateAngularVelocity(last_joint_target_local_rotation, joint_target_local_rotation, deltaTime);
            joint.targetAngularVelocity = now_target_angular_velocity;

            last_joint_target_local_rotation = joint_target_local_rotation;
        }

        /// <summary>
        /// Apply torque according to defined goal `x` angle and force `strength`.
        /// </summary>
        public void AddJointTargetRotation(float x)
        {
            x = (x + 1f) * 0.5f;
            
            var xRot = Mathf.Lerp(-joint_add_range, joint_add_range, x);

            joint_add_target_rotation = Quaternion.Euler(xRot, 0f, 0f);
        }

        public void SetJointStrength(float strength)
        {
            var rawVal = (strength + 1f) * 0.5f * thisJdController.maxJointForceLimit;
            var jd = new JointDrive
            {
                positionSpring = thisJdController.maxJointSpring,
                positionDamper = thisJdController.jointDampen,
                maximumForce = rawVal
            };
            joint.slerpDrive = jd;
            currentStrength = jd.maximumForce;
        }
    }

    public class JointDriveControllerPDFeedback : MonoBehaviour
    {
        [Header("Joint Drive Settings")]
        [Space(10)]
        public float maxJointSpring;

        public float jointDampen;
        public float maxJointForceLimit;
        float m_FacingDot;

        [HideInInspector] public Dictionary<Transform, BodyPartPDFeedback> bodyPartsDict = new Dictionary<Transform, BodyPartPDFeedback>();

        [HideInInspector] public List<BodyPartPDFeedback> bodyPartsList = new List<BodyPartPDFeedback>();
        const float k_MaxAngularVelocity = 50.0f;

        /// <summary>
        /// Create BodyPartPDFeedback object and add it to dictionary.
        /// </summary>
        public void SetupBodyPart(Transform t)
        {
            var bp = new BodyPartPDFeedback(t);
            bp.rb.maxAngularVelocity = k_MaxAngularVelocity;

            // Add & setup the ground contact script
            bp.groundContact = t.GetComponent<GroundContact>();
            if (!bp.groundContact)
            {
                bp.groundContact = t.gameObject.AddComponent<GroundContact>();
                bp.groundContact.agent = gameObject.GetComponent<Agent>();
            }
            else
            {
                bp.groundContact.agent = gameObject.GetComponent<Agent>();
            }

            if (bp.joint)
            {
                var jd = new JointDrive
                {
                    positionSpring = maxJointSpring,
                    positionDamper = jointDampen,
                    maximumForce = maxJointForceLimit
                };
                bp.joint.slerpDrive = jd;
            }

            bp.thisJdController = this;
            bodyPartsDict.Add(t, bp);
            bodyPartsList.Add(bp);
        }

        public void GetCurrentJointForces()
        {
            foreach (var BodyPartPDFeedback in bodyPartsDict.Values)
            {
                if (BodyPartPDFeedback.joint)
                {
                    BodyPartPDFeedback.currentJointForce = BodyPartPDFeedback.joint.currentForce;
                    BodyPartPDFeedback.currentJointForceSqrMag = BodyPartPDFeedback.joint.currentForce.magnitude;
                    BodyPartPDFeedback.currentJointTorque = BodyPartPDFeedback.joint.currentTorque;
                    BodyPartPDFeedback.currentJointTorqueSqrMag = BodyPartPDFeedback.joint.currentTorque.magnitude;
                    if (Application.isEditor)
                    {
                        if (BodyPartPDFeedback.jointForceCurve.length > 1000)
                        {
                            BodyPartPDFeedback.jointForceCurve = new AnimationCurve();
                        }

                        if (BodyPartPDFeedback.jointTorqueCurve.length > 1000)
                        {
                            BodyPartPDFeedback.jointTorqueCurve = new AnimationCurve();
                        }

                        BodyPartPDFeedback.jointForceCurve.AddKey(Time.time, BodyPartPDFeedback.currentJointForceSqrMag);
                        BodyPartPDFeedback.jointTorqueCurve.AddKey(Time.time, BodyPartPDFeedback.currentJointTorqueSqrMag);
                    }
                }
            }
        }
    }
}
