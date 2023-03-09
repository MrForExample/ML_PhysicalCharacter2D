using System;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using BodyPart = Unity.MLAgentsExamples.BodyPart;
using Random = UnityEngine.Random;
using EffectorReferencer = PhysicalCharacter2D.AnimatorReferencerIK2PD.EffectorReferencer;

namespace PhysicalCharacter2D
{
    public class MimicAgent2DIK2PD : Agent
    {
        [Header("Reference motion")]
        public AnimatorReferencerIK2PD animatorReferencer;

        [Header("Walk Speed")]
        [Range(0.1f, 10)]
        //The walking speed to try and achieve
        public float targetWalkingSpeed = 4;

        //The direction an agent will walk during training.
        private Vector3 m_WorldDirToWalk = Vector3.right;

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

        [Header("Limb Poles")]
        public Transform poleLegL;
        public Transform poleLegR;
        public Transform poleArmL;
        public Transform poleArmR;

        [HideInInspector]
        public List<PhysicalLimb> allPhysicalLimbs = new List<PhysicalLimb>();

        [Header("Debug")]
        public bool isDebug = false;

        [HideInInspector]
        public GroundContact[] footsLand;

        //This will be used as a stabilized model space reference point for observations
        //Because ragdolls can move erratically during training, using a stabilized reference transform improves learning
        OrientationCubeController m_OrientationCube;
        [HideInInspector]
        public JointDriveController m_JdController;
        EnvironmentParameters m_ResetParams;

        float totalMass;

        Transform[,] limbArrays;

        public override void Initialize()
        {
            m_OrientationCube = GetComponentInChildren<OrientationCubeController>();

            //Setup each body part
            m_JdController = GetComponent<JointDriveController>();
            m_JdController.SetupBodyPart(hips);
            m_JdController.SetupBodyPart(spine);
            m_JdController.SetupBodyPart(head);
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

            footsLand = new GroundContact[2];
            footsLand[0] = footL.GetComponentInChildren<GroundContact>(true);
            footsLand[1] = footR.GetComponentInChildren<GroundContact>(true);

            m_ResetParams = Academy.Instance.EnvironmentParameters;

            totalMass = 0f;
            foreach (var bp in m_JdController.bodyPartsDict.Values)
                totalMass += bp.rb.mass;
        }

        /// <summary>
        /// Loop over body parts and reset them to initial conditions.
        /// </summary>
        public override void OnEpisodeBegin()
        {
            //Reset all of the body parts
            foreach (var bp in m_JdController.bodyPartsDict.Values)
            {
                bp.Reset(bp);
            }

            UpdateOrientationObjects();

            ReferenceStateInitialization();
        }
        void ReferenceStateInitialization()
        {
            animatorReferencer.ReferenceStateInitializationForRef();
            float deltaTime = Time.fixedDeltaTime;
            animatorReferencer.ForwardAnimatior(deltaTime);

            foreach (var bp in m_JdController.bodyPartsDict.Values)
            {
                var refBp = animatorReferencer.bodyReferencersDict[bp.rb.name];
                bp.rb.rotation = refBp.bodyLastQuaternion;
                bp.rb.transform.localRotation = refBp.bodyLastLocalQuaternion;
                bp.rb.angularVelocity = CommonFunctions.CalculateAngularVelocity(refBp.bodyLastQuaternion, refBp.bodyTransform.rotation, deltaTime);
            }
        }

        public void CollectObservationLimb(PhysicalLimb limb, EffectorReferencer effectorRef, Vector2 hipPos, VectorSensor sensor)
        {
            // Root&End joints offset for both reference and simulated agent
            sensor.AddObservation(effectorRef.rootEffectorOffset);
            sensor.AddObservation(effectorRef.endEffectorOffset);

            Vector2 root_joint_position = CommonFunctions.GetNowJointPosition(limb.upper_body.joint);
            Vector2 end_joint_position = CommonFunctions.GetNowJointPosition(limb.end_body.joint);
            sensor.AddObservation(root_joint_position - hipPos);
            sensor.AddObservation(end_joint_position - root_joint_position);
            // state size: 8
        }
        /// <summary>
        /// Add relevant information on each body part to observations.
        /// </summary>
        public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
        {
            // Reference motion
            var refBp = animatorReferencer.bodyReferencersDict[bp.rb.name];
            sensor.AddObservation(CommonFunctions.CalculateAngularVelocity(refBp.bodyLastQuaternion, refBp.bodyTransform.rotation, Time.fixedDeltaTime));
            sensor.AddObservation(CommonFunctions.CalculateLocalQuaternion(refBp.bodyLastQuaternion, m_OrientationCube.transform.rotation));

            //Ground Check
            sensor.AddObservation(bp.groundContact.touchingGround); // Is this bp touching the ground

            //Get velocities
            Vector2 rbVel = bp.rb.velocity;
            sensor.AddObservation(rbVel);
            sensor.AddObservation(bp.rb.angularVelocity.z);

            sensor.AddObservation(bp.rb.transform.localRotation);
            // state size: 15

            if (bp.rb.transform != hips)
            {
                //Get position relative to hips for both reference and simulated agent 
                Vector2 refPosDiff = refBp.bodyLastPosition - animatorReferencer.hips.bodyLastPosition;
                sensor.AddObservation(refPosDiff);
                
                Vector2 posDiff = bp.rb.position - hips.position;
                sensor.AddObservation(posDiff);
                sensor.AddObservation(bp.currentStrength / m_JdController.maxJointForceLimit);
                // state size: 20
            }
            else
            {
                // Add root height for both reference and simulated agent 
                sensor.AddObservation(refBp.bodyLastPosition.y);
                sensor.AddObservation(hips.position.y);
                // state size: 17
            }
        }

        /// <summary>
        /// Loop over body parts to add them to observation.
        /// </summary>
        public override void CollectObservations(VectorSensor sensor)
        {
            var cubeForward = m_OrientationCube.transform.forward;

            //velocity we want to match
            Vector2 velGoal = cubeForward * targetWalkingSpeed;
            //ragdoll's avg vel
            Vector2 avgVel = GetAvgVelocity();

            //current ragdoll velocity. normalized
            sensor.AddObservation(Vector2.Distance(velGoal, avgVel));
            //avg body vel
            Vector2 avgVelLocal = avgVel;
            sensor.AddObservation(avgVelLocal);
            //vel goal
            Vector2 velGoalLocal = velGoal;
            sensor.AddObservation(velGoalLocal);

            //rotation deltas
            sensor.AddObservation(Quaternion.FromToRotation(-hips.right, cubeForward));
            sensor.AddObservation(Quaternion.FromToRotation(-head.up, cubeForward));
            // state size: 13 

            foreach (var bodyPart in m_JdController.bodyPartsList)
                CollectObservationBodyPart(bodyPart, sensor);
            // state size: 13 + 20 * 14 + 17 = 270

            for (int i = 0; i < allPhysicalLimbs.Count; i++)
                CollectObservationLimb(allPhysicalLimbs[i], animatorReferencer.effectorRefs[i], hips.position, sensor);
            // state size: 270 + 8 * 4 = 302
        }

        public override void OnActionReceived(ActionBuffers actionBuffers)
        {
            var bpDict = m_JdController.bodyPartsDict;
            var continuousActions = actionBuffers.ContinuousActions;
            var i = -1;

            bpDict[spine].SetJointTargetRotation(continuousActions[++i], 0f, 0f);
            bpDict[head].SetJointTargetRotation(continuousActions[++i], 0f, 0);
            bpDict[footL].SetJointTargetRotation(continuousActions[++i], 0f, 0f);
            bpDict[footR].SetJointTargetRotation(continuousActions[++i], 0f, 0f);
            bpDict[handL].SetJointTargetRotation(continuousActions[++i], 0f, 0f);
            bpDict[handR].SetJointTargetRotation(continuousActions[++i], 0f, 0f);

            foreach (var limb in allPhysicalLimbs)
            {
                limb.LimbIKMoveToTargrtPoint(limb.CalculateTargetEndPoint(continuousActions[++i], continuousActions[++i]), Time.fixedDeltaTime);
            }
            
            bpDict[spine].SetJointStrength(continuousActions[++i]);
            bpDict[head].SetJointStrength(continuousActions[++i]);

            for (int iLimb = 0; iLimb < 4; iLimb++)
            {
                float limbStrength = continuousActions[++i];
                for (int iJoint = 0; iJoint < 3; iJoint++)
                    bpDict[limbArrays[iLimb, iJoint]].SetJointStrength(limbStrength);
            }
            // action size: 20
        }

        void UpdateOrientationObjects()
        {
            m_OrientationCube.transform.position = hips.position;
        }

        float wi = 0.7f, wg = 0.3f;
        void FixedUpdate()
        {
            UpdateOrientationObjects();

            float ri = CalculateImitationReward();
            float rg = CalculateTaskReward();
            AddReward(wi * ri + wg * rg);

            animatorReferencer.RecordAllBodiesLastTarget();
        }
        float wp = 0.35f, wv = 0.1f, we = 0.35f, wc = 0.1f, wl = 0.1f;
        float wps = -2f, wvs = -4f, wes = -4f, wcs = -10f, wls = -10f;
        float CalculateImitationReward()
        {
            float rotDiffSum = 0f, angularVelDiffSum = 0f, endDiffSum = 0f, cOMDiffSum = 0f, footsLandSum = 0f;

            Vector2 refHipPos = m_JdController.bodyPartsDict[hips].rb.position;
            Vector2 hipPos = hips.position;
            for (int i = 0; i < allPhysicalLimbs.Count; i++)
            {
                var limb = allPhysicalLimbs[i];
                Vector2 root_joint_position = CommonFunctions.GetNowJointPosition(limb.upper_body.joint);
                Vector2 end_joint_position = CommonFunctions.GetNowJointPosition(limb.end_body.joint);
                endDiffSum += Vector2.SqrMagnitude((root_joint_position - hipPos) - animatorReferencer.effectorRefs[i].rootEffectorOffset);
                endDiffSum += Vector2.SqrMagnitude((end_joint_position - root_joint_position) - animatorReferencer.effectorRefs[i].endEffectorOffset);
            }

            for (int i = 0; i < footsLand.Length; i++)
            {
                footsLandSum += footsLand[i].touchingGround == animatorReferencer.footsLand[i].touchingGround? 0f : 1f;
                //Debug.Log("foot " + i + ": " + animatorReferencer.footsLand[i].touchingGround);
            }
            footsLandSum /= footsLand.Length;

            float angle  = 0.0f;
            Vector3 axis = Vector3.zero;
            Vector2 worldCOM = new Vector3(0f, m_JdController.bodyPartsDict[hips].rb.worldCenterOfMass.y), 
                    targetWorldCOM = new Vector2(0f, animatorReferencer.bodyReferencersDict[hips.name].bodyLastCOM.y);
            foreach (var bp in m_JdController.bodyPartsDict.Values)
            {
                var refBp = animatorReferencer.bodyReferencersDict[bp.rb.name];

                Quaternion deltaRotation = CommonFunctions.CalculateLocalQuaternion(bp.rb.transform.localRotation, refBp.bodyLastLocalQuaternion);
                deltaRotation.ToAngleAxis(out angle, out axis);
                rotDiffSum += Mathf.Pow(angle * Mathf.Deg2Rad, 2f);

                Vector3 targetAngularVel = CommonFunctions.CalculateAngularVelocity(refBp.bodyLastQuaternion, refBp.bodyTransform.rotation, Time.fixedDeltaTime);
                angularVelDiffSum += Mathf.Pow(targetAngularVel.z - bp.rb.angularVelocity.z, 2f);

                Vector2 cOM= (bp.rb.worldCenterOfMass - m_JdController.bodyPartsDict[hips].rb.worldCenterOfMass) * bp.rb.mass;
                worldCOM += cOM;
                targetWorldCOM += (refBp.bodyLastCOM - animatorReferencer.hips.bodyLastCOM) * bp.rb.mass;
            }
            cOMDiffSum = Vector2.SqrMagnitude((targetWorldCOM - worldCOM) / totalMass);
            
            float rp = wp * Mathf.Exp(wps * rotDiffSum);
            float rv = wv * Mathf.Exp(wvs * angularVelDiffSum);
            float re = we * Mathf.Exp(wes * endDiffSum);
            float rc = wc * Mathf.Exp(wcs * cOMDiffSum);
            float rl = wl * Mathf.Exp(wls * footsLandSum);

            float ri = rp + rv + re + rc + rl;
            //Check for NaNs
            if (float.IsNaN(ri))
            {
                throw new ArgumentException(
                    "NaN in imitation reward.\n" +
                    $" rp: {rp};" + $" rv: {rv};" + $" re: {re};" + $" rc: {rc};" 
                );
            }

            return ri;
        }

        float CalculateTaskReward()
        {
            var cubeForward = m_OrientationCube.transform.forward;

            // Set reward for this step according to mixture of the following elements.
            // a. Match target speed
            //This reward will approach 1 if it matches perfectly and approach zero as it deviates
            float rg = GetMatchingVelocityReward(cubeForward * targetWalkingSpeed, GetAvgVelocity());

            //Check for NaNs
            if (float.IsNaN(rg))
            {
                throw new ArgumentException(
                    "NaN in task reward.\n" +
                    $" cubeForward: {cubeForward}\n" + 
                    $" head.forward: {head.forward}\n" + 
                    $" targetWalkingSpeed: {targetWalkingSpeed}\n" +
                    $" hips.velocity: {m_JdController.bodyPartsDict[hips].rb.velocity}"
                );
            }
            return rg;
        }

        //Returns the average velocity of all of the body parts
        //Using the velocity of the hips only has shown to result in more erratic movement from the limbs, so...
        //...using the average helps prevent this erratic movement
        Vector2 GetAvgVelocity()
        {
            Vector2 velSum = Vector2.zero;

            //ALL RBS
            int numOfRb = 0;
            foreach (var item in m_JdController.bodyPartsList)
            {
                numOfRb++;
                velSum.x += item.rb.velocity.x;
                velSum.y += item.rb.velocity.y;
            }

            var avgVel = velSum / numOfRb;
            return avgVel;
        }

        //normalized value of the difference in avg speed vs goal walking speed.
        public float GetMatchingVelocityReward(Vector3 velocityGoal, Vector3 actualVelocity)
        {
            //distance between our actual velocity and goal velocity
            var velDeltaMagnitude = Mathf.Clamp(Vector3.Distance(actualVelocity, velocityGoal), 0, targetWalkingSpeed);

            //return the value on a declining sigmoid shaped curve that decays from 1 to 0
            //This reward will approach 1 if it matches perfectly and approach zero as it deviates
            return Mathf.Pow(1 - Mathf.Pow(velDeltaMagnitude / targetWalkingSpeed, 2), 2);
        }

        void OnDrawGizmos()
        {
            if (isDebug)
            {
                foreach (var limb in allPhysicalLimbs)
                    limb.DebugDraw();
            }
        }
    }
}
