using System;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgentsExamples;
using Unity.MLAgents.Sensors;
using BodyPart = Unity.MLAgentsExamples.BodyPart;
using Random = UnityEngine.Random;

namespace PhysicalCharacter2D
{
    public class MimicAgent2D : Agent
    {
        [Header("Reference motion")]
        public AnimatorReferencer animatorReferencer;

        [Header("Walk Speed")]
        [Range(0.1f, 10)]
        [SerializeField]
        //The walking speed to try and achieve
        private float m_TargetWalkingSpeed = 10;

        public float MTargetWalkingSpeed // property
        {
            get { return m_TargetWalkingSpeed; }
            set { m_TargetWalkingSpeed = Mathf.Clamp(value, .1f, m_maxWalkingSpeed); }
        }

        const float m_maxWalkingSpeed = 10; //The max walking speed

        //Should the agent sample a new goal velocity each episode?
        //If true, walkSpeed will be randomly set between zero and m_maxWalkingSpeed in OnEpisodeBegin()
        //If false, the goal velocity will be walkingSpeed
        public bool randomizeWalkSpeedEachEpisode;

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

        //This will be used as a stabilized model space reference point for observations
        //Because ragdolls can move erratically during training, using a stabilized reference transform improves learning
        OrientationCubeController m_OrientationCube;
        [HideInInspector]
        public JointDriveController m_JdController;
        EnvironmentParameters m_ResetParams;

        DecisionRequester decisionRequester;

        [HideInInspector]
        public BodyPart[] endEffectors;
        float totalMass;

        public override void Initialize()
        {
            m_OrientationCube = GetComponentInChildren<OrientationCubeController>();
            decisionRequester = GetComponentInChildren<DecisionRequester>();

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

            m_ResetParams = Academy.Instance.EnvironmentParameters;

            SetResetParameters();

            endEffectors = new BodyPart[4]{m_JdController.bodyPartsDict[handL], 
                                        m_JdController.bodyPartsDict[handR], 
                                        m_JdController.bodyPartsDict[footL], 
                                        m_JdController.bodyPartsDict[footR]};

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

            //Set our goal walking speed
            MTargetWalkingSpeed =
                randomizeWalkSpeedEachEpisode ? Random.Range(0.1f, m_maxWalkingSpeed) : MTargetWalkingSpeed;

            SetResetParameters();

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

        /// <summary>
        /// Add relevant information on each body part to observations.
        /// </summary>
        public void CollectObservationBodyPart(BodyPart bp, VectorSensor sensor)
        {
            // Reference motion
            var refBp = animatorReferencer.bodyReferencersDict[bp.rb.name];
            sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(CommonFunctions.CalculateAngularVelocity(refBp.bodyLastQuaternion, refBp.bodyTransform.rotation, Time.fixedDeltaTime * decisionRequester.DecisionPeriod)));
            sensor.AddObservation(CommonFunctions.CalculateLocalQuaternion(refBp.bodyLastQuaternion, m_OrientationCube.transform.rotation));

            //Ground Check
            sensor.AddObservation(bp.groundContact.touchingGround); // Is this bp touching the ground

            //Get velocities in the context of our orientation cube's space
            //Note: You can get these velocities in world space as well but it may not train as well.
            sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.velocity));
            sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.angularVelocity));

            //sensor.AddObservation(CommonFunctions.CalculateLocalQuaternion(bp.rb.rotation, m_OrientationCube.transform.rotation));
            sensor.AddObservation(bp.rb.transform.localRotation);
            // state size: 18

            if (bp.rb.transform != hips)
            {
                //Get position relative to hips in the context of our orientation cube's space
                sensor.AddObservation(m_OrientationCube.transform.InverseTransformDirection(bp.rb.position - hips.position));
                sensor.AddObservation(bp.currentStrength / m_JdController.maxJointForceLimit);
                // state size: 22
            }
            else
            {
                // Add root height for both reference and simulated agent 
                sensor.AddObservation(refBp.bodyTransform.position.y);
                sensor.AddObservation(hips.position.y);
                // state size: 20
            }
        }

        /// <summary>
        /// Loop over body parts to add them to observation.
        /// </summary>
        public override void CollectObservations(VectorSensor sensor)
        {
            var cubeForward = m_OrientationCube.transform.forward;

            //velocity we want to match
            Vector2 velGoal = cubeForward * MTargetWalkingSpeed;
            //ragdoll's avg vel
            Vector2 avgVel = GetAvgVelocity();

            //current ragdoll velocity. normalized
            sensor.AddObservation(Vector2.Distance(velGoal, avgVel));
            //avg body vel relative to cube
            Vector2 avgVelLocal = m_OrientationCube.transform.InverseTransformDirection(avgVel);
            sensor.AddObservation(avgVelLocal);
            //vel goal relative to cube
            Vector2 velGoalLocal = m_OrientationCube.transform.InverseTransformDirection(velGoal);
            sensor.AddObservation(velGoalLocal);

            //rotation deltas
            sensor.AddObservation(Quaternion.FromToRotation(-hips.right, cubeForward));
            sensor.AddObservation(Quaternion.FromToRotation(-head.up, cubeForward));
            // state size: 13 

            foreach (var bodyPart in m_JdController.bodyPartsList)
                CollectObservationBodyPart(bodyPart, sensor);
            // state size: 13 + 328
        }

        public override void OnActionReceived(ActionBuffers actionBuffers)
        {
            var bpDict = m_JdController.bodyPartsDict;
            var i = -1;

            var continuousActions = actionBuffers.ContinuousActions;
            bpDict[spine].SetJointTargetRotation(continuousActions[++i], 0f, 0f);

            bpDict[thighL].SetJointTargetRotation(continuousActions[++i], 0f, 0f);
            bpDict[thighR].SetJointTargetRotation(continuousActions[++i], 0f, 0f);
            bpDict[shinL].SetJointTargetRotation(continuousActions[++i], 0f, 0f);
            bpDict[shinR].SetJointTargetRotation(continuousActions[++i], 0f, 0f);
            bpDict[footR].SetJointTargetRotation(continuousActions[++i], 0f, 0f);
            bpDict[footL].SetJointTargetRotation(continuousActions[++i], 0f, 0f);

            bpDict[armL].SetJointTargetRotation(continuousActions[++i], 0f, 0f);
            bpDict[armR].SetJointTargetRotation(continuousActions[++i], 0f, 0);
            bpDict[forearmL].SetJointTargetRotation(continuousActions[++i], 0, 0);
            bpDict[forearmR].SetJointTargetRotation(continuousActions[++i], 0, 0);
            bpDict[head].SetJointTargetRotation(continuousActions[++i], 0f, 0);

            //update joint strength settings
            bpDict[spine].SetJointStrength(continuousActions[++i]);
            bpDict[head].SetJointStrength(continuousActions[++i]);
            bpDict[thighL].SetJointStrength(continuousActions[++i]);
            bpDict[shinL].SetJointStrength(continuousActions[++i]);
            bpDict[footL].SetJointStrength(continuousActions[++i]);
            bpDict[thighR].SetJointStrength(continuousActions[++i]);
            bpDict[shinR].SetJointStrength(continuousActions[++i]);
            bpDict[footR].SetJointStrength(continuousActions[++i]);
            bpDict[armL].SetJointStrength(continuousActions[++i]);
            bpDict[forearmL].SetJointStrength(continuousActions[++i]);
            bpDict[armR].SetJointStrength(continuousActions[++i]);
            bpDict[forearmR].SetJointStrength(continuousActions[++i]);
        }

        //Update OrientationCube
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
        float wp = 0.65f, wv = 0.1f, we = 0.15f, wc = 0.1f;
        //float wps = -2f, wvs = -0.1f, wes = -40f, wcs = -10f;
        float wps = -2f, wvs = -40f, wes = -40f, wcs = -10f;
        float CalculateImitationReward()
        {
            float rotDiffSum = 0f, angularVelDiffSum = 0f, endDiffSum = 0f, cOMDiffSum = 0f;

            Vector3 hipPos = m_JdController.bodyPartsDict[hips].rb.position;
            for (int i = 0; i < endEffectors.Length; i++)
            {
                endDiffSum += Vector3.SqrMagnitude(animatorReferencer.endEffectorPos[i] - (endEffectors[i].rb.position - hipPos));
            }

            float angle  = 0.0f;
            Vector3 axis = Vector3.zero,
                    worldCOM = new Vector3(0f, m_JdController.bodyPartsDict[hips].rb.worldCenterOfMass.y), 
                    targetWorldCOM = new Vector3(0f, animatorReferencer.bodyReferencersDict[hips.name].bodyLastCOM.y);
            foreach (var bp in m_JdController.bodyPartsDict.Values)
            {
                var refBp = animatorReferencer.bodyReferencersDict[bp.rb.name];

                Quaternion deltaRotation = CommonFunctions.CalculateLocalQuaternion(bp.rb.transform.localRotation, refBp.bodyLastLocalQuaternion);
                deltaRotation.ToAngleAxis(out angle, out axis);
                rotDiffSum += Mathf.Pow(angle * Mathf.Deg2Rad, 2f);

                Vector3 targetAngularVel = CommonFunctions.CalculateAngularVelocity(refBp.bodyLastQuaternion, refBp.bodyTransform.rotation, Time.fixedDeltaTime);
                angularVelDiffSum += Vector3.SqrMagnitude(targetAngularVel - bp.rb.angularVelocity);

                worldCOM += (bp.rb.worldCenterOfMass - m_JdController.bodyPartsDict[hips].rb.worldCenterOfMass) * bp.rb.mass;
                targetWorldCOM += (refBp.bodyLastCOM - animatorReferencer.hips.bodyLastCOM) * bp.rb.mass;
            }
            cOMDiffSum = Vector3.SqrMagnitude((targetWorldCOM - worldCOM) / totalMass);
            
            float rp = wp * Mathf.Exp(wps * rotDiffSum);
            float rv = wv * Mathf.Exp(wvs * angularVelDiffSum);
            float re = we * Mathf.Exp(wes * endDiffSum);
            float rc = wc * Mathf.Exp(wcs * cOMDiffSum);

            float ri = rp + rv + re + rc;
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
            var matchSpeedReward = GetMatchingVelocityReward(cubeForward * MTargetWalkingSpeed, GetAvgVelocity());

            // b. Rotation alignment with target direction.
            //This reward will approach 1 if it faces the target direction perfectly and approach zero as it deviates
            var lookAtTargetReward = (Vector3.Dot(cubeForward, -head.up) + 1) * .5F;

            float rg = matchSpeedReward * lookAtTargetReward;
            //Check for NaNs
            if (float.IsNaN(rg))
            {
                throw new ArgumentException(
                    "NaN in task reward.\n" +
                    $" cubeForward: {cubeForward}\n" + 
                    $" head.forward: {head.forward}\n" + 
                    $" maximumWalkingSpeed: {m_maxWalkingSpeed}\n" +
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
            var velDeltaMagnitude = Mathf.Clamp(Vector3.Distance(actualVelocity, velocityGoal), 0, MTargetWalkingSpeed);

            //return the value on a declining sigmoid shaped curve that decays from 1 to 0
            //This reward will approach 1 if it matches perfectly and approach zero as it deviates
            return Mathf.Pow(1 - Mathf.Pow(velDeltaMagnitude / MTargetWalkingSpeed, 2), 2);
        }

        /// <summary>
        /// Agent touched the target
        /// </summary>
        public void TouchedTarget()
        {
            AddReward(1f);
            EndEpisode();
        }

        public void SetTorsoMass()
        {
            m_JdController.bodyPartsDict[spine].rb.mass = m_ResetParams.GetWithDefault("spine_mass", 8);
            m_JdController.bodyPartsDict[hips].rb.mass = m_ResetParams.GetWithDefault("hip_mass", 8);
        }

        public void SetResetParameters()
        {
            SetTorsoMass();
        }
    }
}
