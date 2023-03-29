using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace PhysicalCharacter2D
{
    public class AnimatorReferencerIK2PD : MonoBehaviour
    {
        public bool disableRefSpritesRender = true;
        public bool disableSpritesRender = false;
        public MimicAgent2DIK2PD agent2D;
        Animator animator;
        public float maxAnimationLength = 1f;
        [HideInInspector]
        public BodyReferencer hips;
        [HideInInspector]
        public List<EffectorReferencer> effectorRefs = new List<EffectorReferencer>();
        public class EffectorReferencer
        {
            public Transform hips, rootJointParent, endJointParent;
            Vector2 rootJointLocalOffset, endJointLocalOffset;
            public Vector2 rootEffectorOffset, endEffectorOffset;
            public EffectorReferencer(Vector3 newRootJointLocalOffset, Vector3 newEndJointLocalOffset, Transform newHip, Transform newRootJointParent, Transform newEndJointParent)
            {
                hips = newHip;
                rootJointParent = newRootJointParent;
                endJointParent = newEndJointParent;
                rootJointLocalOffset = newRootJointLocalOffset;
                endJointLocalOffset = newEndJointLocalOffset;
            }
            public void recordLastTarget()
            {
                Vector3 rootJointPos = rootJointParent.position + rootJointParent.TransformVector(rootJointLocalOffset);
                rootEffectorOffset = rootJointPos - hips.position;
                endEffectorOffset = endJointParent.position + endJointParent.TransformVector(endJointLocalOffset) - rootJointPos;
            }
        }
        [HideInInspector]
        public GroundTrigger[] footsLand;

        [HideInInspector] public Dictionary<string, BodyReferencer> bodyReferencersDict = new Dictionary<string, BodyReferencer>();

        public class BodyReferencer
        {
            public Transform bodyTransform;
            public Quaternion bodyLastQuaternion;
            public Quaternion bodyLastLocalQuaternion;
            public Vector2 bodyLastPosition;
            public Vector2 bodyLastCOM;
            public Vector2 bodyCOMLocalOffset;
            public BodyReferencer(Transform newBodyTransform)
            {
                bodyTransform = newBodyTransform;
            }
            public void recordLastTarget()
            {
                bodyLastQuaternion = bodyTransform.rotation;
                bodyLastLocalQuaternion = bodyTransform.localRotation;
                bodyLastPosition = bodyTransform.position;
                bodyLastCOM = bodyTransform.position + bodyTransform.TransformVector(bodyCOMLocalOffset);
            }
        }
        void Start()
        {
            animator = GetComponent<Animator>();

            var allTransforms = transform.GetComponentsInChildren<Transform>();
            foreach (var bp in agent2D.m_JdController.bodyPartsDict.Values)
            {
                Transform newBodyTransform = null;
                foreach (var t in allTransforms)
                {
                    if (t.name == bp.rb.name)
                    {
                        newBodyTransform = t;
                        break;
                    }
                }
                bodyReferencersDict.Add(bp.rb.name, new BodyReferencer(newBodyTransform));
                var refBp = bodyReferencersDict[bp.rb.name];
                refBp.bodyCOMLocalOffset = refBp.bodyTransform.InverseTransformVector(bp.rb.worldCenterOfMass - bp.rb.transform.position);
            }

            hips = bodyReferencersDict[agent2D.hips.name];

            foreach (var limb in agent2D.allPhysicalLimbs)
            {
                Transform rootJointParent = limb.upper_body.joint.connectedBody.transform;
                Transform endJointParent = limb.end_body.joint.connectedBody.transform;
                Vector3 root_joint_position = CommonFunctions.GetNowJointPosition(limb.upper_body.joint);
                Vector3 end_joint_position = CommonFunctions.GetNowJointPosition(limb.end_body.joint);

                Vector3 newRootJointLocalOffset = rootJointParent.InverseTransformVector(root_joint_position - rootJointParent.position);
                Vector3 newEndJointLocalOffset = endJointParent.InverseTransformVector(end_joint_position - endJointParent.position);
                Transform newRootJointParent = bodyReferencersDict[rootJointParent.name].bodyTransform;
                Transform newEndJointParent = bodyReferencersDict[endJointParent.name].bodyTransform;

                var newEffectorRef = new EffectorReferencer(newRootJointLocalOffset, newEndJointLocalOffset, hips.bodyTransform, newRootJointParent, newEndJointParent);
                effectorRefs.Add(newEffectorRef);
            }

            if (disableRefSpritesRender)
            {
                var allRefSprites = transform.GetComponentsInChildren<SpriteRenderer>(true);
                foreach (var sr in allRefSprites)
                {
                    sr.enabled = false;
                    sr.gameObject.SetActive(false);
                }
            }
            if (disableSpritesRender)
            {         
                var allSprites = agent2D.GetComponentsInChildren<SpriteRenderer>(true);
                foreach (var sr in allSprites)
                {
                    sr.enabled = false;
                    sr.gameObject.SetActive(false);
                }
            }

            footsLand = new GroundTrigger[2];
            footsLand[0] = bodyReferencersDict[agent2D.footL.name].bodyTransform.GetComponentInChildren<GroundTrigger>(true);
            footsLand[1] = bodyReferencersDict[agent2D.footR.name].bodyTransform.GetComponentInChildren<GroundTrigger>(true);
        }

        public void ReferenceStateInitializationForRef()
        {
            animator.Update(Random.Range(0f, maxAnimationLength));
            RecordAllBodiesLastTarget();
        }
        public void RecordAllBodiesLastTarget()
        {
            foreach (var bodyPart in bodyReferencersDict.Values)
                bodyPart.recordLastTarget();
            RecordAllLastEndEffectorTarget();
        }
        public void RecordAllLastEndEffectorTarget()
        {
            foreach (var effector in effectorRefs)
                effector.recordLastTarget();
        }
        public void ForwardAnimatior(float deltaTime)
        {
            animator.Update(deltaTime);
        }
    }
}
