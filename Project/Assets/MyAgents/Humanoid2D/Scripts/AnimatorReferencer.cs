using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace PhysicalCharacter2D
{
    public class AnimatorReferencer : MonoBehaviour
    {
        public bool disableRefSpritesRender = true;
        public bool disableSpritesRender = false;
        public MimicAgent2D agent2D;
        Animator animator;
        public float maxAnimationLength = 1f;
        [HideInInspector]
        public BodyReferencer hips;
        BodyReferencer[] endEffectors;
        [HideInInspector]
        public Vector3[] endEffectorPos;
        [HideInInspector]
        public GroundTrigger[] footsLand;

        [HideInInspector] public Dictionary<string, BodyReferencer> bodyReferencersDict = new Dictionary<string, BodyReferencer>();

        public class BodyReferencer
        {
            public Transform bodyTransform;
            public Quaternion bodyLastQuaternion;
            public Quaternion bodyLastLocalQuaternion;
            public Vector2 bodyLastPosition;
            public Vector3 bodyLastCOM;
            public Vector3 bodyCOMLocalOffset;
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

            endEffectors = new BodyReferencer[5];
            endEffectorPos = new Vector3[5];
            for (int i = 0; i < agent2D.endEffectors.Length; i++)
                endEffectors[i] = bodyReferencersDict[agent2D.endEffectors[i].rb.name];

            hips = bodyReferencersDict[agent2D.hips.name];

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
            RecordAllLastEndEffectorPosition();
        }
        public void RecordAllLastEndEffectorPosition()
        {
            for (int i = 0; i < endEffectors.Length; i++)
                endEffectorPos[i] = endEffectors[i].bodyTransform.position - hips.bodyTransform.position;
        }
        public void ForwardAnimatior(float deltaTime)
        {
            animator.Update(deltaTime);
        }
    }
}
