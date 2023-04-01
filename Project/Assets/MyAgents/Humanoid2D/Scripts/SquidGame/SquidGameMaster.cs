using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using PhysicalCharacter2D;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgentsExamples;

namespace SquidGame
{
    public class SquidGameMaster : MonoBehaviour
    {
        CameraFollow cameraFollow;
        public Transform[] cameraChangeTargets;
        const float maxSupportTime = 1f;
        float nowSupportTime = 0f;
        bool canAgentsMove = true;
        List<SquidInterface> allSquidGamers = new List<SquidInterface>();

        int currentTargetIndex = 0;

        void Start()
        {
            cameraFollow = GetComponent<CameraFollow>();
            foreach (var t in cameraChangeTargets)
            {
                allSquidGamers.Add(t.GetComponentInParent<SquidInterface>());
            }

            /*
            var sIs = FindObjectsOfType<Agent>().OfType<SquidInterface>();
            foreach (SquidInterface sI in sIs) 
            {
                allSquidGamers.Add(sI);
            }
            */
        }

        // Update is called once per frame
        void Update()
        {
            // Stop
            if (Input.GetKeyDown(KeyCode.Space))
            {
                canAgentsMove = !canAgentsMove;
                foreach (var gamer in allSquidGamers)
                {
                    gamer.canAgentMove = canAgentsMove;
                }

                if (canAgentsMove)
                {
                    nowSupportTime = maxSupportTime;
                    foreach (var gamer in allSquidGamers)
                        gamer.TurnOnOrOffSupport(true);
                }
            }

            if (nowSupportTime > 0f)
            {
                nowSupportTime -= Time.deltaTime;
                if (nowSupportTime <= 0f)
                {
                    foreach (var gamer in allSquidGamers)
                        gamer.TurnOnOrOffSupport(false);
                }
            }

            // Kill
            if (Input.GetKeyDown(KeyCode.Q))
            {
                allSquidGamers[currentTargetIndex].TurnOnOrOffJoints(false);
            }
            else if (Input.GetKeyDown(KeyCode.R))
            {
                var gamer = allSquidGamers[currentTargetIndex];
                gamer.TurnOnOrOffJoints(true);
                gamer.TurnOnOrOffSupport(true);
                nowSupportTime = maxSupportTime;
            }
        }

        // Using on UI, click avatar to change target
        public void ChangeCameraTarget(int i)
        {
            cameraFollow.target = cameraChangeTargets[i];
            currentTargetIndex = i;
        }
    }
}

