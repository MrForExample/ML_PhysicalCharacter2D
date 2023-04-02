using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using PhysicalCharacter2D;
using System.Linq;
using Unity.MLAgents;
using Unity.MLAgentsExamples;
using ParticleEffect;

namespace SquidGame
{
    public class SquidGameMaster : MonoBehaviour
    {
        CameraFollow cameraFollow;
        public Transform[] cameraChangeTargets;
        public GameObject[] killUIs;
        const float maxSupportTime = 1f;
        float nowSupportTime = 0f;

        const float maxKillWaitTime = 1f;
        float nowKillWaitTime = 0f;

        bool canAgentsMove = true;
        List<SquidInterface> allSquidGamers = new List<SquidInterface>();

        int currentTargetIndex = 0, lastKillIndex;

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

            if (nowKillWaitTime > 0f)
            {
                nowKillWaitTime -= Time.deltaTime;
                if (nowKillWaitTime <= 0f)
                {
                    allSquidGamers[lastKillIndex].TurnOnOrOffJoints(false);
                    killUIs[lastKillIndex].SetActive(true);
                }
            }
            else
            {
                // Kill
                if (Input.GetKeyDown(KeyCode.Q))
                {
                    lastKillIndex = currentTargetIndex;
                    nowKillWaitTime = maxKillWaitTime;
                    (allSquidGamers[lastKillIndex] as Agent).GetComponentInChildren<SpecificSpawnPart>().ScaleAndStart();
                }
                else if (Input.GetKeyDown(KeyCode.R))
                {
                    var gamer = allSquidGamers[currentTargetIndex];
                    gamer.TurnOnOrOffJoints(true);
                    gamer.TurnOnOrOffSupport(true);
                    nowSupportTime = maxSupportTime;

                    killUIs[currentTargetIndex].SetActive(false);
                }
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

