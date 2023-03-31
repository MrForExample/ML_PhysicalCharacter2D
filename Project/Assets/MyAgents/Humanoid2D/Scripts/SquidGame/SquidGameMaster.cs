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

        KeyCode[] keypadCodes = new KeyCode[]
        {
            KeyCode.Alpha1,
            KeyCode.Alpha2,
            KeyCode.Alpha3,
            KeyCode.Alpha4,
            KeyCode.Alpha5,
            KeyCode.Alpha6,
            KeyCode.Alpha7,
            KeyCode.Alpha8,
            KeyCode.Alpha9
        };

        void Start()
        {
            cameraFollow = GetComponent<CameraFollow>();

            var sIs = FindObjectsOfType<Agent>().OfType<SquidInterface>();
            foreach (SquidInterface sI in sIs) 
            {
                allSquidGamers.Add(sI);
            }
        }

        // Update is called once per frame
        void Update()
        {
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

            for (int i = 0; i < 9; i++)
            {
                if (Input.GetKeyDown(keypadCodes[i]))
                {
                    ChangeCameraTarget(i);
                }
            }
        }

        public void ChangeCameraTarget(int i)
        {
            cameraFollow.target = cameraChangeTargets[i];
        }
    }
}

