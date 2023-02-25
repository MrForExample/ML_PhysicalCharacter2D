using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace PhysicalCharacter2D
{
    public class GroundTrigger : MonoBehaviour
    {
        public bool touchingGround;
        const string k_Ground = "ground"; // Tag of ground object.

        /// <summary>
        /// Check for collision with ground, and optionally penalize agent.
        /// </summary>
        void OnTriggerEnter(Collider col)
        {
            if (col.transform.CompareTag(k_Ground))
            {
                touchingGround = true;
            }
        }

        /// <summary>
        /// Check for end of ground collision and reset flag appropriately.
        /// </summary>
        void OnTriggerExit(Collider col)
        {
            if (col.transform.CompareTag(k_Ground))
            {
                touchingGround = false;
            }
        }
    }
}