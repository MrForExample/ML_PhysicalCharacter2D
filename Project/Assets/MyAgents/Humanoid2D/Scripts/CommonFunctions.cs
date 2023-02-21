using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace PhysicalCharacter2D
{
    public static class CommonFunctions
    {
        public static Quaternion CalculateLocalQuaternion(Quaternion previousRotation, Quaternion currentRotation)
        {
            return currentRotation * Quaternion.Inverse(previousRotation);
        }
        public static Vector3 CalculateAngularVelocity(Quaternion previousRotation, Quaternion currentRotation, float deltaTime)
        {
            Quaternion deltaRotation = CalculateLocalQuaternion(previousRotation, currentRotation);
            
            float angle  = 0.0f;
            Vector3 axis = Vector3.zero;
            
            deltaRotation.ToAngleAxis(out angle, out axis);
            
            angle *= Mathf.Deg2Rad;
            
            return axis * angle * (1.0f / deltaTime);
        }
    }
}
