using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace PhysicalCharacter2D
{
    public static class CommonFunctions
    {
        public static Quaternion CalculateLocalQuaternion(Quaternion parentRotation, Quaternion childRotation)
        {
            return childRotation * Quaternion.Inverse(parentRotation);
        }
        public static Vector3 CalculateEulerAngleDiff(Quaternion last_rotation, Quaternion now_rotation)
        {
            Quaternion delta_rotation = now_rotation * Quaternion.Inverse(last_rotation);
            Vector3 euler_rotation = new Vector3(
                Mathf.DeltaAngle(0, delta_rotation.eulerAngles.x),
                Mathf.DeltaAngle(0, delta_rotation.eulerAngles.y),
                Mathf.DeltaAngle(0, delta_rotation.eulerAngles.z));
            return euler_rotation;
        }
        public static Vector3 CalculateAngularVelocity(Quaternion last_rotation, Quaternion now_rotation, float deltaTime)
        {
            Vector3 euler_rotation = CalculateEulerAngleDiff(last_rotation, now_rotation);
            Vector3 angular_velocity = euler_rotation / Time.fixedDeltaTime * Mathf.Deg2Rad;
            return angular_velocity;
        }
        /// <summary>
        /// Sometime value could be infinity!
        /// </summary>
        public static Vector3 CalculateAngularVelocityInf(Quaternion previousRotation, Quaternion currentRotation, float deltaTime)
        {
            Quaternion deltaRotation = CalculateLocalQuaternion(previousRotation, currentRotation);
            
            float angle  = 0.0f;
            Vector3 axis = Vector3.zero;
            
            deltaRotation.ToAngleAxis(out angle, out axis);
            
            angle *= Mathf.Deg2Rad;
            
            return axis * angle * (1.0f / deltaTime);
        }
        public static float LawOfCosinesFindAngle(float a, float b, float c)
        {
            /*
                c
            B ______ A
              \    /
             a \  / b
                \/
                C
            */
            float cosC = (Mathf.Pow(a, 2f) + Mathf.Pow(b, 2f) - Mathf.Pow(c, 2f)) / (2f * a * b);
            float C = Mathf.Acos(Mathf.Clamp(cosC, -1f, 1f));
            return C;
        }
        public static float LawOfCosinesFindSide(float a, float b, float C)
        {
            /*
                c
            B ______ A
              \    /
             a \  / b
                \/
                C
            */
            float c = Mathf.Sqrt(Mathf.Pow(a, 2f) + Mathf.Pow(b, 2f) - 2f * a * b * Mathf.Cos(C));
            return c;
        }
        public static Vector3 GetNowJointPosition(ConfigurableJoint joint)
        {
            return joint.connectedBody.transform.TransformPoint(joint.connectedAnchor);
        }
    }
}
