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
        public static float CalculatePDForce1DStable(float targetPos, float targetVel, float nowPos, float nowVel, float frequency, float damping, float dt)
        {
            float kp, kd, ksg, kdg;
            PreCalculatePD(frequency, damping, dt, out kp, out kd);
            FinalCalculatePD(kp, kd, dt, out ksg, out kdg);
            float nowPDForce = ksg * (targetPos - nowPos) + kdg * (targetVel - nowVel);
            return nowPDForce;
        }
        public static Vector3 CalculatePDTorque(Quaternion desiredRotation, Quaternion nowRotation, Rigidbody rigidbody, float frequency, float damping, float dt)
        {
            float kp, kd;
            PreCalculatePD(frequency, damping, dt, out kp, out kd);
            Vector3 x;
            float xMag;
            Quaternion q = desiredRotation * Quaternion.Inverse(nowRotation);
            // Q can be the-long-rotation-around-the-sphere eg. 350 degrees
            // We want the equivalant short rotation eg. -10 degrees
            // Check if rotation is greater than 190 degees == q.w is negative
            if (q.w < 0)
            {
                // Convert the quaterion to eqivalent "short way around" quaterion
                q.x = -q.x;
                q.y = -q.y;
                q.z = -q.z;
                q.w = -q.w;
            }
            q.ToAngleAxis (out xMag, out x);
            x.Normalize ();
            x *= Mathf.Deg2Rad;
            Vector3 pidv = kp * x * xMag - kd * rigidbody.angularVelocity;
            Quaternion rotInertia2World = rigidbody.inertiaTensorRotation * nowRotation;
            pidv = Quaternion.Inverse(rotInertia2World) * pidv;
            pidv.Scale(rigidbody.inertiaTensor);
            pidv = rotInertia2World * pidv;
            return pidv;
        }
        static void PreCalculatePD(float frequency, float damping, float dt, out float kp, out float kd)
        {
            kp = (6f*frequency)*(6f*frequency)* 0.25f;
            kd = 4.5f*frequency*damping;
        }
        static void FinalCalculatePD(float kp, float kd, float dt, out float ksg, out float kdg)
        {
            float g = 1 / (1 + kd * dt + kp * dt * dt);
            ksg = kp * g;
            kdg = (kd + kp * dt) * g;
        }
    }
}
