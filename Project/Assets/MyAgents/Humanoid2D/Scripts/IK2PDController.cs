using System;
using System.Collections.Generic;
using UnityEngine;

namespace PhysicalCharacter2D
{
    public class PhysicalBody
    {
        public Rigidbody rg;
        public ConfigurableJoint joint;
        public float link_length;
        public Quaternion ik_axis_bias;
        Quaternion joint_init_axis_rotation;
        Quaternion world_to_joint_space;
        Quaternion joint_to_world_space;
        Quaternion last_joint_target_local_rotation = Quaternion.identity;

        public PhysicalBody(Rigidbody new_rg, ConfigurableJoint new_joint, ConfigurableJoint child_joint = null)
        {
            rg = new_rg;
            joint = new_joint;

            joint_init_axis_rotation = joint.transform.localRotation;
            // Calculate the rotation expressed by the joint's axis and secondary axis and get its space transform matrix
            var right = joint.axis;
            var up = joint.secondaryAxis;
            var forward = Vector3.Cross(right, up).normalized;
            world_to_joint_space = Quaternion.LookRotation(forward, up);
            joint_to_world_space = Quaternion.Inverse(world_to_joint_space);

            last_joint_target_local_rotation = joint.targetRotation;

            // Calculate body's link length
            if (child_joint != null)
                link_length = (CommonFunctions.GetNowJointPosition(child_joint) - CommonFunctions.GetNowJointPosition(joint)).magnitude;
        }

        public void SetJointPDTarget(Quaternion target_local_rotation, float deltaTime)
        {
            // Matrix change of basis, Transform difference of local rotation back into joint space
            Quaternion joint_target_local_rotation = joint_to_world_space * (Quaternion.Inverse(target_local_rotation) * joint_init_axis_rotation) * world_to_joint_space;
            // Set target rotation to our newly calculated rotation
            joint.targetRotation = joint_target_local_rotation;    

            // Set target angular velocity according to last and current real joint target rotation
            Vector3 now_target_angular_velocity = CommonFunctions.CalculateAngularVelocity(last_joint_target_local_rotation, joint_target_local_rotation, deltaTime);
            joint.targetAngularVelocity = now_target_angular_velocity;

            last_joint_target_local_rotation = joint_target_local_rotation;
        }
    }
    public class PhysicalLimb
    {
        [HideInInspector]
        public PhysicalBody[] limb_bodies;
        [HideInInspector]
        public PhysicalBody end_body, lower_body, upper_body;
        float max_leg_extend_length, min_leg_extend_length;
        Vector3 max_target_c_dir, min_target_c_dir;
        Transform pole;
        Transform rootParent;

        // Debug
        bool is_debug;
        List<Vector3> draw_triangles = new List<Vector3>();
        List<Tuple<Vector3, Vector3>> draw_lines = new List<Tuple<Vector3, Vector3>>();
        List<Color> lines_colors = new List<Color>();

        public PhysicalLimb(PhysicalBody[] all_limb_bodies, Transform pole_target, bool debug_activate = false)
        {
            limb_bodies = all_limb_bodies;
            end_body = limb_bodies[0]; lower_body = limb_bodies[1]; upper_body = limb_bodies[2];
            
            pole = pole_target;
            rootParent = upper_body.joint.connectedBody.transform;

            max_leg_extend_length = lower_body.link_length + upper_body.link_length;

            var min_lower_angle = Mathf.Deg2Rad * Mathf.Max(0f, (180f - (lower_body.joint.highAngularXLimit.limit - lower_body.joint.lowAngularXLimit.limit)));
            min_leg_extend_length = CommonFunctions.LawOfCosinesFindSide(upper_body.link_length, lower_body.link_length, min_lower_angle);

            Vector3 end_joint_position = CommonFunctions.GetNowJointPosition(end_body.joint);
            Vector3 root_joint_position = CommonFunctions.GetNowJointPosition(upper_body.joint);
            Vector3 target_c_dir = (end_joint_position - root_joint_position).normalized;

            // Quaternion.AngleAxis when axis pointing outwards, positive angle rotates vector clockwise 
            max_target_c_dir = rootParent.InverseTransformDirection(Quaternion.AngleAxis(upper_body.joint.highAngularXLimit.limit, Vector3.back) * target_c_dir);
            min_target_c_dir = rootParent.InverseTransformDirection(Quaternion.AngleAxis(upper_body.joint.lowAngularXLimit.limit, Vector3.back) * target_c_dir);

            Vector3 ik_plane_normal = Vector3.Cross(target_c_dir, pole.position - root_joint_position);

            // Calculate rotation transformation matrix from IK plane to body's local space
            for (int i = 1; i < limb_bodies.Length; i++)
            {
                var joint = limb_bodies[i].joint;
                Quaternion ik_wrd_rot = Quaternion.LookRotation((CommonFunctions.GetNowJointPosition(limb_bodies[i-1].joint) - CommonFunctions.GetNowJointPosition(limb_bodies[i].joint)).normalized, ik_plane_normal);
                Quaternion ik_loc_rot = Quaternion.Inverse(joint.connectedBody.transform.rotation) * ik_wrd_rot;
                limb_bodies[i].ik_axis_bias = Quaternion.Inverse(ik_loc_rot) * joint.transform.localRotation;               
            }

            is_debug = debug_activate;
        }
        /// <summary>
        /// Moving leg to given target using IK->PD controller
        /// </summary>
        /// <param name="now_target_end_point"> current target feet joint world position </param>
        public void LimbIKMoveToTargrtPoint(Vector3 now_target_end_point, float deltaTime)
        {
            /*
                            c
            leg root joint ______ feet joint
                           \    /
                          a \  / b
                             \/
                         knee joint
            */

            ConfigurableJoint joint_u = upper_body.joint;
            ConfigurableJoint joint_l = lower_body.joint;
            float upper_leg_length = upper_body.link_length;
            float lower_leg_length = lower_body.link_length;

            // Get base height of upper joint for character to maintain
            Vector3 joint_pos_u = CommonFunctions.GetNowJointPosition(joint_u);
            Vector3 target_c = Vector3.ClampMagnitude(now_target_end_point - joint_pos_u, max_leg_extend_length);
            Vector3 ik_plane_normal = Vector3.Cross(target_c, pole.position - joint_pos_u);

            float angle_u = CommonFunctions.LawOfCosinesFindAngle(upper_leg_length, target_c.magnitude, lower_leg_length);
            Vector3 target_a = Quaternion.AngleAxis(angle_u * Mathf.Rad2Deg, ik_plane_normal) * target_c.normalized * upper_leg_length;
            Vector3 joint_pos_l = joint_pos_u + target_a;
            Vector3 target_b = now_target_end_point - joint_pos_l;
            // Transform link-ik axis rotation from world-local-joint space
            Quaternion ik_joint_u_target_wrd_rot = Quaternion.LookRotation(target_a.normalized, ik_plane_normal);
            Quaternion ik_joint_u_target_loc_rot = Quaternion.Inverse(joint_u.connectedBody.transform.rotation) * ik_joint_u_target_wrd_rot;
            Quaternion joint_u_target_loc_rot = ik_joint_u_target_loc_rot * upper_body.ik_axis_bias;

            Quaternion ik_joint_l_target_wrd_rot = Quaternion.LookRotation(target_b.normalized, ik_plane_normal);
            Quaternion ik_joint_l_target_loc_rot = Quaternion.Inverse(joint_l.connectedBody.transform.rotation) * ik_joint_l_target_wrd_rot;
            Quaternion joint_l_target_loc_rot = ik_joint_l_target_loc_rot * lower_body.ik_axis_bias;
            
            upper_body.SetJointPDTarget(joint_u_target_loc_rot, deltaTime);
            lower_body.SetJointPDTarget(joint_l_target_loc_rot, deltaTime);

            // Debug
            if (is_debug)
            {
                draw_triangles.Clear();
                draw_lines.Clear();
                lines_colors.Clear();

                Vector3 ik_plane_center = (joint_pos_l + joint_pos_u + now_target_end_point) / 3f;
                draw_lines.Add(new Tuple<Vector3, Vector3>(ik_plane_center, ik_plane_center + ik_plane_normal));
                lines_colors.Add(Color.white);

                draw_triangles.Add(joint_pos_u);
                draw_triangles.Add(joint_pos_u + target_a);
                draw_triangles.Add(now_target_end_point);

                draw_lines.Add(new Tuple<Vector3, Vector3>(joint_pos_u, pole.position));
                lines_colors.Add(Color.blue);
            }
        }
        /// <summary>
        /// Both dir_a and len_a should be in range [-1f, 1f]
        /// </summary>
        public Vector3 CalculateTargetEndPoint(float dir_a, float len_a)
        {
            dir_a = (dir_a + 1f) * 0.5f;
            len_a = (len_a + 1f) * 0.5f;

            var target_c = rootParent.TransformDirection(Vector3.Slerp(min_target_c_dir, max_target_c_dir, dir_a).normalized) * Mathf.Lerp(min_leg_extend_length, max_leg_extend_length, len_a);
            return CommonFunctions.GetNowJointPosition(limb_bodies[2].joint) + target_c;
        }

        public void DebugDraw()
        {
            // Draw IK triangles
            Gizmos.color = Color.green;
            for (int i = 0; i < draw_triangles.Count; i++)
            {
                Gizmos.DrawLine(draw_triangles[i], draw_triangles[(i+1) % 3 == 0 ? i-2 : i+1]);
                Gizmos.DrawSphere(draw_triangles[i], 0.1f);
            }
            // Draw line
            for (int i = 0; i < draw_lines.Count; i++)
            {
                Gizmos.color = lines_colors[i];
                Gizmos.DrawLine(draw_lines[i].Item1, draw_lines[i].Item2);
            }
            // Draw pole
            Gizmos.color = Color.blue;
            Gizmos.DrawSphere(pole.position, 0.065f);
        }
    }
}
