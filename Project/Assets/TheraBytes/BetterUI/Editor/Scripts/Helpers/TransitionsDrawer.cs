using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;

namespace TheraBytes.BetterUi.Editor
{
    [CustomPropertyDrawer(typeof(Transitions))]
    public class TransitionsDrawer : PropertyDrawer
    {
        const float LineHeight = 20;
        const float SmallSpacing = 2;

        const float BigSpacing = 5;
        Transitions info;

        public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
        {
            var cur = property.GetValue<Transitions>();
            int lineCount = 1;
            int defaultCount = 1 + cur.StateNames.Count;
            float extraHeight = 2 * BigSpacing;
            switch (cur.Mode)
            {
                case Transitions.TransitionMode.SpriteSwap:
                case Transitions.TransitionMode.Animation:
                case Transitions.TransitionMode.ObjectActiveness:
                case Transitions.TransitionMode.LocationAnimationTransition:
                    lineCount += defaultCount;
                    break;

                case Transitions.TransitionMode.Alpha:
                case Transitions.TransitionMode.Color32Tint:
                    lineCount += defaultCount + 1;
                    extraHeight += BigSpacing;
                    break;

                case Transitions.TransitionMode.MaterialProperty:
                    lineCount += defaultCount + 2;
                    extraHeight += 2 * BigSpacing;
                    break;

                case Transitions.TransitionMode.ColorTint:
                    lineCount += defaultCount + 3;
                    extraHeight += BigSpacing;
                    break;

                case Transitions.TransitionMode.CustomCallback:
                    for(int i = 0; i < cur.StateNames.Count; i++)
                    {
                        extraHeight += GetCustomCallbackHeight(cur, i);
                    }
                    break;
                case Transitions.TransitionMode.None:
                default:
                    break;
            }

            return lineCount * LineHeight + extraHeight;
        }

        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {
            info = property.GetValue<Transitions>();
            
            DrawGui(position, info, property);
        }

        [Obsolete(EditorGuiUtils.ObsoleteMessage)]
        public static void DrawGui(Transitions sel, SerializedProperty property)
        {
            EditorGuiUtils.DrawOldMethodCallWarning();
        }

        public static void DrawGui(Rect position, Transitions sel, SerializedProperty property)
        {
            var rect = new Rect(position.x, position.y + BigSpacing, position.width, EditorGUIUtility.singleLineHeight);
            var mode = (Transitions.TransitionMode)EditorGUI.EnumPopup(rect, "Mode", sel.Mode);
            rect.y += rect.height + SmallSpacing;

            if (mode != sel.Mode)
            {
                sel.SetMode(mode);
            }

            if (sel.Mode == Transitions.TransitionMode.None)
                return;

            SerializedProperty transitionProp = null;
            List<string> postProps = new List<string>();
            

            switch (mode)
            {
                case Transitions.TransitionMode.ColorTint:
                    
                    transitionProp = property.FindPropertyRelative("colorTransitions");
                    postProps.Add("affectedColor");
                    postProps.Add("colorMultiplier");
                    postProps.Add("fadeDuration");
                    break;

                case Transitions.TransitionMode.Color32Tint:

                    transitionProp = property.FindPropertyRelative("color32Transitions");
                    postProps.Add("fadeDuration");
                    break;

                case Transitions.TransitionMode.SpriteSwap:

                    transitionProp = property.FindPropertyRelative("spriteSwapTransitions");
                    break;

                case Transitions.TransitionMode.Animation:

                    transitionProp = property.FindPropertyRelative("animationTransitions");
                    break;

                case Transitions.TransitionMode.ObjectActiveness:

                    transitionProp = property.FindPropertyRelative("activenessTransitions");
                    break;

                case Transitions.TransitionMode.Alpha:

                    transitionProp = property.FindPropertyRelative("alphaTransitions");
                    postProps.Add("fadeDuration");
                    break;

                case Transitions.TransitionMode.MaterialProperty:

                    transitionProp = property.FindPropertyRelative("materialPropertyTransitions");
                    postProps.Add("fadeDuration");
                    break;

                case Transitions.TransitionMode.LocationAnimationTransition:

                    transitionProp = property.FindPropertyRelative("locationAnimationTransitions");
                    break;

                case Transitions.TransitionMode.CustomCallback:

                    transitionProp = property.FindPropertyRelative("customTransitions");
                    break;
            }

            var targetProp = transitionProp.FindPropertyRelative("target");
            if (targetProp != null)
            {
                EditorGUI.PropertyField(rect, targetProp);
                rect.y += rect.height + SmallSpacing;
            }

            rect.y += BigSpacing;

            if (targetProp == null || sel.TransitionStates.Target != null)
            {
                if(mode == Transitions.TransitionMode.MaterialProperty)
                {
                    DrawMaterialPropertySelector(rect, sel, transitionProp);
                    rect.y += rect.height + BigSpacing;
                }

                EditorGUI.indentLevel += 1;

                var statesProp = transitionProp.FindPropertyRelative("states");
                for (int i = 0; i < statesProp.arraySize; i++)
                {
                    var p = statesProp.GetArrayElementAtIndex(i);
                    var pName = p.FindPropertyRelative("Name");
                    var pVal = p.FindPropertyRelative("StateObject");

                    if (mode == Transitions.TransitionMode.LocationAnimationTransition)
                    {
                        // special drawer for location transitions
                        var options = (sel.TransitionStates.Target as LocationAnimations).Animations.Select(o => o.Name).ToList();
                        options.Insert(0, "[ None ]");
                        int prevIdx = options.IndexOf(pVal.stringValue);
                        int newIdx = EditorGUI.Popup(rect, pName.stringValue, prevIdx, options.ToArray());
                        rect.y += rect.height + SmallSpacing;

                        if (prevIdx != newIdx)
                        {
                            pVal.stringValue = (newIdx > 0) ? options[newIdx] : "";
                            pVal.serializedObject.ApplyModifiedProperties();
                        }
                    }
                    else if(mode == Transitions.TransitionMode.CustomCallback)
                    {
                        rect.height = GetCustomCallbackHeight(sel, i);
                        EditorGUI.PropertyField(rect, pVal, new GUIContent(pName.stringValue));
                        rect.y += rect.height + SmallSpacing;
                    }
                    else
                    {
                        EditorGUI.PropertyField(rect, pVal, new GUIContent(pName.stringValue));
                        rect.y += rect.height + SmallSpacing;
                    }
                }

                EditorGUI.indentLevel -= 1;

                if (postProps.Count > 0)
                {
                    rect.y += BigSpacing;

                    foreach (string pName in postProps)
                    {
                        var p = transitionProp.FindPropertyRelative(pName);
                        EditorGUI.PropertyField(rect, p);
                        rect.y += rect.height + SmallSpacing;
                    }

#if UNITY_2019_1_OR_NEWER
                    if (!sel.StateNames.Contains("Selected") && sel.StateNames.Contains("Pressed"))
                    {
                        if (GUI.Button(rect, "Upgrade"))
                        {
                            sel.ComplementStateNames(Transitions.SelectionStateNames);
                        }

                        rect.y += rect.height + SmallSpacing;
                    }
#endif
                }
            }
            
        }

        private static float GetCustomCallbackHeight(Transitions sel, int i)
        {
            var custom = sel.TransitionStates as CustomTransitions;
            var state = custom.GetStates().ToList()[i];
            int cnt = state.StateObject.GetPersistentEventCount();

            if(cnt == 0)
            {
                return 5.75f * EditorGUIUtility.singleLineHeight;
            }

            return (3 + 2.75f * cnt) * EditorGUIUtility.singleLineHeight;
        }

        private static void DrawMaterialPropertySelector(Rect rect, Transitions sel, SerializedProperty transitionProp)
        {
            var matPropTrans = (sel.TransitionStates as MaterialPropertyTransition);
            if (matPropTrans == null)
                return;

            var img = (matPropTrans.Target as BetterImage);
            if (img == null)
                return;

            var options = img.MaterialProperties.FloatProperties.Select(o => o.Name).ToArray();

            var sp = transitionProp.FindPropertyRelative("propertyIndex");
            int cur = sp.intValue;
            int matPropIndex = EditorGUI.Popup(rect, "Affected Property", cur, options);

            sp.intValue = matPropIndex;
            
        }
    }
}
