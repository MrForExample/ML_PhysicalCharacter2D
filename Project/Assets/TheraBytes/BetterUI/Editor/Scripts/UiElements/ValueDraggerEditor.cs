using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;

namespace TheraBytes.BetterUi.Editor
{
    [CustomEditor(typeof(ValueDragger)), CanEditMultipleObjects]
    public class ValueDraggerEditor : UnityEditor.Editor
    {
        SerializedProperty
            fallbackDragSettings, customDragSettings,
            fallbackValueSettings, customValueSettings,
            fallbackDragDistance, customDragDistance,
            value, onValueChanged;

        // as ValueDragger derives from BetterSelectable, we need to use this.
        BetterElementHelper<Selectable, BetterSelectable> helper =
            new BetterElementHelper<Selectable, BetterSelectable>();

        void OnEnable()
        {
            fallbackDragSettings = serializedObject.FindProperty("fallbackDragSettings");
            customDragSettings = serializedObject.FindProperty("customDragSettings");
            fallbackValueSettings = serializedObject.FindProperty("fallbackValueSettings");
            customValueSettings = serializedObject.FindProperty("customValueSettings");
            fallbackDragDistance = serializedObject.FindProperty("fallbackDragDistance");
            customDragDistance = serializedObject.FindProperty("customDragDistance");
            value = serializedObject.FindProperty("value");
            onValueChanged = serializedObject.FindProperty("onValueChanged");

        }

        public override void OnInspectorGUI()
        {
            ScreenConfigConnectionHelper.DrawGui("Drag Settings",
                customDragSettings, ref fallbackDragSettings, DrawDragSettings);

            ScreenConfigConnectionHelper.DrawGui("Value Settings",
                customValueSettings, ref fallbackValueSettings, DrawValueSettings);

            ScreenConfigConnectionHelper.DrawSizerGui("Drag Distance per Integer Step",
                customDragDistance, ref fallbackDragDistance);

            EditorGUILayout.PropertyField(value);
            EditorGUILayout.PropertyField(onValueChanged);

            helper.DrawGui(serializedObject);

            serializedObject.ApplyModifiedProperties();
        }

        private void DrawDragSettings(string configName, SerializedProperty prop)
        {
            var direction = prop.FindPropertyRelative("Direction");
            var invert = prop.FindPropertyRelative("Invert");

            EditorGUILayout.PropertyField(direction);
            EditorGUILayout.PropertyField(invert);
        }

        private void DrawValueSettings(string configName, SerializedProperty prop)
        {
            var hasMinValue = prop.FindPropertyRelative("HasMinValue");
            var minValue = prop.FindPropertyRelative("MinValue");

            var hasMaxValue = prop.FindPropertyRelative("HasMaxValue");
            var maxValue = prop.FindPropertyRelative("MaxValue");

            var wholeNumbers = prop.FindPropertyRelative("WholeNumbers");

            DrawCheckboxField("Min Value", hasMinValue, minValue);
            DrawCheckboxField("Max Value", hasMaxValue, maxValue);

            EditorGUILayout.Space();
            EditorGUILayout.PropertyField(wholeNumbers);
        }

        private static void DrawCheckboxField(string label, SerializedProperty checkValue, SerializedProperty fieldValue)
        {
            var rect = EditorGUILayout.GetControlRect();
            var checkRect = checkValue.boolValue
                ? new Rect(rect.x, rect.y, EditorGUIUtility.labelWidth, rect.height)
                : rect;

            checkValue.boolValue = EditorGUI.ToggleLeft(checkRect, label, checkValue.boolValue);
            if (checkValue.boolValue)
            {
                var fieldRect = new Rect(checkRect.xMax, rect.y, rect.xMax - checkRect.xMax, rect.height);
                EditorGUI.PropertyField(fieldRect, fieldValue, GUIContent.none);
            }

        }
    }
}
