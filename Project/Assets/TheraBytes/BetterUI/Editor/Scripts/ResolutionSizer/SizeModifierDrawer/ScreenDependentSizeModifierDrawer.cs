using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEditorInternal;
using UnityEngine;
using UnityEngine.UI;

namespace TheraBytes.BetterUi.Editor
{
    public abstract class ScreenDependentSizeDrawer<T> : ScreenDependentSizeModifierDrawer
    {
        private const string CanvasScalerWarnText = 
            "The Canvas Scaler is not set to 'Constant Pixel Size'. The resizing done by Better UI may lead to unexpected results. You can either change the canvas scaler or remove all size modifications of this sizer.";

        static readonly GUIContent removeSizeModsContent = new GUIContent("Remove Size Modifications", "Removes all size modifications of this sizer, so that the 'Current Value' will always be the same as the 'Optimized Size'.");

        static readonly GUIContent fixCanvasScalerContent = new GUIContent("Fix Canvas Scaler", "Changes the Canvas Scaler mode to 'Constant Pixel Size'. This will affect all elements beneath the root-canvas.");
        static GUIContent warningContent;
        static GUIContent WarningContent
        {
            get
            {
                if(warningContent == null)
                {
                    warningContent = EditorGUIUtility.IconContent("CollabError");
                    warningContent.tooltip = CanvasScalerWarnText;
                }

                return warningContent;
            }
        }

        Dictionary<string, ReorderableList> lists = new Dictionary<string, ReorderableList>();
        bool foldout;

        public override void OnGUI(Rect rect, SerializedProperty property, GUIContent label)
        {
            var sizer = property.GetValue<ScreenDependentSize<T>>();
            
            GUILayout.Space(-20); // cheat to draw above allocated rect

            EditorGUILayout.BeginVertical(GUI.skin.box);


            EditorGUILayout.LabelField("CurrentValue", GetValueString(sizer.LastCalculatedSize), EditorStyles.boldLabel);
            ShowField(property, "OptimizedSize", "Optimized Size", ref sizer.OptimizedSize);

            MonoBehaviour obj = (property.serializedObject.targetObject as MonoBehaviour);
            var scaler = obj.GetComponentInParent<CanvasScaler>();

            bool isConstantPixelSize = scaler == null || scaler.uiScaleMode == CanvasScaler.ScaleMode.ConstantPixelSize;
            bool isProblematic = !isConstantPixelSize && sizer.GetModifiers().Any(o => o.SizeModifiers.Any());

            EditorGUI.indentLevel += 1;
            EditorGUILayout.BeginHorizontal();
            foldout = EditorGUILayout.Foldout(foldout, "Size Modification");

            if(isProblematic)
            {
                if(GUILayout.Button(WarningContent, "Label", GUILayout.Width(16)))
                {
                    GenericMenu menu = new GenericMenu();
                    menu.AddItem(removeSizeModsContent, false, () => RemoveSizeModifiers(obj, sizer));
                    menu.AddItem(fixCanvasScalerContent, false, () => FixCanvasScaler(scaler));
                    menu.ShowAsContext();
                }
            }

            EditorGUILayout.EndHorizontal();

            if (foldout)
            {
                if(isProblematic)
                {
                    EditorGUILayout.BeginVertical("HelpBox");
                    EditorGUILayout.BeginHorizontal();

                    EditorGUILayout.LabelField(WarningContent);

                    EditorGUILayout.BeginVertical();
                    GUIStyle textStyle = EditorStyles.label;
                    textStyle.wordWrap = true;

                    GUILayout.Label(CanvasScalerWarnText, textStyle);

                    EditorGUILayout.BeginHorizontal();
                    GUILayout.FlexibleSpace();

                    if(GUILayout.Button(removeSizeModsContent, "minibutton"))
                    {
                        RemoveSizeModifiers(obj, sizer);
                    }

                    if(GUILayout.Button(fixCanvasScalerContent, "minibutton"))
                    {
                        FixCanvasScaler(scaler);
                    }

                    EditorGUILayout.EndHorizontal();
                    EditorGUILayout.EndVertical();
                    EditorGUILayout.EndHorizontal();
                    EditorGUILayout.EndVertical();
                    EditorGUILayout.Space();
                }

                EditorGUILayout.BeginHorizontal();

                OverrideScreenProperties overrideProps = null;

                #region find optimized resolution

                ScreenInfo screenInfo = null;
                if(obj != null)
                {
                    overrideProps = obj.GetComponentInParent<OverrideScreenProperties>();
                    if(overrideProps != null)
                    {
                        var settings = overrideProps.SettingsList.Items.FirstOrDefault(o => o.ScreenConfigName == sizer.ScreenConfigName)
                            ?? overrideProps.FallbackSettings;

                        OverrideScreenProperties parent = (settings.PropertyIterator().Any(o => o.Mode == OverrideScreenProperties.OverrideMode.Inherit))
                           ? overrideProps.GetComponentInParent<OverrideScreenProperties>()
                           : null;

                        float optWidth = overrideProps.CalculateOptimizedValue(settings, OverrideScreenProperties.ScreenProperty.Width, parent);
                        float optHeight = overrideProps.CalculateOptimizedValue(settings, OverrideScreenProperties.ScreenProperty.Height, parent);
                        float optDpi = overrideProps.CalculateOptimizedValue(settings, OverrideScreenProperties.ScreenProperty.Dpi, parent);

                        screenInfo = new ScreenInfo(new Vector2(optWidth, optHeight), optDpi);
                    }
                }

                if(screenInfo == null)
                {
                    screenInfo = ResolutionMonitor.GetOpimizedScreenInfo(sizer.ScreenConfigName);
                }

                #endregion

                string opt = string.Format("{0} x {1} @ {2} DPI", screenInfo.Resolution.x, screenInfo.Resolution.y, screenInfo.Dpi);

                EditorGUILayout.LabelField(opt, EditorStyles.boldLabel);

                if (GUILayout.Button("Change"))
                {
                    Selection.activeObject = (UnityEngine.Object)overrideProps ?? ResolutionMonitor.Instance;
                }
                EditorGUILayout.EndHorizontal();



                ShowOptionalField(property, "MinSize", "UseMinSize", "Min Size", ref sizer.MinSize);
                ShowOptionalField(property, "MaxSize", "UseMaxSize", "Max Size", ref sizer.MaxSize);

                property.serializedObject.ApplyModifiedPropertiesWithoutUndo();

                EditorGUILayout.Space();

                DrawModifiers(property);

            }

            EditorGUI.indentLevel -= 1;
            EditorGUILayout.EndVertical();
        }

        private void FixCanvasScaler(CanvasScaler scaler)
        {
            Undo.RecordObject(scaler, "Fix Canvas Scaler Mode");
            scaler.uiScaleMode = CanvasScaler.ScaleMode.ConstantPixelSize;
        }

        private void RemoveSizeModifiers(Component container, ScreenDependentSize<T> sizer)
        {
            Undo.RecordObject(container, "Remove Size Modifiers");
            foreach (var mod in sizer.GetModifiers())
            {
                mod.SizeModifiers.Clear();
            }
        }


        protected void DrawModifierList(SerializedProperty property, string title)
        {
            var listProp = property.FindPropertyRelative("SizeModifiers");
            var list = GetList(listProp, title);

            property.serializedObject.Update();
            list.DoLayoutList();
            property.serializedObject.ApplyModifiedProperties();
        }

        ReorderableList GetList(SerializedProperty property, string title)
        {
            if (!(lists.ContainsKey(title)))
            {
                ReorderableList list = new ReorderableList(property.serializedObject, property, true, true, true, true);
                list.elementHeight = EditorGUIUtility.singleLineHeight + 4;
                list.drawHeaderCallback = (Rect rect) =>
                {
                    int tmp = EditorGUI.indentLevel;
                    EditorGUI.indentLevel = 0;
                    EditorGUI.LabelField(rect, title, EditorStyles.miniLabel);
                    EditorGUI.indentLevel = tmp;
                };

                list.drawElementCallback = (Rect rect, int index, bool isActive, bool isFocused) =>
                {
                    int tmp = EditorGUI.indentLevel;
                    EditorGUI.indentLevel = 0;

                    var element = list.serializedProperty.GetArrayElementAtIndex(index);

                    float height = EditorGUIUtility.singleLineHeight;
                    rect.y += 2;

                    EditorGUI.PropertyField(
                      new Rect(rect.x, rect.y, 90, height),
                      element.FindPropertyRelative("Mode"), GUIContent.none);

                    EditorGUI.PropertyField(
                        new Rect(rect.x + 100, rect.y, rect.width - 100, height),
                        element.FindPropertyRelative("Impact"), GUIContent.none);

                    EditorGUI.indentLevel = tmp;
                };

                lists.Add(title, list);
            }

            return lists[title];
        }

        void ShowOptionalField(SerializedProperty parentProp, string propName, string boolPropName, string displayName, ref T value)
        {
            var boolProp = parentProp.FindPropertyRelative(boolPropName);

            EditorGUILayout.BeginHorizontal();

            // HACK: the toggle has actually a width of 16 but for some reason, it is not clickable in that size.
            //       calculating based on the indent level seems to be the way to go.
            //       then the indent level for the other property is temporarily reduced to move the property closer
            //       to the desired position.
            //       However, the distance grows based on indent level but is okay for all Better UI controls.
            var width = 16 * (1 + EditorGUI.indentLevel);
            boolProp.boolValue = EditorGUILayout.Toggle( GUIContent.none, boolProp.boolValue, GUILayout.Width(width));

            EditorGUI.BeginDisabledGroup(!boolProp.boolValue);
            EditorGUILayout.BeginVertical();
            EditorGUI.indentLevel -= 1;

            ShowField(parentProp, propName, displayName, ref value);

            EditorGUI.indentLevel += 1;
            EditorGUILayout.EndVertical();
            EditorGUI.EndDisabledGroup();

            EditorGUILayout.EndHorizontal();
        }

        protected virtual void ShowField(SerializedProperty parentProp, string propName, string displayName, ref T value)
        {
            var prop = parentProp.FindPropertyRelative(propName);
            EditorGUILayout.PropertyField(prop, new GUIContent(displayName));
        }

        protected abstract void DrawModifiers(SerializedProperty property);
        protected abstract string GetValueString(T obj);
    }

    public abstract class ScreenDependentSizeModifierDrawer : PropertyDrawer
    {
        [CustomPropertyDrawer(typeof(SizeModifierCollection))]
        public class SizeModifierCollectionDrawer : PropertyDrawer
        {
            ReorderableList list;

            ReorderableList GetList(SerializedProperty property)
            {
                if (list == null)
                {
                    list = new ReorderableList(property.serializedObject, property, true, true, true, true);
                    list.elementHeight = EditorGUIUtility.singleLineHeight + 4;
                    list.drawHeaderCallback = (Rect rect) =>
                    {
                        EditorGUI.LabelField(rect, "Size Modifiers");
                    };
                    list.drawElementCallback = (Rect rect, int index, bool isActive, bool isFocused) =>
                    {
                        var element = list.serializedProperty.GetArrayElementAtIndex(index);

                        float height = EditorGUIUtility.singleLineHeight;
                        rect.y += 2;

                        EditorGUI.PropertyField(
                          new Rect(rect.x, rect.y, 90, height),
                          element.FindPropertyRelative("Mode"), GUIContent.none);

                        EditorGUI.PropertyField(
                            new Rect(rect.x + 100, rect.y, rect.width - 100, height),
                            element.FindPropertyRelative("Impact"), GUIContent.none);
                    };

                }

                return list;
            }

            public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
            {
                return 0; // use layout
            }

            public override void OnGUI(Rect rect, SerializedProperty property, GUIContent label)
            {
                var listProp = property.FindPropertyRelative("SizeModifiers");
                var list = GetList(listProp);

                property.serializedObject.Update();
                list.DoLayoutList();

            }
        }

    }
}
