using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEngine;

namespace TheraBytes.BetterUi.Editor
{
    public delegate void ScreenConfigConnectionCallback(string configName, SerializedProperty property);

    public static class ScreenConfigConnectionHelper
    {
        static HashSet<int> foldoutHashes = new HashSet<int>();

        public static void DrawSizerGui(string title, SerializedProperty collection, ref SerializedProperty fallback)
        {
            int indent = EditorGUI.indentLevel;
            EditorGUI.indentLevel = 0;
            DrawGui(title, collection, ref fallback, null, (name, o) =>
            {
                var obj = o.GetValue<ScreenDependentSize>();
                obj.DynamicInitialization();
            });

            EditorGUI.indentLevel = indent;
        }


        public static void DrawGui(string title, SerializedProperty collection, ref SerializedProperty fallback,
            ScreenConfigConnectionCallback drawContent = null, 
            ScreenConfigConnectionCallback newElementInitCallback = null,
            ScreenConfigConnectionCallback elementDeleteCallback = null)
        {
            int baseHash = title.GetHashCode();

            GUILayout.Space(2);

            Rect bgRect = EditorGUILayout.BeginVertical("box");
            
            EditorGUILayout.LabelField(title, EditorStyles.boldLabel);

            var configs = collection.GetValue<ISizeConfigCollection>();
            if (configs.IsDirty)
            {
                configs.Sort();
                return;
            }

            string currentConfig = configs.GetCurrentConfigName();
            string currentScreen = ResolutionMonitor.CurrentScreenConfiguration?.Name;
            SerializedProperty items = collection.FindPropertyRelative("items");


            // LIST
            bool configFound = currentConfig != null;
            HashSet<string> names = new HashSet<string>();

            for (int i = 0; i < items.arraySize; i++)
            {
                SerializedProperty item = items.GetArrayElementAtIndex(i);
                var nameProp = item.FindPropertyRelative("screenConfigName");
                string name = "?";

                if (nameProp != null)
                {
                    name = nameProp.stringValue;
                    names.Add(name);
                }
                else
                {
                    Debug.LogError("no serialized property named 'screenConfigName' found.");
                }

                bool foldout = false;

                // DELETE
                DrawItemHeader(name, baseHash, false, currentConfig, out foldout, () =>
                {
                    if (elementDeleteCallback != null)
                        elementDeleteCallback(name, item);

                    items.DeleteArrayElementAtIndex(i);
                    i--;
                    foldout = false;
                });

                if (foldout)
                {
                    if (drawContent != null)
                        drawContent(name, item);
                    else
                        EditorGUILayout.PropertyField(item);
                }
            }

            // FALLBACK
            bool fallbackFoldout;
            string fallbackName = string.Format("{0} (Fallback)", ResolutionMonitor.Instance.FallbackName);
            DrawItemHeader(fallbackName, baseHash, !configFound, currentConfig ?? fallbackName, out fallbackFoldout);
            if (fallbackFoldout)
            {
                if (drawContent != null)
                    drawContent(fallbackName, fallback);
                else
                    EditorGUILayout.PropertyField(fallback);
            }

            // ADD NEW
            string[] options = ResolutionMonitor.Instance.OptimizedScreens
                .Where(o => !(names.Contains(o.Name)))
                .Select(o => o.Name)
                .ToArray();

            if (options.Length > 0)
            {

                EditorGUILayout.BeginHorizontal();
                GUILayout.FlexibleSpace();

                // Quick Action: Add current config
                if (currentConfig != currentScreen)
                {
                    GUIStyle style = "minibutton";
                    var content = EditorGUIUtility.IconContent("OL Plus");
                    content.text = $" {currentScreen}";

                    var size = style.CalcSize(content);
                    Rect rect = new Rect(bgRect.xMax - size.x - 20, bgRect.y + 3, size.x, 16);

                    if (GUI.Button(rect, content, "minibutton"))
                    {
                        AddSizerToList(currentScreen, ref fallback, items, newElementInitCallback);
                        configs.MarkDirty();
                    }
                }

                //int idx = EditorGUILayout.Popup(-1, options, "OL Plus", GUILayout.Width(20));
                Rect r = new Rect(bgRect.x + bgRect.width - 20, bgRect.y + 3, 20, 20);
                int idx = EditorGUI.Popup(r, -1, options, "OL Plus");

                if (idx != -1)
                {
                    string name = options[idx];
                    idx = -1;

                    AddSizerToList(name, ref fallback, items, newElementInitCallback);
                    configs.MarkDirty();
                }

                EditorGUILayout.EndHorizontal();
            }

            fallback.serializedObject.ApplyModifiedProperties();

            // immediately handle the ordering of the configs and update the serialized object
            // otherwise this would happen one frame delayed and an error would popup in the console.
            if(configs.IsDirty)
            {
                configs.Sort();
                fallback.serializedObject.Update();
            }

            EditorGUILayout.EndVertical();
        }

        public static void AddSizerToList(string configName, ref SerializedProperty fallback, SerializedProperty items, 
            ScreenConfigConnectionCallback callback = null)
        {
            string fallbackPath = fallback.propertyPath;
            items.arraySize += 1;
            var newElement = items.GetArrayElementAtIndex(items.arraySize - 1);

            SerializedPropertyUtil.Copy(fallback, newElement);

            // after the copy action the property pointer points somewhere.
            // so, point to the right prop again.
            newElement = items.GetArrayElementAtIndex(items.arraySize - 1);
            fallback = fallback.serializedObject.FindProperty(fallbackPath);

            if (callback != null)
            {
                callback(configName, newElement);
            }

            var prop = newElement.FindPropertyRelative("screenConfigName");
            if (prop != null)
                prop.stringValue = configName;
            else
                Debug.LogError("no serialized property named 'screenConfigName' found.");
        }

        private static void DrawItemHeader(string configName, int baseHash, bool setCurrent, string currentConfigName, out bool foldout, Action deleteCallback = null)
        {
            int hash = GetHash(baseHash, configName);
            bool isCurrentConfig = configName == currentConfigName;
            bool isSimulatedConfig = (ResolutionMonitor.SimulatedScreenConfig != null) && (configName == ResolutionMonitor.SimulatedScreenConfig.Name);
            bool exists = ResolutionMonitor.Instance.FallbackName + " (Fallback)" == configName
                || ResolutionMonitor.Instance.OptimizedScreens.Any(o => o.Name == configName);

            foldout = foldoutHashes.Contains(hash) || isCurrentConfig;

            EditorGUILayout.BeginHorizontal();

            string title = string.Format("{0} {1}{2} {3}{4}",
                (foldout) ? "▼" : "►",
                (isCurrentConfig) ? "♦" : "◊",
                (isSimulatedConfig) ? " ⃰" : " ",
                configName,
                (exists) ? "" : " (‼ not found ‼)");

            if (GUILayout.Button(title, "TextField", GUILayout.ExpandWidth(true)))//(foldout) ? "MiniPopup" : "MiniPullDown"))
            {
                if (!(isCurrentConfig) && !(foldoutHashes.Remove(hash)))
                {
                    foldoutHashes.Add(hash);
                    foldout = true;
                }
            }

           GUILayout.Space(-6);

            if (deleteCallback != null)
            {
                if (GUILayout.Button("X", "SearchCancelButton", GUILayout.Width(20)))//"MiniButton", GUILayout.Width(20))))
                {
                    if (EditorUtility.DisplayDialog("Delete?",
                    string.Format("Do you really want to delete the configuration '{0}'?", configName),
                    "Delete", "Cancel"))
                    {
                        deleteCallback();
                    }
                }
            }
            else
            {
                GUILayout.Box("", "SearchCancelButtonEmpty", GUILayout.Width(20));
            }

            EditorGUILayout.EndHorizontal();
        }

        private static int GetHash(int baseHash, string configName)
        {
            return baseHash ^ configName.GetHashCode();
        }
    }
}
