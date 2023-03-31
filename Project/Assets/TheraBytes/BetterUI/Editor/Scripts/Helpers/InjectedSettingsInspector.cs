using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEditor;
using UnityEngine;

namespace TheraBytes.BetterUi.Editor
{
    public class InjectedSettingsInspector
    {
        class SettingsProperty
        {
            public GUIContent Label { get; set; }
            public string SettingsPropName { get; set; }
        }

        class CheckToSkipRest : SettingsProperty 
        {
            public bool ValueToSkip { get; set; }
        }

        class CheckWithSizer : SettingsProperty
        {
            public string SizerFallbackName { get; set; }
            public SerializedProperty SizerFallback { get; set; }
            public SerializedProperty CustomSizers { get; set; }
        }

        class CheckWithProp : SettingsProperty
        {
            public string OtherSettingsPropertyName { get; set; }
        }

        string settingsName;
        SerializedObject serializedObject;
        SerializedProperty settingsFallback, settingsList;

        // key: fallback, value: customSizers
        List<SettingsProperty> allControls = new List<SettingsProperty>();

        public InjectedSettingsInspector(string settingsName, SerializedObject serializedObject,
            string settingsListName, string settingsFallbackName)
        {
            this.settingsName = settingsName;
            this.serializedObject = serializedObject;
            this.settingsList = serializedObject.FindProperty(settingsListName);
            this.settingsFallback = serializedObject.FindProperty(settingsFallbackName);
        }

        public void Register(string displayName, string settingsPropName)
        {
            var p = new SettingsProperty() 
            { 
                Label = new GUIContent(displayName),
                SettingsPropName = settingsPropName
            };

            allControls.Add(p);
        }

        public void Register(string displayName, string settingsBoolName, string otherSettingsPropertyName)
        {
            var p = new CheckWithProp()
            {
                Label = new GUIContent(displayName),
                SettingsPropName = settingsBoolName,
                OtherSettingsPropertyName = otherSettingsPropertyName
            };

            allControls.Add(p);
        }

        public void Register(string displayName, string settingsBoolName, string customSizersName, string sizerFallbackName)
        {
            var p = new CheckWithSizer()
            {
                Label = new GUIContent(displayName),
                SettingsPropName = settingsBoolName,
                SizerFallbackName = sizerFallbackName,
                CustomSizers = serializedObject.FindProperty(customSizersName),
                SizerFallback = serializedObject.FindProperty(sizerFallbackName),
            };

            allControls.Add(p);
        }

        public void RegisterSkipRest(string displayName, string settingsBoolName, bool valueToSkip)
        {
            var p = new CheckToSkipRest()
            {
                Label = new GUIContent(displayName),
                SettingsPropName = settingsBoolName, 
                ValueToSkip = valueToSkip 
            };

            allControls.Add(p);
        }

        public void RegisterSpace()
        {
            allControls.Add(null);
        }

        public void Draw()
        {
            ScreenConfigConnectionHelper.DrawGui("Settings", settingsList, ref settingsFallback, DrawSettings,
                AddSettings, DeleteSettings);
        }

        public void DrawSettings(string configName, SerializedProperty settings)
        {
            EditorGUILayout.BeginVertical("box");
            DrawControls(configName, settings);
            EditorGUILayout.EndVertical();
        }

        public void DrawControls(string configName, SerializedProperty settings)
        {
            foreach (var p in allControls)
            {
                if(p == null)
                {
                    EditorGUILayout.Space();
                    continue;
                }

                SerializedProperty prop = settings.FindPropertyRelative(p.SettingsPropName);
                if (p is CheckWithProp cwp)
                {
                    EditorGUILayout.BeginHorizontal();
                    EditorGUILayout.PropertyField(prop, p.Label);
                    if (prop.boolValue)
                    {
                        SerializedProperty other = settings.FindPropertyRelative(cwp.OtherSettingsPropertyName);
                        EditorGUILayout.PropertyField(other, GUIContent.none);
                    }
                    EditorGUILayout.EndHorizontal();
                }
                else
                {
                    EditorGUILayout.PropertyField(prop, p.Label);

                    if (p is CheckWithSizer cws)
                    {
                        if (prop.boolValue)
                        {
                            DrawSizer(configName, cws.SizerFallback, cws.CustomSizers);
                        }
                    }
                    else if (p is CheckToSkipRest csr)
                    {
                        if (prop.boolValue == csr.ValueToSkip)
                            return;

                        EditorGUILayout.Space();
                    }
                }
            }
        }

        private void DeleteSettings(string configName, SerializedProperty property)
        {
            foreach (var p in allControls.OfType<CheckWithSizer>())
            {
                int idx;
                SerializedProperty sizersProp = FindSizer(configName, null, p.CustomSizers, out idx);
                if (sizersProp != null)
                {
                    SerializedProperty items = p.CustomSizers.FindPropertyRelative("items");
                    items.DeleteArrayElementAtIndex(idx);
                }
            }
        }

        private void AddSettings(string configName, SerializedProperty property)
        {
            foreach (var p in allControls.OfType<CheckWithSizer>())
            {
                SerializedProperty sizersProp = FindSizer(configName, null, p.CustomSizers);
                if (sizersProp == null)
                {
                    SerializedProperty items = p.CustomSizers.FindPropertyRelative("items");
                    SerializedProperty fallback = p.SizerFallback;
                    ScreenConfigConnectionHelper.AddSizerToList(configName, ref fallback, items);

                    var configs = p.CustomSizers.GetValue<ISizeConfigCollection>();
                    configs.MarkDirty();
                }
            }

            // after adding the fallback values are pointing somewhere because of copying of all properties
            RestoreSizerFallbackReferences();
        }

        void RestoreSizerFallbackReferences()
        {
            foreach(var p in allControls.OfType<CheckWithSizer>())
            {
                p.SizerFallback = serializedObject.FindProperty(p.SizerFallbackName);
            }
        }


        static void DrawSizer(string configName, SerializedProperty fallback, SerializedProperty customSizers)
        {
            EditorGUI.indentLevel++;

            SerializedProperty prop = FindSizer(configName, fallback, customSizers);
            if (prop != null)
            {
                EditorGUILayout.PropertyField(prop);
            }
            else
            {
                EditorGUILayout.HelpBox(string.Format("could not find sizer for config '{0}'", configName), MessageType.Error);
            }

            EditorGUI.indentLevel--;
        }

        static SerializedProperty FindSizer(string configName, SerializedProperty fallback, SerializedProperty customSizers)
        {
            int idx;
            return FindSizer(configName, fallback, customSizers, out idx);
        }

        static SerializedProperty FindSizer(string configName, SerializedProperty fallback, SerializedProperty customSizers, out int sizerIndex)
        {
            bool isFallback = !(ResolutionMonitor.Instance.OptimizedScreens.Any(o => o.Name == configName));
            sizerIndex = -1;

            if (isFallback)
            {
                return fallback;
            }
            else
            {
                SerializedProperty items = customSizers.FindPropertyRelative("items");
                for (int i = 0; i < items.arraySize; i++)
                {
                    SerializedProperty prop = items.GetArrayElementAtIndex(i);
                    SerializedProperty propConfig = prop.FindPropertyRelative("screenConfigName");
                    if (propConfig.stringValue == configName)
                    {
                        sizerIndex = i;
                        return prop;
                    }
                }
            }

            return null;
        }

    }
}
