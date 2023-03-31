using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEngine;
using UnityEngine.UI;

namespace TheraBytes.BetterUi.Editor
{
    [CustomEditor(typeof(BetterLayoutElement)), CanEditMultipleObjects]
    public class BetterLayoutElementEditor : UnityEditor.Editor
    {
        InjectedSettingsInspector helper;
        SerializedProperty layoutPriority;

        void OnEnable()   
        {
            helper = new InjectedSettingsInspector("Settings", serializedObject, "customSettings", "settingsFallback");
            helper.RegisterSkipRest("Ignore Layout", "IgnoreLayout", true);

            helper.Register("Min Width", "MinWidthEnabled", 
                "customMinWidthSizers", "minWidthSizerFallback");

            helper.Register("Min Height", "MinHeightEnabled",
                "customMinHeightSizers", "minHeightSizerFallback");

            helper.Register("Preferred Width", "PreferredWidthEnabled", 
                "customPreferredWidthSizers", "preferredWidthSizerFallback");

            helper.Register("Preferred Height", "PreferredHeightEnabled",
                "customPreferredHeightSizers", "preferredHeightSizerFallback");

            helper.Register("Flexible Width", "FlexibleWidthEnabled", "FlexibleWidth");
            helper.Register("Flexible Height", "FlexibleHeightEnabled", "FlexibleHeight");

            layoutPriority = serializedObject.FindProperty("m_LayoutPriority");
        }

        public override void OnInspectorGUI()
        {
            helper.Draw();

            if(layoutPriority != null) // Unity 2017+
            {
                EditorGUILayout.PropertyField(layoutPriority);
            }
        }

        [MenuItem("CONTEXT/LayoutElement/♠ Make Better")]
        public static void MakeBetter(MenuCommand command)
        {
            LayoutElement layout = command.context as LayoutElement;
            var ignore = layout.ignoreLayout;
            var minWidth = layout.minWidth;
            var minHeight = layout.minHeight;
            var prefWidth = layout.preferredWidth;
            var prefHeight = layout.preferredHeight;
            var flexWidth = layout.flexibleWidth;
            var flexHeight = layout.flexibleHeight;

            var newLayout = Betterizer.MakeBetter<LayoutElement, BetterLayoutElement>(layout) as BetterLayoutElement;
            if(newLayout != null)
            {
                newLayout.CurrentSettings.IgnoreLayout = ignore;

                newLayout.CurrentSettings.MinWidthEnabled = (minWidth >= 0);
                newLayout.CurrentSettings.MinHeightEnabled = (minHeight >= 0);

                newLayout.CurrentSettings.PreferredWidthEnabled = (prefWidth >= 0);
                newLayout.CurrentSettings.PreferredHeightEnabled = (prefHeight >= 0);

                newLayout.CurrentSettings.FlexibleWidthEnabled = (flexWidth >= 0);
                newLayout.CurrentSettings.FlexibleHeightEnabled = (flexHeight >= 0);


                newLayout.MinHeightSizer.SetSize(newLayout, minHeight);
                newLayout.MinWidthSizer.SetSize(newLayout, minWidth);

                newLayout.PreferredWidthSizer.SetSize(newLayout, prefWidth);
                newLayout.PreferredHeightSizer.SetSize(newLayout, prefHeight);

                newLayout.CurrentSettings.FlexibleWidth = flexWidth;
                newLayout.CurrentSettings.FlexibleHeight = flexHeight;
            }
        }
    }
}
