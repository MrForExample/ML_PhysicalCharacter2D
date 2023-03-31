using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEngine;

namespace TheraBytes.BetterUi.Editor
{
    public static class EditorGuiUtils
    {
        public const string ObsoleteMessage = "This method is obsolete.You most probably need to update the Better TextMesh Pro package. Please check the upgrade guide for more information: https://documentation.therabytes.de/better-ui/UpgradeGuide.html ";

        public static void DrawOldMethodCallWarning()
        {
            EditorGUILayout.HelpBox(@"Calling an old method. You probably need to update the Better TextMesh Pro package.

Please install the right 'BetterUI_TextMeshPro' package found at 'Assets/TheraBytes/BetterUI/Packages' (probably you need the package that ends with '_EditorPanelUI').

For more information, please read the upgrade guide.", MessageType.Warning);


            if (GUILayout.Button("Open Upgrade Guide", "minibutton"))
            {
                Application.OpenURL("https://documentation.therabytes.de/better-ui/UpgradeGuide.html");
            }

            EditorGUILayout.Space();
        }

        #region Backwards Compatibility Methods
        [Obsolete(ObsoleteMessage)]
        public static void DrawLayoutList<T>(string listTitle,
            List<T> list, SerializedProperty listProp, ref int count, ref bool foldout,
            Action<SerializedProperty> createCallback, Action<T, SerializedProperty> drawItemCallback)
        {
            DrawOldMethodCallWarning();
        }

        [Obsolete(ObsoleteMessage)]
        public static void DrawLayoutList<T>(string listTitle,
            List<T> list, SerializedProperty listProp, ref int count,
            Action<SerializedProperty> createCallback, Action<T, SerializedProperty> drawItemCallback)
        {
            DrawOldMethodCallWarning();
        }

        [Obsolete(ObsoleteMessage)]
        public static void DrawLayoutList<T>(string listTitle, bool usingFoldout,
            List<T> list, SerializedProperty listProp, ref int count, ref bool foldout,
            Action<SerializedProperty> createCallback, Action<T, SerializedProperty> drawItemCallback)
        {
            DrawOldMethodCallWarning();
        }

        [Obsolete(ObsoleteMessage)]
        public static void DrawTransitions(string title,
            List<Transitions> transitions, SerializedProperty transitionsProp, ref int count,
            params string[] stateNames)
        {
            DrawOldMethodCallWarning();
        }
        #endregion
    }
}
