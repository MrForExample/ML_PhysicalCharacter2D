using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEditor;
using UnityEngine;

namespace TheraBytes.BetterUi
{
    public class AboutWindow : EditorWindow
    {
        const string VERSION = "2.5";

        [MenuItem("Tools/Better UI/Help/Documentation", false, 300)]
        public static void OpenDocumentation()
        {
            Application.OpenURL("https://documentation.therabytes.de/better-ui");
        }

        [MenuItem("Tools/Better UI/Help/Get Support (Forum)", false, 330)]
        public static void OpenForum()
        {
            Application.OpenURL("https://forum.unity3d.com/threads/better-ui.453808/");
        }

        [MenuItem("Tools/Better UI/Help/Get Support (Email)", false, 331)]
        public static void WriteMail()
        {
            Application.OpenURL("mailto:info@therabytes.de?subject=Better%20UI");
        }

        [MenuItem("Tools/Better UI/Help/Leave a Review", false, 360)]
        public static void OpenAssetStore()
        {
            Application.OpenURL("https://assetstore.unity.com/packages/tools/gui/better-ui-79031#reviews");
        }

        [MenuItem("Tools/Better UI/Help/About", false, 390)]
        public static void ShowWindow()
        {
            var win = EditorWindow.GetWindow(typeof(AboutWindow), true, "About");
            win.minSize = new Vector2(524, 400);
            win.maxSize = win.minSize;
        }


        GUIContent image;


        void OnEnable()
        {
            image = new GUIContent(Resources.Load<Texture2D>("wizard_banner"));
        }

        void OnGUI()
        {
            EditorGUILayout.Space();
            EditorGUILayout.LabelField(image, EditorStyles.wordWrappedLabel);
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Better UI", EditorStyles.boldLabel);
            EditorGUILayout.LabelField($"Version {VERSION}");
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("© 2023 Thera Bytes GmbH - All Rights Reserved", EditorStyles.miniBoldLabel);
            EditorGUILayout.LabelField("Created by Saloomon Zwecker", EditorStyles.miniLabel);
            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Standard Unity Asset Store End User License Agreement", EditorStyles.miniLabel);
        }
    }
}
