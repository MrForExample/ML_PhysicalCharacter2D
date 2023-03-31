using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEditor.UI;
using UnityEditorInternal;
using UnityEngine;
using UnityEngine.UI;

namespace TheraBytes.BetterUi.Editor
{
    [CustomEditor(typeof(BetterInputField)), CanEditMultipleObjects]
    public class BetterInputFieldEditor : InputFieldEditor
    {
        BetterElementHelper<InputField, BetterInputField> helper =
            new BetterElementHelper<InputField, BetterInputField>();

        SerializedProperty additionalPlaceholdersProp;

        protected override void OnEnable()
        {
            base.OnEnable();
            additionalPlaceholdersProp = serializedObject.FindProperty("additionalPlaceholders");
        }


        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            helper.DrawGui(serializedObject);

            ThirdParty.ReorderableListGUI.Title("Additional Placeholders");
            ThirdParty.ReorderableListGUI.ListField(additionalPlaceholdersProp);

            serializedObject.ApplyModifiedProperties();
        }

        [MenuItem("CONTEXT/InputField/♠ Make Better")]
        public static void MakeBetter(MenuCommand command)
        {
            InputField obj = command.context as InputField;
            Betterizer.MakeBetter<InputField, BetterInputField>(obj);
        }
    }
}
