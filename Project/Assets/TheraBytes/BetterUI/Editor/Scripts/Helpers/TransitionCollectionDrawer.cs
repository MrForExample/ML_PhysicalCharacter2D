using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using System.Threading.Tasks;
using TheraBytes.BetterUi.Editor.ThirdParty;
using UnityEditor;
using UnityEditorInternal;
using UnityEngine;

namespace TheraBytes.BetterUi.Editor
{
#if ODIN_INSPECTOR
    [Sirenix.OdinInspector.Editor.DrawerPriority(0, 0, 1)]
#endif
    [CustomPropertyDrawer(typeof(List<Transitions>))]
    public class TransitionCollectionDrawer : PropertyDrawer
    {
        string[] stateNames;
        SerializedProperty listProperty;
        ReorderableListControl listEditor;
        IReorderableListAdaptor adaptor;

        public TransitionCollectionDrawer() { }
        public TransitionCollectionDrawer(Type containerObjectType, 
            string fieldName = "betterTransitions",
            BindingFlags bindingFlags = BindingFlags.NonPublic | BindingFlags.Instance)
        {
            var fieldInfo = containerObjectType.GetField(fieldName, bindingFlags);
            TryReadStateNames(fieldInfo);
        }

        public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
        {

            TryReadStateNames(base.fieldInfo);
            Draw(() => property);
        }

        public void Draw(Func<SerializedProperty> getProperty)
        {
            if(this.listProperty == null)
            {
                this.listProperty = getProperty();
            }

            if (adaptor == null)
            {
                adaptor = new SerializedPropertyAdaptor(listProperty);
            }

            if(stateNames == null)
            {
                var msg = string.Format(
                        "No [TransitionStates(...)] or [DefaultTransitionStates] attribute defined for field <b>{0}</b>",
                        listProperty.displayName);

                EditorGUILayout.HelpBox(msg, MessageType.Error);
                return;
            }

            if (listEditor == null)
            {
                listEditor = new ReorderableListControl(ReorderableListFlags.ShowSizeField);
                listEditor.ItemInserted += ItemInserted;
            }

            ReorderableListGUI.Title(listProperty.displayName);
            listEditor.Draw(adaptor);
            listProperty.serializedObject.ApplyModifiedProperties();
        }

        private bool TryReadStateNames(FieldInfo fieldInfo)
        {
            if (stateNames != null)
                return true;

            var attribute = fieldInfo.GetCustomAttributes(typeof(TransitionStatesAttribute), true)
                    .FirstOrDefault() as TransitionStatesAttribute;

            if (attribute == null)
                return false;

            stateNames = attribute.States;
            return true;
        }

        private void ItemInserted(object sender, ItemInsertedEventArgs args)
        {
            var p = listProperty.GetArrayElementAtIndex(args.ItemIndex);
            var namesProp = p.FindPropertyRelative("stateNames");

            // unity copies the previous content.
            // so we need to fill the array only the first time.
            if (namesProp.arraySize >= stateNames.Length)
                return;

            for (int i = 0; i < stateNames.Length; i++)
            {
                namesProp.InsertArrayElementAtIndex(i);
                namesProp.serializedObject.ApplyModifiedPropertiesWithoutUndo();
                string name = stateNames[i];
                var elem = namesProp.GetArrayElementAtIndex(i);
                elem.stringValue = name;
            }

            namesProp.serializedObject.ApplyModifiedPropertiesWithoutUndo();
        }
    }
}
