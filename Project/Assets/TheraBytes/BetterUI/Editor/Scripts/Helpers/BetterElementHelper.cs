using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEditor;
using UnityEngine;

namespace TheraBytes.BetterUi.Editor
{
    public class BetterElementHelper<TBase, TBetter>
        where TBase : MonoBehaviour
        where TBetter : TBase, IBetterTransitionUiElement
    {
        TransitionCollectionDrawer drawer;
        string fieldName;

        public BetterElementHelper(string fieldName = "betterTransitions")
        {
            this.fieldName = fieldName;
            this.drawer = new TransitionCollectionDrawer(typeof(TBetter), fieldName);
        }

        public void DrawGui(SerializedObject serializedObject)
        {
            drawer.Draw(() => serializedObject.FindProperty(fieldName));
        }


        [Obsolete(EditorGuiUtils.ObsoleteMessage)]
        public void DrawGui(SerializedObject serializedObject, UnityEngine.Object target)
        {
            EditorGuiUtils.DrawOldMethodCallWarning();

            DrawGui(serializedObject);
        }

    }
}
