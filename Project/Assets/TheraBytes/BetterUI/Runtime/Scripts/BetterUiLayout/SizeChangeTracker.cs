using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

namespace TheraBytes.BetterUi
{
    [HelpURL("https://documentation.therabytes.de/better-ui/SizeChangeTracker.html")]
    [AddComponentMenu("Better UI/Helpers/Size Change Tracker", 30)]
    [RequireComponent(typeof(RectTransform))]
    [RequireComponent(typeof(ILayoutElement))]
    public class SizeChangeTracker : UIBehaviour, ILayoutSelfController
    {
        [Flags]
        public enum Axis
        {
            None = 0,
            Horizontal = 1 << 0,
            Vertical = 1 << 1,
            HorizontalAndVertical = Horizontal | Vertical,
        }

        public enum TrackMode
        {
            PreferredSize,
            MinSize,
        }

        public TrackMode SizeMode = TrackMode.PreferredSize;
        public Axis AffectedAxis = Axis.HorizontalAndVertical;
        public RectTransform[] AffectedObjects;

        bool isInRecursion = false;
        Vector2 previousSize;

        RectTransform rectTransform;
        public RectTransform RectTransform
        {
            get
            {
                if (rectTransform == null)
                {
                    rectTransform = transform as RectTransform;
                }

                return rectTransform;
            }
        }

        protected override void OnEnable()
        {
            base.OnEnable();

            CallForAffectedObjects((dp) => dp.ChildAddedOrEnabled(this.transform), force: true);
        }

        protected override void OnDisable()
        {
            base.OnDisable();

            CallForAffectedObjects((dp) => dp.ChildRemovedOrDisabled(this.transform), force: true);
            previousSize = Vector2.zero;
        }

        protected override void OnTransformParentChanged()
        {
            base.OnTransformParentChanged();
            CallForAffectedObjects((dp) => dp.ChildRemovedOrDisabled(this.transform), force: true);
            previousSize = Vector2.zero;
        }

        protected override void OnRectTransformDimensionsChange()
        {
            base.OnRectTransformDimensionsChange();
            CallForAffectedObjects((dp) => dp.ChildSizeChanged(this.transform));
        }

        void ILayoutController.SetLayoutHorizontal()
        {
            if (!AffectedAxis.HasFlag(Axis.Horizontal))
                return;

            CallForAffectedObjects((dp) => dp.ChildSizeChanged(this.transform));
        }

        void ILayoutController.SetLayoutVertical()
        {
            if (!AffectedAxis.HasFlag(Axis.Vertical))
                return;

            CallForAffectedObjects((dp) => dp.ChildSizeChanged(this.transform));
        }


        private void CallForAffectedObjects(Action<ILayoutChildDependency> function, bool force = false)
        {
            if (function == null)
                throw new ArgumentNullException("function must not be null");

            if (isInRecursion)
                return;

            Vector2 size = GetCurrentSize();
            if (!force && !HasSizeChanged(size, previousSize))
                return;

            previousSize = size;
            isInRecursion = true;

            try
            {
                foreach (RectTransform rt in AffectedObjects)
                {
                    if (rt == null)
                        continue;

                    foreach (ILayoutChildDependency dp in rt.GetComponents<ILayoutChildDependency>())
                    {
                        if (dp == null)
                            continue;

                        function(dp);
                    }
                }
            }
            catch (Exception ex)
            {
                Debug.LogException(ex);
            }
            finally
            {
                isInRecursion = false;
            }

        }

        private Vector2 GetCurrentSize()
        {
            switch (SizeMode)
            {
                case TrackMode.PreferredSize:

                    return new Vector2(
                        LayoutUtility.GetPreferredWidth(RectTransform),
                        LayoutUtility.GetPreferredHeight(RectTransform));

                case TrackMode.MinSize:

                    return new Vector2(
                        LayoutUtility.GetMinWidth(RectTransform),
                        LayoutUtility.GetMinHeight(RectTransform));

                default: throw new ArgumentException();
            }
        }

        private bool HasSizeChanged(Vector2 size, Vector2 previousSize)
        {
            switch (AffectedAxis)
            {
                case Axis.None: return false;
                case Axis.Horizontal: return size.x != previousSize.x;
                case Axis.Vertical: return size.y != previousSize.y;
                case Axis.HorizontalAndVertical: return size != previousSize;
                default: throw new ArgumentException();
            }
        }
    }
}
