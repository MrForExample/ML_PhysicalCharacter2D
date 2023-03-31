using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Reflection;
using System.Text;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.UI;

namespace TheraBytes.BetterUi
{
#if UNITY_2018_3_OR_NEWER
    [ExecuteAlways]
#else
    [ExecuteInEditMode]
#endif
    [HelpURL("https://documentation.therabytes.de/better-ui/BetterContentSizeFitter.html")]
    [AddComponentMenu("Better UI/Layout/Better Content Size Fitter", 30)]
    public class BetterContentSizeFitter : ContentSizeFitter, IResolutionDependency, ILayoutChildDependency, ILayoutElement, ILayoutIgnorer
    {
        [Serializable]
        public class Settings : IScreenConfigConnection
        {
            public FitMode HorizontalFit;
            public FitMode VerticalFit;


            public bool IsAnimated;
            public float AnimationTime = 0.2f;

            public bool HasMinWidth;
            public bool HasMinHeight;

            public bool HasMaxWidth;
            public bool HasMaxHeight;

            [SerializeField]
            string screenConfigName;
            public string ScreenConfigName { get { return screenConfigName; } set { screenConfigName = value; } }
        }

        [Serializable]
        public class SettingsConfigCollection : SizeConfigCollection<Settings> { }

        RectTransform rectTransform { get { return this.transform as RectTransform; } }

        public Settings CurrentSettings { get { return customSettings.GetCurrentItem(settingsFallback); } }

        public RectTransform Source { get { return (source != null) ? source : this.rectTransform; } set { source = value; SetDirty(); } }
        public bool TreatAsLayoutElement { get { return treatAsLayoutElement; } set { treatAsLayoutElement = value; } }


        public FloatSizeModifier CurrentMinWidth { get { return minWidthSizers.GetCurrentItem(minWidthSizerFallback); } }
        public FloatSizeModifier CurrentMinHeight { get { return minHeightSizers.GetCurrentItem(minHeightSizerFallback); } }
        public FloatSizeModifier CurrentMaxWidth { get { return maxWidthSizers.GetCurrentItem(maxWidthSizerFallback); } }
        public Vector2SizeModifier CurrentPadding { get { return paddingSizers.GetCurrentItem(paddingFallback); } }

        public new FitMode horizontalFit
        {
            get { return base.horizontalFit; }
            set
            {
                Config.Set(value, (o) => base.horizontalFit = value, (o) => CurrentSettings.HorizontalFit = value);
            }
        }
        public new FitMode verticalFit
        {
            get { return base.verticalFit; }
            set
            {
                Config.Set(value, (o) => base.verticalFit = value, (o) => CurrentSettings.VerticalFit = value);
            }
        }

        [SerializeField]
        RectTransform source;

        [SerializeField]
        Settings settingsFallback = new Settings();

        [SerializeField]
        SettingsConfigCollection customSettings = new SettingsConfigCollection();

        [SerializeField]
        FloatSizeModifier minWidthSizerFallback = new FloatSizeModifier(0, 0, 4000);
        [SerializeField]
        FloatSizeConfigCollection minWidthSizers = new FloatSizeConfigCollection();


        [SerializeField]
        FloatSizeModifier minHeightSizerFallback = new FloatSizeModifier(0, 0, 4000);
        [SerializeField]
        FloatSizeConfigCollection minHeightSizers = new FloatSizeConfigCollection();

        [SerializeField]
        FloatSizeModifier maxWidthSizerFallback = new FloatSizeModifier(1000, 0, 4000);
        [SerializeField]
        FloatSizeConfigCollection maxWidthSizers = new FloatSizeConfigCollection();


        [SerializeField]
        FloatSizeModifier maxHeightSizerFallback = new FloatSizeModifier(1000, 0, 4000);
        [SerializeField]
        FloatSizeConfigCollection maxHeightSizers = new FloatSizeConfigCollection();


        [SerializeField]
        Vector2SizeModifier paddingFallback = new Vector2SizeModifier(new Vector2(), new Vector2(-5000, -5000), new Vector2(5000, 5000));
        [SerializeField]
        Vector2SizeConfigCollection paddingSizers = new Vector2SizeConfigCollection();

        [SerializeField]
        bool treatAsLayoutElement = true;

        RectTransformData start = new RectTransformData();
        RectTransformData end = new RectTransformData();

        bool isAnimating;
        Vector2 lastCalculatedSize;

        protected override void OnEnable()
        {
            base.OnEnable();
            Apply();
        }

        protected override void OnDisable()
        {
            base.OnDisable();
            isAnimating = false;
        }

        public void OnResolutionChanged()
        {
            Apply();
        }

        void Apply()
        {
            Settings settings = CurrentSettings;
            base.m_HorizontalFit = settings.HorizontalFit;
            base.m_VerticalFit = settings.VerticalFit;

            SetDirty();
        }

        public override void SetLayoutHorizontal()
        {
            SetLayout(0);
        }

        public override void SetLayoutVertical()
        {
            SetLayout(1);
        }

        void SetLayout(int axis)
        {
            if (axis == 0 && CurrentSettings.HorizontalFit == FitMode.Unconstrained)
                return;

            if (axis == 1 && CurrentSettings.VerticalFit == FitMode.Unconstrained)
                return;

            if (isAnimating)
                return;


            if (CurrentSettings.IsAnimated)
            {
                start.PullFromTransform(this.transform as RectTransform);
            }

            // disable layout element functionality to prevent wrong size calculation for itself.
            bool wasLayoutElement = this.treatAsLayoutElement;
            this.treatAsLayoutElement = false;

            if (axis == 0)
            {
                base.SetLayoutHorizontal();
            }
            else
            {
                base.SetLayoutVertical();
            }

            ApplyOffsetToDefaultSize(axis, (axis == 0) ? m_HorizontalFit : m_VerticalFit);

            if (CurrentSettings.IsAnimated)
            {
                end.PullFromTransform(this.transform as RectTransform);
                start.PushToTransform(this.transform as RectTransform);

                Animate();
            }

            // restore layout element functionality to prevent wrong size calculation for parent layout groups.
            this.treatAsLayoutElement = wasLayoutElement;
        }

        void ApplyOffsetToDefaultSize(int axis, FitMode fitMode)
        {
            Vector2 padding = paddingSizers.GetCurrentItem(paddingFallback).CalculateSize(this);
            bool hasMax = (axis == 0) ? CurrentSettings.HasMaxWidth : CurrentSettings.HasMaxHeight;
            bool hasMin = (axis == 0) ? CurrentSettings.HasMinWidth : CurrentSettings.HasMinHeight;

            if (hasMax || hasMin || !Mathf.Approximately(padding[axis], 0) || source != null)
            {

                float size = (fitMode == FitMode.MinSize)
                        ? LayoutUtility.GetMinSize(Source, axis)
                        : LayoutUtility.GetPreferredSize(Source, axis);


                size += padding[axis];

                size = ClampSize((RectTransform.Axis)axis, size);

                lastCalculatedSize[axis] = size;
                rectTransform.SetSizeWithCurrentAnchors((RectTransform.Axis)axis, size);
            }
        }


        float ClampSize(RectTransform.Axis axis, float size)
        {
            switch (axis)
            {
                case RectTransform.Axis.Horizontal:

                    if (CurrentSettings.HasMinWidth)
                    {
                        size = Mathf.Max(size, minWidthSizers.GetCurrentItem(minWidthSizerFallback).CalculateSize(this));
                    }

                    if (CurrentSettings.HasMaxWidth)
                    {
                        size = Mathf.Min(size, maxWidthSizers.GetCurrentItem(maxWidthSizerFallback).CalculateSize(this));
                    }
                    break;

                case RectTransform.Axis.Vertical:

                    if (CurrentSettings.HasMinHeight)
                    {
                        size = Mathf.Max(size, minHeightSizers.GetCurrentItem(minHeightSizerFallback).CalculateSize(this));
                    }

                    if (CurrentSettings.HasMaxHeight)
                    {
                        size = Mathf.Min(size, maxHeightSizers.GetCurrentItem(maxHeightSizerFallback).CalculateSize(this));
                    }
                    break;
            }

            return size;
        }

        Bounds GetChildBounds()
        {
            RectTransform rt = this.transform as RectTransform;
            Bounds bounds = new Bounds();
            for (int i = 0; i < transform.childCount; i++)
            {
                Transform child = transform.GetChild(i);
                if (!(child.gameObject.activeSelf))
                    continue;

                Bounds b = RectTransformUtility.CalculateRelativeRectTransformBounds(rt, child);
                bounds.Encapsulate(b);
            }

            return bounds;
        }

        private void Animate()
        {
            if (!CurrentSettings.IsAnimated)
                return;


            this.StopAllCoroutines();

            StartCoroutine(CoAnimate());
        }

        private IEnumerator CoAnimate()
        {
            float t = 0;
            isAnimating = true;

            yield return null;

            while (t < CurrentSettings.AnimationTime)
            {
                t += Time.unscaledDeltaTime;
                float amount = Mathf.SmoothStep(0, 1, t / CurrentSettings.AnimationTime);
                RectTransformData data = RectTransformData.Lerp(start, end, amount);
                data.PushToTransform(this.transform as RectTransform);

                yield return null;
            }

            end.PushToTransform(this.transform as RectTransform);

            isAnimating = false;

            // In case that we missed something during animation
            // simply apply the changes without animation
            CurrentSettings.IsAnimated = false;
            SetLayoutHorizontal();
            SetLayoutVertical();
            CurrentSettings.IsAnimated = true;
        }

        #region ILayoutChildDependency

        public void ChildSizeChanged(Transform child)
        {
            ChildChanged();
        }

        public void ChildAddedOrEnabled(Transform child)
        {
            ChildChanged();
        }

        public void ChildRemovedOrDisabled(Transform child)
        {
            ChildChanged();
        }

        void ChildChanged()
        {
            bool tmp = CurrentSettings.IsAnimated;
            CurrentSettings.IsAnimated = false;
            SetLayoutHorizontal();
            SetLayoutVertical();
            CurrentSettings.IsAnimated = tmp;
        }


        #endregion

        #region ILayoutElement & ILayoutIgnorer
        float ILayoutElement.minWidth
        {
            get { return treatAsLayoutElement && CurrentSettings.HasMinWidth ? CurrentMinWidth.LastCalculatedSize : -1; }
        }
        float ILayoutElement.minHeight
        {
            get { return treatAsLayoutElement && CurrentSettings.HasMinHeight ? CurrentMinHeight.LastCalculatedSize : -1; }
        }


        float ILayoutElement.preferredWidth
        {
            get
            {
                if (!treatAsLayoutElement)
                    return -1;

                SetLayoutHorizontal();
                return lastCalculatedSize.x;
            }
        }

        float ILayoutElement.preferredHeight
        {
            get
            {
                if (!treatAsLayoutElement)
                    return -1;

                SetLayoutVertical();
                return lastCalculatedSize.y;
            }
        }

        float ILayoutElement.flexibleWidth { get { return -1; } }
        float ILayoutElement.flexibleHeight { get { return -1; } }

        int ILayoutElement.layoutPriority { get { return 1; } }

        bool ILayoutIgnorer.ignoreLayout { get { return !treatAsLayoutElement; } }

        void ILayoutElement.CalculateLayoutInputHorizontal()
        {
            SetLayoutHorizontal();
        }

        void ILayoutElement.CalculateLayoutInputVertical()
        {
            SetLayoutVertical();
        }
        #endregion

#if UNITY_EDITOR
        protected override void OnValidate()
        {
            base.OnValidate();
            Apply();
        }


#endif

    }

}
