using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.Serialization;
using UnityEngine.UI;

namespace TheraBytes.BetterUi
{
#if UNITY_2018_3_OR_NEWER
    [ExecuteAlways]
#else
    [ExecuteInEditMode]
#endif
    [HelpURL("https://documentation.therabytes.de/better-ui/BetterOffsetter.html")]
    [RequireComponent(typeof(RectTransform))]
    [AddComponentMenu("Better UI/Layout/Better Offsetter", 30)]
    public class BetterOffsetter : UIBehaviour, ILayoutController, ILayoutSelfController, IResolutionDependency
    {
        [Serializable]
        public class Settings : IScreenConfigConnection
        {
            public bool ApplyPosX { get { return applyPosX; } set { applySizeX = value; } }
            public bool ApplyPosY { get { return applyPosY; } set { applyPosY = value; } }
            public bool ApplySizeX{ get { return applySizeX; } set { applySizeX = value; } }
            public bool ApplySizeY { get { return applySizeY; } set { applySizeY = value; } }

            [SerializeField]
            bool applyPosX = false;

            [SerializeField]
            bool applyPosY = false;

            [SerializeField]
            bool applySizeX = false;

            [SerializeField]
            bool applySizeY = false;

            [SerializeField]
            string screenConfigName;
            public string ScreenConfigName { get { return screenConfigName; } set { screenConfigName = value; } }
        }

        [Serializable]
        public class SettingsConfigCollection : SizeConfigCollection<Settings> { }
        public Settings CurrentSettings { get { return customSettings.GetCurrentItem(settingsFallback); } }

        [SerializeField]
        Settings settingsFallback = new Settings();

        [SerializeField]
        SettingsConfigCollection customSettings = new SettingsConfigCollection();


        public FloatSizeModifier AnchoredPositionXSizer 
        {
            get { return customAnchorPosXSizers.GetCurrentItem(anchorPosXSizerFallback); }
        }

        public FloatSizeModifier AnchoredPositionYSizer
        {
            get { return customAnchorPosYSizers.GetCurrentItem(anchorPosYSizerFallback); }
        }


        public FloatSizeModifier SizeDeltaXSizer 
        { 
            get { return customSizeDeltaXSizers.GetCurrentItem(sizeDeltaXSizerFallback); } 
        }

        public FloatSizeModifier SizeDeltaYSizer
        {
            get { return customSizeDeltaYSizers.GetCurrentItem(sizeDeltaYSizerFallback); }
        }

        [SerializeField] FloatSizeModifier anchorPosXSizerFallback = new FloatSizeModifier(100, 0, 1000);
        [SerializeField] FloatSizeConfigCollection customAnchorPosXSizers = new FloatSizeConfigCollection();

        [SerializeField] FloatSizeModifier anchorPosYSizerFallback = new FloatSizeModifier(100, 0, 1000);
        [SerializeField] FloatSizeConfigCollection customAnchorPosYSizers = new FloatSizeConfigCollection();
        
        [SerializeField] FloatSizeModifier sizeDeltaXSizerFallback = new FloatSizeModifier(100, 0, 1000);
        [SerializeField] FloatSizeConfigCollection customSizeDeltaXSizers = new FloatSizeConfigCollection();

        [SerializeField] FloatSizeModifier sizeDeltaYSizerFallback = new FloatSizeModifier(100, 0, 1000);
        [SerializeField] FloatSizeConfigCollection customSizeDeltaYSizers = new FloatSizeConfigCollection();


        DrivenRectTransformTracker rectTransformTracker = new DrivenRectTransformTracker();

        protected override void OnEnable()
        {
            base.OnEnable();
            ApplySize();
        }

        protected override void OnDisable()
        {
            base.OnDisable();
            rectTransformTracker.Clear();
        }

#if UNITY_EDITOR
        protected override void OnValidate()
        {
            base.OnValidate();

            // NOTE: Unity sends a message when setting RectTransform.sizeDelta which is required here.
            //       This logs a warning: "SendMessage cannot be called during Awake, CheckConsistency, or OnValidate"
            //       However, everything seems to work anyway. Seems like there is no easy way to work around this problem.
            ApplySize();
        }
#endif

        void ApplySize()
        {
            if (!isActiveAndEnabled)
                return;

            RectTransform rt = (this.transform as RectTransform);
            Vector2 pos = rt.anchoredPosition;
            Vector2 size = rt.sizeDelta;

            Settings settings = CurrentSettings;
            rectTransformTracker.Clear();

            if (settings.ApplySizeX)
            {
                size.x = SizeDeltaXSizer.CalculateSize(this);
                rectTransformTracker.Add(this, this.transform as RectTransform, DrivenTransformProperties.SizeDeltaX);
            }

            if (settings.ApplySizeY)
            {
                size.y = SizeDeltaYSizer.CalculateSize(this);
                rectTransformTracker.Add(this, this.transform as RectTransform, DrivenTransformProperties.SizeDeltaY);
            }

            if (settings.ApplyPosX)
            {
                pos.x = AnchoredPositionXSizer.CalculateSize(this);
                rectTransformTracker.Add(this, this.transform as RectTransform, DrivenTransformProperties.AnchoredPositionX);
            }

            if (settings.ApplyPosY)
            {
                pos.y = AnchoredPositionYSizer.CalculateSize(this);
                rectTransformTracker.Add(this, this.transform as RectTransform, DrivenTransformProperties.AnchoredPositionY);
            }

            rt.anchoredPosition = pos;
            rt.sizeDelta = size;
        }

        public void OnResolutionChanged()
        {
            ApplySize();
        }

        public void SetLayoutHorizontal()
        {
            ApplySize();
        }

        public void SetLayoutVertical()
        {
            ApplySize();
        }

        protected override void OnRectTransformDimensionsChange()
        {
            base.OnRectTransformDimensionsChange();
            ApplySize();
        }
    }
}
