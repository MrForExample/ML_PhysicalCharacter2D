using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.UI;

namespace TheraBytes.BetterUi
{
#if UNITY_2018_3_OR_NEWER
    [ExecuteAlways]
#else
    [ExecuteInEditMode]
#endif
    [HelpURL("https://documentation.therabytes.de/better-ui/BetterAxisAlignedLayoutGroup.html")]
    [AddComponentMenu("Better UI/Layout/Better Axis Aligned Layout Group", 30)]
    public class BetterAxisAlignedLayoutGroup 
        : HorizontalOrVerticalLayoutGroup, IBetterHorizontalOrVerticalLayoutGroup, IResolutionDependency
    {

        [Serializable]
        public class Settings : IScreenConfigConnection
        {
            public TextAnchor ChildAlignment;
            
            public bool ReverseArrangement = false;

            public bool ChildForceExpandHeight = false;
            public bool ChildForceExpandWidth = false;

            public bool ChildScaleWidth = false;
            public bool ChildScaleHeight = false;

            public bool ChildControlWidth = true;
            public bool ChildControlHeight = true;

            public Axis Orientation;

            [SerializeField]
            string screenConfigName;
            public string ScreenConfigName { get { return screenConfigName; } set { screenConfigName = value; } }


            public Settings(TextAnchor childAlignment, bool expandWidth, bool expandHeight, Axis orientation)
            {
                this.ChildAlignment = childAlignment;
                this.ChildForceExpandWidth = expandWidth;
                this.ChildForceExpandHeight = expandHeight;
                this.Orientation = orientation;
            }
        }

        [Serializable]
        public class SettingsConfigCollection : SizeConfigCollection<Settings> { }

        public enum Axis
        {
            Horizontal,
            Vertical,
        }

        public MarginSizeModifier PaddingSizer { get { return customPaddingSizers.GetCurrentItem(paddingSizerFallback); } }
        public FloatSizeModifier SpacingSizer { get { return customSpacingSizers.GetCurrentItem(spacingSizerFallback); } }
        public Settings CurrentSettings { get { return customSettings.GetCurrentItem(settingsFallback); } }
        public Axis Orientation { get { return orientation; } set { orientation = value; } }
        bool isVertical { get { return orientation == Axis.Vertical; } }
        
        [SerializeField]
        MarginSizeModifier paddingSizerFallback =
            new MarginSizeModifier(new Margin(), new Margin(), new Margin(1000, 1000, 1000, 1000));

        [SerializeField]
        MarginSizeConfigCollection customPaddingSizers = new MarginSizeConfigCollection();
        
        [SerializeField]
        FloatSizeModifier spacingSizerFallback =
            new FloatSizeModifier(0, 0, 300);

        [SerializeField]
        FloatSizeConfigCollection customSpacingSizers = new FloatSizeConfigCollection();

        [SerializeField]
        Settings settingsFallback;

        [SerializeField]
        SettingsConfigCollection customSettings = new SettingsConfigCollection();

        [SerializeField]
        Axis orientation;

        #region new base setters
        public new RectOffset padding
        {
            get { return base.padding; }
            set { Config.Set(value, (o) => base.padding = value, (o) => PaddingSizer.SetSize(this, new Margin(o))); }
        }

        public new float spacing
        {
            get { return base.spacing; }
            set { Config.Set(value, (o) => base.spacing = value, (o) => SpacingSizer.SetSize(this, o)); }
        }
        public new TextAnchor childAlignment
        {
            get { return base.childAlignment; }
            set { Config.Set(value, (o) => base.childAlignment = o, (o) => CurrentSettings.ChildAlignment = o); }
        }


        public new bool childForceExpandHeight
        {
            get { return base.childForceExpandHeight; }
            set { Config.Set(value, (o) => base.childForceExpandHeight = o, (o) => CurrentSettings.ChildForceExpandHeight = o); }
        }

        public new bool childForceExpandWidth
        {
            get { return base.childForceExpandWidth; }
            set { Config.Set(value, (o) => base.childForceExpandWidth = o, (o) => CurrentSettings.ChildForceExpandWidth = o); }
        }

#if UNITY_2020_1_OR_NEWER
        public new bool reverseArrangement
        {
            get { return base.reverseArrangement; }
            set { Config.Set(value, (o) => base.reverseArrangement = o, (o) => CurrentSettings.ReverseArrangement = o); }
        }
#endif
#if UNITY_2019_1_OR_NEWER
        public new bool childScaleWidth
        {
            get { return base.childScaleWidth; }
            set { Config.Set(value, (o) => base.childScaleWidth = o, (o) => CurrentSettings.ChildScaleWidth = o); }
        }

        public new bool childScaleHeight
        {
            get { return base.childScaleHeight; }
            set { Config.Set(value, (o) => base.childScaleHeight = o, (o) => CurrentSettings.ChildScaleHeight = o); }
        }
#endif
#if !(UNITY_5_4) && !(UNITY_5_3)
        public new bool childControlWidth
        {
            get { return base.childControlWidth; }
            set { Config.Set(value, (o) => base.childControlWidth = o, (o) => CurrentSettings.ChildControlWidth = o); }
        }

        public new bool childControlHeight
        {
            get { return base.childControlHeight; }
            set { Config.Set(value, (o) => base.childControlHeight = o, (o) => CurrentSettings.ChildControlHeight = o); }
        }
#endif
#endregion


        protected override void OnEnable()
        {
            base.OnEnable();

            if (settingsFallback == null || string.IsNullOrEmpty(settingsFallback.ScreenConfigName))
            {
                StartCoroutine(InitDelayed());
            }
            else
            {
                CalculateCellSize();
            }
        }

        protected override void OnTransformChildrenChanged()
        {
            base.OnTransformChildrenChanged();
            
            if(isActiveAndEnabled)
            {
                StartCoroutine(SetDirtyDelayed());
            }
        }

        private IEnumerator SetDirtyDelayed()
        {
            yield return null;

            base.SetDirty();
        }

        protected override void OnRectTransformDimensionsChange()
        {
            base.OnRectTransformDimensionsChange();
            base.SetDirty();
        }
        

        IEnumerator InitDelayed()
        {
            yield return null;

            settingsFallback = new Settings(this.childAlignment, this.childForceExpandWidth, this.childForceExpandHeight, this.orientation)
            {
#if !(UNITY_5_4) && !(UNITY_5_3)
                ChildControlWidth = this.childControlWidth,
                ChildControlHeight = this.childControlHeight,
#endif
#if UNITY_2019_1_OR_NEWER
                ChildScaleWidth = this.childScaleWidth,
                ChildScaleHeight = this.childScaleHeight,
#endif
#if UNITY_2020_1_OR_NEWER
                ReverseArrangement = this.reverseArrangement,
#endif
                ScreenConfigName = "Fallback",
            };

            CalculateCellSize();
        }

        public override void CalculateLayoutInputHorizontal()
        {
            base.CalculateLayoutInputHorizontal();
            base.CalcAlongAxis(0, isVertical);
        }
        
        public override void CalculateLayoutInputVertical()
        {
            base.CalcAlongAxis(1, isVertical);
        }
        
        public override void SetLayoutHorizontal()
        {
            base.SetChildrenAlongAxis(0, isVertical);
        }
        
        public override void SetLayoutVertical()
        {
            base.SetChildrenAlongAxis(1, isVertical);
        }

        public void OnResolutionChanged()
        {
            CalculateCellSize();
        }

        public void CalculateCellSize()
        {
            Rect r = this.rectTransform.rect;
            if (r.width == float.NaN || r.height == float.NaN)
                return;

            ApplySettings(CurrentSettings);

            base.m_Spacing = SpacingSizer.CalculateSize(this);

            Margin pad = PaddingSizer.CalculateSize(this);
            pad.CopyValuesTo(base.m_Padding);

        }

        void ApplySettings(Settings settings)
        {
            if (settingsFallback == null)
                return;

            this.m_ChildAlignment = settings.ChildAlignment;
            this.orientation = settings.Orientation;
            this.m_ChildForceExpandWidth = settings.ChildForceExpandWidth;
            this.m_ChildForceExpandHeight = settings.ChildForceExpandHeight;

#if !(UNITY_5_4) && !(UNITY_5_3)
            this.m_ChildControlWidth = settings.ChildControlWidth;
            this.m_ChildControlHeight = settings.ChildControlHeight;
#endif
#if UNITY_2019_1_OR_NEWER
            this.childScaleWidth = settings.ChildScaleWidth;
            this.childScaleHeight = settings.ChildScaleHeight;
#endif
#if UNITY_2020_1_OR_NEWER
            this.reverseArrangement = settings.ReverseArrangement;
#endif
        }


#if UNITY_EDITOR
        protected override void OnValidate()
        {
            CalculateCellSize();
            base.OnValidate();
        }
#endif
        
    }
}
