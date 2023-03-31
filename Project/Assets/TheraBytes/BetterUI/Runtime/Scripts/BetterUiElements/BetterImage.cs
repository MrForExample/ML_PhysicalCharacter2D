using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.Linq;
using System.Text;
using UnityEngine;
using UnityEngine.Serialization;
using UnityEngine.Sprites;
using UnityEngine.UI;

namespace TheraBytes.BetterUi
{
    public enum ColorMode { Color, HorizontalGradient, VerticalGradient, }

#if UNITY_2018_3_OR_NEWER
    [ExecuteAlways]
#else
    [ExecuteInEditMode]
#endif
    [HelpURL("https://documentation.therabytes.de/better-ui/BetterImage.html")]
    [AddComponentMenu("Better UI/Controls/Better Image", 30)]
    public class BetterImage : Image, IResolutionDependency, IImageAppearanceProvider
    {
        static readonly Vector2[] vertScratch = new Vector2[4];
        static readonly Vector2[] uvScratch = new Vector2[4];

        #region Nested Types
        [Serializable]
        public class SpriteSettings : IScreenConfigConnection
        {
            public Sprite Sprite;
            public ColorMode ColorMode;
            public Color PrimaryColor;
            public Color SecondaryColor;

            [SerializeField]
            string screenConfigName;
            public string ScreenConfigName { get { return screenConfigName; } set { screenConfigName = value; } }


            public SpriteSettings(Sprite sprite, ColorMode colorMode, Color primary, Color secondary)
            {
                this.Sprite = sprite;
                this.ColorMode = colorMode;
                this.PrimaryColor = primary;
                this.SecondaryColor = secondary;

            }
        }

        [Serializable]
        public class SpriteSettingsConfigCollection : SizeConfigCollection<SpriteSettings> { }
        #endregion

#if !UNITY_2019_1_OR_NEWER
        float multipliedPixelsPerUnit => base.pixelsPerUnit;
#endif

        public bool KeepBorderAspectRatio
        {
            get { return keepBorderAspectRatio; }
            set { keepBorderAspectRatio = value; SetVerticesDirty(); }
        }

        public Vector2SizeModifier SpriteBorderScale { get { return customBorderScales.GetCurrentItem(spriteBorderScaleFallback); } }

        public string MaterialType
        {
            get { return materialType; }
            set { ImageAppearanceProviderHelper.SetMaterialType(value, this, materialProperties, ref materialEffect, ref materialType); }
        }

        public MaterialEffect MaterialEffect
        {
            get { return materialEffect; }
            set { ImageAppearanceProviderHelper.SetMaterialEffect(value, this, materialProperties, ref materialEffect, ref materialType); }
        }

        public VertexMaterialData MaterialProperties { get { return materialProperties; } }

        public ColorMode ColoringMode 
        { 
            get { return colorMode; }
            set
            {
                Config.Set(value, (o) => colorMode = value, (o) => CurrentSpriteSettings.ColorMode = value);
                SetVerticesDirty();
            } 
        }

        public Color SecondColor 
        { 
            get { return secondColor; }
            set
            {
                Config.Set(value, (o) => secondColor = value, (o) => CurrentSpriteSettings.SecondaryColor = value);
                SetVerticesDirty(); 
            } 
        }

        public override Color color
        {
            get { return base.color; }
            set
            {
                Config.Set(value, (o) => base.color = value, (o) => CurrentSpriteSettings.PrimaryColor = value);
            }
        }

        public new Sprite sprite
        {
            get { return base.sprite; }
            set
            {
                Config.Set(value, (o) => base.sprite = value, (o) => CurrentSpriteSettings.Sprite = value);
            }
        }

        [SerializeField]
        ColorMode colorMode = ColorMode.Color;

        [SerializeField]
        Color secondColor = Color.white;


        [SerializeField]
        VertexMaterialData materialProperties = new VertexMaterialData();

        [SerializeField]
        string materialType;

        [SerializeField]
        MaterialEffect materialEffect;

        [SerializeField]
        float materialProperty1, materialProperty2, materialProperty3;

        [SerializeField]
        bool keepBorderAspectRatio;

        [FormerlySerializedAs("spriteBorderScale")]
        [SerializeField]
        Vector2SizeModifier spriteBorderScaleFallback =
            new Vector2SizeModifier(Vector2.one, Vector2.zero, 3 * Vector2.one);

        [SerializeField]
        Vector2SizeConfigCollection customBorderScales = new Vector2SizeConfigCollection();

        [SerializeField]
        SpriteSettings fallbackSpriteSettings;

        [SerializeField]
        SpriteSettingsConfigCollection customSpriteSettings = new SpriteSettingsConfigCollection();

        public SpriteSettings CurrentSpriteSettings
        {
            get
            {
                DoValidation();
                return customSpriteSettings.GetCurrentItem(fallbackSpriteSettings);
            } 
        }
        
        Animator animator;

        private Sprite activeSprite => (overrideSprite != null) ? overrideSprite : sprite;

        protected override void OnEnable()
        {
            base.OnEnable();

            AssignSpriteSettings();

            animator = GetComponent<Animator>();

            if (MaterialProperties.FloatProperties != null)
            {
                if (MaterialProperties.FloatProperties.Length > 0)
                    materialProperty1 = MaterialProperties.FloatProperties[0].Value;

                if (MaterialProperties.FloatProperties.Length > 1)
                    materialProperty2 = MaterialProperties.FloatProperties[1].Value;

                if (MaterialProperties.FloatProperties.Length > 2)
                    materialProperty3 = MaterialProperties.FloatProperties[2].Value;
            }
        }

        public float GetMaterialPropertyValue(int propertyIndex)
        {
            return ImageAppearanceProviderHelper.GetMaterialPropertyValue(propertyIndex,
                ref materialProperty1, ref materialProperty2, ref materialProperty3);
        }

        public void SetMaterialProperty(int propertyIndex, float value)
        {
            ImageAppearanceProviderHelper.SetMaterialProperty(propertyIndex, value, this, materialProperties, 
                ref materialProperty1, ref materialProperty2, ref materialProperty3);
        }

        protected override void OnPopulateMesh(VertexHelper toFill)
        { 
            if (animator != null 
                && MaterialProperties.FloatProperties != null 
                && (Application.isPlaying == animator.isActiveAndEnabled))
            {
                if (MaterialProperties.FloatProperties.Length > 0)
                    MaterialProperties.FloatProperties[0].Value = materialProperty1;

                if (MaterialProperties.FloatProperties.Length > 1)
                    MaterialProperties.FloatProperties[1].Value = materialProperty2;

                if (MaterialProperties.FloatProperties.Length > 2)
                    MaterialProperties.FloatProperties[2].Value = materialProperty3;
            }

            if (this.activeSprite == null)
            {
                GenerateSimpleSprite(toFill, false);
                return;
            }

            switch (type)
            {
                case Type.Simple:
                    GenerateSimpleSprite(toFill, base.preserveAspect);
                    break;

                case Type.Sliced:
                    GenerateSlicedSprite(toFill);
                    break;

                case Type.Tiled:
                    GenerateTiledSprite(toFill);
                    break;

                case Type.Filled:
                default:
                    base.OnPopulateMesh(toFill);
                    break;
            }
        }

        #region Simple
        private void GenerateSimpleSprite(VertexHelper vh, bool preserveAspect)
        {
            Rect rect = GetDrawingRect(preserveAspect);
            Vector4 uv = (activeSprite == null)
                ? Vector4.zero
                : DataUtility.GetOuterUV(activeSprite);

            vh.Clear();
            AddQuad(vh, rect,
                rect.min, rect.max,
                colorMode, color, secondColor,
                new Vector2(uv.x, uv.y), new Vector2(uv.z, uv.w));
        }

        private Rect GetDrawingRect(bool shouldPreserveAspect)
        {
            Vector4 padding = (activeSprite == null) ? Vector4.zero : DataUtility.GetPadding(activeSprite);
            Vector2 spriteSize = (activeSprite == null) ? Vector2.zero : new Vector2(activeSprite.rect.width, activeSprite.rect.height);
            Rect rect = GetPixelAdjustedRect();

            if (activeSprite == null)
                return rect;

            float w = Mathf.RoundToInt(spriteSize.x);
            float h = Mathf.RoundToInt(spriteSize.y);
            Vector4 paddingFraction = new Vector4(
                padding.x / w,
                padding.y / h,
                (w - padding.z) / w,
                (h - padding.w) / h);

            if (shouldPreserveAspect && spriteSize.sqrMagnitude > 0f)
            {
                PreserveSpriteAspectRatio(ref rect, spriteSize);
            }

            var v = new Vector4(
                rect.x + rect.width * paddingFraction.x,
                rect.y + rect.height * paddingFraction.y,
                rect.x + rect.width * paddingFraction.z,
                rect.y + rect.height * paddingFraction.w);

            return new Rect(v.x, v.y, v.z - v.x, v.w - v.y);
        }

        private void PreserveSpriteAspectRatio(ref Rect rect, Vector2 spriteSize)
        {
            float spriteAspect = spriteSize.x / spriteSize.y;
            float rectAspect = rect.width / rect.height;
            if (spriteAspect > rectAspect)
            {
                float height = rect.height;
                rect.height = rect.width * (1f / spriteAspect);
                rect.y += (height - rect.height) * base.rectTransform.pivot.y;
            }
            else
            {
                float width = rect.width;
                rect.width = rect.height * spriteAspect;
                rect.x += (width - rect.width) * base.rectTransform.pivot.x;
            }
        }
        #endregion

        #region Sliced
        void GenerateSlicedSprite(VertexHelper toFill)
        {
            if (!hasBorder)
            {
                base.OnPopulateMesh(toFill);
                return;
            }

            Vector4 outer, inner, padding, border;

            if (activeSprite != null)
            {
                outer = DataUtility.GetOuterUV(activeSprite);
                inner = DataUtility.GetInnerUV(activeSprite);
                padding = DataUtility.GetPadding(activeSprite);
                border = activeSprite.border;
            }
            else
            {
                outer = Vector4.zero;
                inner = Vector4.zero;
                padding = Vector4.zero;
                border = Vector4.zero;
            }

            Vector2 scale = SpriteBorderScale.CalculateSize(this) / multipliedPixelsPerUnit;
            Rect rect = GetPixelAdjustedRect();

            border = new Vector4(
                scale.x * border.x,
                scale.y * border.y,
                scale.x * border.z,
                scale.y * border.w);

            border = GetAdjustedBorders(border, rect, KeepBorderAspectRatio,
                new Vector2(
                    activeSprite.rect.width * scale.x,
                    activeSprite.rect.height * scale.y) );


            if ((border.x + border.z) > rect.width)
            {
                float s = rect.width / (border.x + border.z);
                border.x *= s;
                border.z *= s;
            }

            if (border.y + border.w > rect.height)
            {
                float s = rect.height / (border.y + border.w);
                border.y *= s;
                border.w *= s;
            }


            padding = padding / multipliedPixelsPerUnit;

            vertScratch[0] = new Vector2(padding.x, padding.y);
            vertScratch[3] = new Vector2(rect.width - padding.z, rect.height - padding.w);

            vertScratch[1].x = border.x;
            vertScratch[1].y = border.y;
            vertScratch[2].x = rect.width - border.z;
            vertScratch[2].y = rect.height - border.w;

            for (int i = 0; i < 4; i++)
            {
                vertScratch[i].x += rect.x;
                vertScratch[i].y += rect.y;
            }

            uvScratch[0] = new Vector2(outer.x, outer.y);
            uvScratch[1] = new Vector2(inner.x, inner.y);
            uvScratch[2] = new Vector2(inner.z, inner.w);
            uvScratch[3] = new Vector2(outer.z, outer.w);

            toFill.Clear();

            for (int x = 0; x < 3; x++)
            {
                int xIdx = x + 1;

                for (int y = 0; y < 3; y++)
                {
                    if (this.fillCenter || x != 1 || y != 1)
                    {
                        int yIdx = y + 1;


                        AddQuad(toFill,
                            posMin: new Vector2(vertScratch[x].x, vertScratch[y].y),
                            posMax: new Vector2(vertScratch[xIdx].x, vertScratch[yIdx].y),
                            bounds: rect,
                            mode: colorMode, colorA: color, colorB: secondColor,
                            uvMin: new Vector2(uvScratch[x].x, uvScratch[y].y),
                            uvMax: new Vector2(uvScratch[xIdx].x, uvScratch[yIdx].y));
                    }

                }
            }
        }

        #endregion

        #region Tiled
        private void GenerateTiledSprite(VertexHelper toFill)
        {
            Vector4 outerUV, innerUV, border;
            Vector2 spriteSize;

            if (activeSprite == null)
            {
                outerUV = Vector4.zero;
                innerUV = Vector4.zero;
                border = Vector4.zero;
                spriteSize = Vector2.one * 100f;
            }
            else
            {
                outerUV = DataUtility.GetOuterUV(activeSprite);
                innerUV = DataUtility.GetInnerUV(activeSprite);
                border = activeSprite.border;
                spriteSize = activeSprite.rect.size;
            }
            Rect rect = base.GetPixelAdjustedRect();

            float tileWidth = (spriteSize.x - border.x - border.z) / multipliedPixelsPerUnit;
            float tileHeight = (spriteSize.y - border.y - border.w) / multipliedPixelsPerUnit;

            border = this.GetAdjustedBorders(border / multipliedPixelsPerUnit, rect, false,
                                new Vector2(
                                    activeSprite.textureRect.width,
                                    activeSprite.textureRect.height));

            Vector2 scale = SpriteBorderScale.CalculateSize(this);
            tileWidth *= scale.x;
            tileHeight *= scale.y;

            Vector2 uvMin = new Vector2(innerUV.x, innerUV.y);
            Vector2 uvMax = new Vector2(innerUV.z, innerUV.w);

            UIVertex.simpleVert.color = this.color;

            // Min to max max range for tiled region in coordinates relative to lower left corner.
            float xMin = scale.x * border.x;
            float xMax = rect.width - (scale.x * border.z);
            float yMin = scale.y * border.y;
            float yMax = rect.height - (scale.y * border.w);

            toFill.Clear();

            Vector2 uvMax2 = uvMax;
            Vector2 pos = rect.position;

            if (tileWidth <= 0f)
            {
                tileWidth = xMax - xMin;
            }
            if (tileHeight <= 0f)
            {
                tileHeight = yMax - yMin;
            }
            if (this.fillCenter)
            {
                for (float y1 = yMin; y1 < yMax; y1 = y1 + tileHeight)
                {
                    float y2 = y1 + tileHeight;
                    if (y2 > yMax)
                    {
                        uvMax2.y = uvMin.y + (uvMax.y - uvMin.y) * (yMax - y1) / (y2 - y1);
                        y2 = yMax;
                    }
                    uvMax2.x = uvMax.x;
                    for (float x1 = xMin; x1 < xMax; x1 = x1 + tileWidth)
                    {
                        float x2 = x1 + tileWidth;
                        if (x2 > xMax)
                        {
                            uvMax2.x = uvMin.x + (uvMax.x - uvMin.x) * (xMax - x1) / (x2 - x1);
                            x2 = xMax;
                        }
                        AddQuad(toFill, rect,
                            new Vector2(x1, y1) + pos,
                            new Vector2(x2, y2) + pos,
                            colorMode, color, secondColor,
                            uvMin, uvMax2);
                    }
                }
            }
            if (this.hasBorder)
            {
                uvMax2 = uvMax;
                for (float y1 = yMin; y1 < yMax; y1 = y1 + tileHeight)
                {
                    float y2 = y1 + tileHeight;
                    if (y2 > yMax)
                    {
                        uvMax2.y = uvMin.y + (uvMax.y - uvMin.y) * (yMax - y1) / (y2 - y1);
                        y2 = yMax;
                    }
                    AddQuad(toFill, rect,
                        new Vector2(0f, y1) + pos,
                        new Vector2(xMin, y2) + pos,
                        colorMode, color, secondColor,
                        new Vector2(outerUV.x, uvMin.y), new Vector2(uvMin.x, uvMax2.y));

                    AddQuad(toFill, rect,
                        new Vector2(xMax, y1) + pos,
                        new Vector2(rect.width, y2) + pos,
                        colorMode, color, secondColor,
                        new Vector2(uvMax.x, uvMin.y), new Vector2(outerUV.z, uvMax2.y));
                }
                uvMax2 = uvMax;
                for (float x1 = xMin; x1 < xMax; x1 = x1 + tileWidth)
                {
                    float x2 = x1 + tileWidth;
                    if (x2 > xMax)
                    {
                        uvMax2.x = uvMin.x + (uvMax.x - uvMin.x) * (xMax - x1) / (x2 - x1);
                        x2 = xMax;
                    }
                    AddQuad(toFill, rect,
                        new Vector2(x1, 0f) + pos,
                        new Vector2(x2, yMin) + pos,
                        colorMode, color, secondColor,
                        new Vector2(uvMin.x, outerUV.y), new Vector2(uvMax2.x, uvMin.y));

                    AddQuad(toFill, rect,
                        new Vector2(x1, yMax) + pos,
                        new Vector2(x2, rect.height) + pos,
                        colorMode, color, secondColor,
                        new Vector2(uvMin.x, uvMax.y), new Vector2(uvMax2.x, outerUV.w));
                }
                AddQuad(toFill, rect,
                    new Vector2(0f, 0f) + pos,
                    new Vector2(xMin, yMin) + pos,
                    colorMode, color, secondColor,
                    new Vector2(outerUV.x, outerUV.y), new Vector2(uvMin.x, uvMin.y));

                AddQuad(toFill, rect,
                    new Vector2(xMax, 0f) + pos,
                    new Vector2(rect.width, yMin) + pos,
                    colorMode, color, secondColor,
                    new Vector2(uvMax.x, outerUV.y), new Vector2(outerUV.z, uvMin.y));

                AddQuad(toFill, rect,
                    new Vector2(0f, yMax) + pos,
                    new Vector2(xMin, rect.height) + pos,
                    colorMode, color, secondColor,
                    new Vector2(outerUV.x, uvMax.y), new Vector2(uvMin.x, outerUV.w));

                AddQuad(toFill, rect,
                    new Vector2(xMax, yMax) + pos,
                    new Vector2(rect.width, rect.height) + pos,
                    colorMode, color, secondColor,
                    new Vector2(uvMax.x, uvMax.y), new Vector2(outerUV.z, outerUV.w));
            }
        }
        #endregion

        private void AddQuad(
            VertexHelper vertexHelper, Rect bounds,
            Vector2 posMin, Vector2 posMax, 
            ColorMode mode, Color colorA, Color colorB,
            Vector2 uvMin, Vector2 uvMax)
        {
            ImageAppearanceProviderHelper.AddQuad(vertexHelper, bounds, posMin, posMax, mode, colorA, colorB, uvMin, uvMax, materialProperties);
        }
        

        Vector4 GetAdjustedBorders(Vector4 border, Rect rect, bool keepAspect, Vector2 texSize)
        {
            float scale = 1;
            for (int axis = 0; axis <= 1; axis++)
            {
                // If the rect is smaller than the combined borders, then there's not room for the borders at their normal size.
                // In order to avoid artefacts with overlapping borders, we scale the borders down to fit.
                float combinedBorders = border[axis] + border[axis + 2];
                if (rect.size[axis] < combinedBorders)// && combinedBorders != 0)
                {
                    if(keepAspect)
                    {
                        scale = Mathf.Min(scale, rect.size[axis] / combinedBorders);
                    }
                    else
                    {
                        float borderScaleRatio = rect.size[axis] / combinedBorders;
                        border[axis] *= borderScaleRatio;
                        border[axis + 2] *= borderScaleRatio;
                    }
                }
                else if (combinedBorders == 0 && keepAspect)
                {
                    int o = (axis + 1) % 2;
                    combinedBorders = border[o] + border[o + 2];

                    scale = rect.size[axis] / texSize[axis];
                    if(scale * combinedBorders > rect.size[o])
                    {
                        scale = rect.size[o] / combinedBorders;
                    }
                }
            }

            if (keepAspect)
            {
                border = scale * border;
            }

            return border;
        }

        public void OnResolutionChanged()
        {
            SetVerticesDirty();
            AssignSpriteSettings();
        }

        private void AssignSpriteSettings()
        {
            var settings = CurrentSpriteSettings;
            this.sprite = settings.Sprite;
            this.colorMode = settings.ColorMode;
            this.color = settings.PrimaryColor;
            this.secondColor = settings.SecondaryColor;
        }

#if UNITY_EDITOR

        protected override void OnValidate()
        {
            base.OnValidate();
            DoValidation();
            AssignSpriteSettings();
        }

#endif

        private void DoValidation()
        {
            bool isUnInitialized = fallbackSpriteSettings == null
                || (fallbackSpriteSettings.Sprite == null
                && fallbackSpriteSettings.ColorMode == ColorMode.Color
                && fallbackSpriteSettings.PrimaryColor == new Color());

            if (isUnInitialized)
            {
                fallbackSpriteSettings = new SpriteSettings(this.sprite, this.colorMode, this.color, this.secondColor);
            }

        }
    }
}
