using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TheraBytes.BetterUi
{
    public class HueSaturationBrightnessMaterial : PredefinedMaterialInfo
    {
        public const string Name = "Hue Saturation Brightness";
        public const int Hue = 0;
        public const int Saturation = 1;
        public const int Brightness = 2;

        public override string NameId => Name;
        public int HueIndex => Hue;
        public int SaturationIndex => Saturation;
        public int BrightnessIndex => Brightness;
    }

    public class ColorOverlayMaterial : PredefinedMaterialInfo
    {
        public const string Name = "Color Overlay";
        public const int Opacity = 0;

        public override string NameId => Name;
        public int OpacityIndex => Opacity;
    }

    public class GrayscaleMaterial : PredefinedMaterialInfo
    {
        public const string Name = "Grayscale";
        public const int Amount = 0;

        public override string NameId => Name;
        public int AmountIndex => Amount;
    }

    public class StandardMaterial : PredefinedMaterialInfo
    {
        public const string Name = "Standard";

        public override string NameId => Name;
    }

    public abstract class PredefinedMaterialInfo
    {
        public abstract string NameId { get; }
        public static implicit operator string(PredefinedMaterialInfo material)
        {
            return material.NameId;
        }
    }
}
