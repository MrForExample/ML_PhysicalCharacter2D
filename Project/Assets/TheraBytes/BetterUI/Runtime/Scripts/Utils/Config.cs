using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace TheraBytes.BetterUi
{
    public static class Config
    {
        static bool applyAssignmentsToCurrentSettings = true;

        /// <summary>
        /// If true (default), setters of properties which exist in the unity-base-class 
        /// will also change the current settings of the active screen configuration.
        /// 
        /// Example: myBetterImage.color = Color.green; // <- will also change the color in the current settings
        /// 
        /// Note that this will only happen if you access the property through a variable
        /// which is declared as the better version.
        /// </summary>
        public static bool ApplyAssignmentsToCurrentSettings
        {
            get { return applyAssignmentsToCurrentSettings; }
            set { applyAssignmentsToCurrentSettings = value; }
        }

        /// <summary>
        /// Internal helper method for setters of hidden or overridden properties.
        /// </summary>
        public static void Set<T>(T value, Action<T> setBase, Action<T> setSettings)
        {
            if (applyAssignmentsToCurrentSettings)
                setSettings(value);

            setBase(value);
        }
    }
}
