using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using UnityEngine;
using UnityEngine.Events;
using UnityEngine.EventSystems;

namespace TheraBytes.BetterUi
{
    [HelpURL("https://documentation.therabytes.de/better-ui/ValueDragger.html")]
    [AddComponentMenu("Better UI/Controls/Value Dragger", 30)]
    public class ValueDragger : BetterSelectable, IDragHandler, IBeginDragHandler, IPointerClickHandler
    {
        #region Nested Types
        [Serializable]
        public class ValueDragEvent : UnityEvent<float> { }

        public enum DragDirection
        {
            Horizontal = 0, // maps to Vector2's X index
            Vertical = 1,   // maps to Vector2's Y index
        }

        [Serializable]
        public class DragSettings : IScreenConfigConnection
        {
            public DragDirection Direction = DragDirection.Horizontal;
            public bool Invert;

            [SerializeField]
            string screenConfigName;
            public string ScreenConfigName { get { return screenConfigName; } set { screenConfigName = value; } }
        }

        [Serializable]
        public class DragSettingsConfigCollection : SizeConfigCollection<DragSettings> { }

        [Serializable]
        public class ValueSettings : IScreenConfigConnection
        {
            public bool HasMinValue;
            public float MinValue = 0f;

            public bool HasMaxValue;
            public float MaxValue = 1f;

            public bool WholeNumbers;

            [SerializeField]
            string screenConfigName;
            public string ScreenConfigName { get { return screenConfigName; } set { screenConfigName = value; } }
        }

        [Serializable]
        public class ValueSettingsConfigCollection : SizeConfigCollection<ValueSettings> { }
        #endregion

        [SerializeField]
        DragSettings fallbackDragSettings = new DragSettings();
        [SerializeField]
        DragSettingsConfigCollection customDragSettings = new DragSettingsConfigCollection();

        [SerializeField]
        ValueSettings fallbackValueSettings = new ValueSettings();
        [SerializeField]
        ValueSettingsConfigCollection customValueSettings = new ValueSettingsConfigCollection();

        [SerializeField]
        FloatSizeModifier fallbackDragDistance = new FloatSizeModifier(1, float.Epsilon, 10000);
        [SerializeField]
        FloatSizeConfigCollection customDragDistance = new FloatSizeConfigCollection();

        [SerializeField]
        float value;

        [SerializeField]
        ValueDragEvent onValueChanged = new ValueDragEvent();

        float internalValue;

        public DragSettings CurrentDragSettings 
        {
            get { return customDragSettings.GetCurrentItem(fallbackDragSettings); } 
        }

        public ValueSettings CurrentValueSettings
        { 
            get { return customValueSettings.GetCurrentItem(fallbackValueSettings); }
        }

        public FloatSizeModifier CurrentDragDistanceSizer
        { 
            get { return customDragDistance.GetCurrentItem(fallbackDragDistance); } 
        }

        public float Value { get { return this.value; } set { ApplyValue(value); } }

        public ValueDragEvent OnValueChanged { get { return onValueChanged; } set { onValueChanged = value; } }

        void IBeginDragHandler.OnBeginDrag(PointerEventData eventData)
        {
            internalValue = value;
            CurrentDragDistanceSizer.CalculateSize(this);
        }

        void IDragHandler.OnDrag(PointerEventData eventData)
        {
            var dragSettings = CurrentDragSettings;

            int axis = (int)dragSettings.Direction;
            float delta = eventData.delta[axis];
            float divisor = CurrentDragDistanceSizer.LastCalculatedSize;

            internalValue += (dragSettings.Invert)
                ? -delta / divisor
                :  delta / divisor;

            ApplyValue(internalValue);
        }

        void IPointerClickHandler.OnPointerClick(PointerEventData eventData)
        {
            // consume the click by implementing this interface.
            // we don't want to let the click pass through to a control higher up in the hierarchy.
        }

        private void ApplyValue(float val)
        {
            var valueSettings = CurrentValueSettings;
            if(valueSettings.HasMinValue && val < valueSettings.MinValue)
            {
                val = valueSettings.MinValue;
            }
            else if(valueSettings.HasMaxValue && val > valueSettings.MaxValue)
            {
                val = valueSettings.MaxValue;
            }

            if(valueSettings.WholeNumbers)
            {
                val = (int)val;
            }

            if (val != value)
            {
                value = val;
                onValueChanged.Invoke(value);
            }
        }
    }
}
