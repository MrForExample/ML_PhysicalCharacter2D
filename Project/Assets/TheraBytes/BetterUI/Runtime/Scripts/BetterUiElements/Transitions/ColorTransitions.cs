using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using UnityEngine.UI;

#pragma warning disable 0649 // disable "never assigned" warnings

namespace TheraBytes.BetterUi
{
    [Serializable]
    public class ColorTransitions : TransitionStateCollection<Color>
    {
        static Dictionary<ColorTransitions, Coroutine> activeCoroutines = 
            new Dictionary<ColorTransitions, Coroutine>();

        static List<ColorTransitions> keysToRemove = new List<ColorTransitions>();


        public enum AffectedColor
        {
            ColorMixedIn = 0,

            MainColorDirect = 1,
            SecondColorDirect = 2,
        }

        [Serializable]
        public class ColorTransitionState : TransitionState
        {
            public ColorTransitionState(string name, Color stateObject)
                : base(name, stateObject)
            { }
        }


        public override UnityEngine.Object Target { get { return target; } }
        public float FadeDurtaion { get { return fadeDuration; } set { fadeDuration = value; } }


        [SerializeField]
        Graphic target;

        [Range(1, 5)]
        [SerializeField]
        float colorMultiplier = 1;

        [SerializeField]
        float fadeDuration = 0.1f;

        [SerializeField] AffectedColor affectedColor;

        [SerializeField]
        List<ColorTransitionState> states = new List<ColorTransitionState>();


        public ColorTransitions(params string[] stateNames)
            : base(stateNames)
        {
        }

        protected override void ApplyState(TransitionState state, bool instant)
        {
            if (this.Target == null)
                return;

            if (!(Application.isPlaying))
            {
                instant = true;
            }

            // Backwards compatibility: colorMultiplyer is a new field. 
            // It is 0 for upgrades from 1.x versions of Better UI.
            if (colorMultiplier <= float.Epsilon)
            {
                colorMultiplier = 1;
            }

            switch (affectedColor)
            {
                case AffectedColor.ColorMixedIn:
                    this.target.CrossFadeColor(state.StateObject * colorMultiplier, (instant) ? 0f : this.fadeDuration, true, true);
                    break;

                case AffectedColor.MainColorDirect:
                    CrossFadeColor(target.color, state.StateObject * colorMultiplier, (instant) ? 0 : fadeDuration);
                    break;

                case AffectedColor.SecondColorDirect:
                    Color start;
                    if (target is IImageAppearanceProvider img)
                    {
                        start = img.SecondColor;
                    }
                    else
                    {
                        throw new NotSupportedException("SecondaryColor transition not suppoted for " 
                            + target.GetType().Name);
                    }
                    
                    CrossFadeColor(start, state.StateObject * colorMultiplier, (instant) ? 0 : fadeDuration);
                    break;


                default:
                    throw new NotImplementedException();
            }

        }

        internal override void AddStateObject(string stateName)
        {
            var obj = new ColorTransitionState(stateName, Color.white);
            this.states.Add(obj);
        }

        protected override IEnumerable<TransitionState> GetTransitionStates()
        {
            foreach (var s in states)
                yield return s;
        }

        internal override void SortStates(string[] sortedOrder)
        {
            base.SortStatesLogic(states, sortedOrder);
        }

        void CrossFadeColor(Color startValue, Color targetValue, float duration)
        {

            // Stop clashing coroutines
            foreach (var key in activeCoroutines.Keys)
            {
                if (key.target == this.target && key.affectedColor == this.affectedColor)
                {
                    if (key.target != null)
                        key.target.StopCoroutine(activeCoroutines[key]);

                    keysToRemove.Add(key);
                }
            }

            foreach (var key in keysToRemove)
            {
                activeCoroutines.Remove(key);
            }

            keysToRemove.Clear();

            // trigger value changes
            if (duration == 0 || !target.enabled || !target.gameObject.activeInHierarchy)
            {
                ApplyColor(targetValue);
            }
            else
            {
                Coroutine coroutine = target.StartCoroutine(CoCrossFadeColorDirect(startValue, targetValue, duration));
                activeCoroutines.Add(this, coroutine);
            }
        }

        private IEnumerator CoCrossFadeColorDirect(Color startValue, Color targetValue, float duration)
        {
            // animate
            float startTime = Time.unscaledTime;
            float endTime = startTime + duration;

            while (Time.unscaledTime < endTime)
            {
                float amount = (Time.unscaledTime - startTime) / duration;
                Color value = Color.Lerp(startValue, targetValue, amount);
                ApplyColor(value);
                yield return null;
            }

            ApplyColor(targetValue);
        }


        void ApplyColor(Color color)
        {
            if (target is IImageAppearanceProvider img)
            {
                switch (affectedColor)
                {
                    case AffectedColor.MainColorDirect:
                        img.color = color;
                        break;
                    case AffectedColor.SecondColorDirect:
                        img.SecondColor = color;
                        break;
                    default: throw new ArgumentException("MainColorDirect or GradientSecondColor expected.");
                }
            }
            else if(affectedColor == AffectedColor.MainColorDirect)
            {
                target.color = color;
            }
            else
            {
                throw new ArgumentException("affected object doesn't have a secondary color.");
            }
        }

    }

}
