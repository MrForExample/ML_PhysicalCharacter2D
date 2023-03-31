using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using UnityEngine;
using UnityEngine.UI;

namespace TheraBytes.BetterUi
{
    [HelpURL("https://documentation.therabytes.de/better-ui/BetterToggle.html")]
    [AddComponentMenu("Better UI/Controls/Better Toggle", 30)]
    public class BetterToggle : Toggle, IBetterTransitionUiElement
    {
        public List<Transitions> BetterTransitions { get { return betterTransitions; } }
        public List<Transitions> BetterTransitionsWhenOn { get { return betterTransitionsWhenOn; } }
        public List<Transitions> BetterTransitionsWhenOff { get { return betterTransitionsWhenOff; } }
        public List<Transitions> BetterToggleTransitions { get { return betterToggleTransitions; } }

        [SerializeField, DefaultTransitionStates]
        List<Transitions> betterTransitions = new List<Transitions>();

        [SerializeField, TransitionStates("On", "Off")]
        List<Transitions> betterToggleTransitions = new List<Transitions>();
        [SerializeField, DefaultTransitionStates]
        List<Transitions> betterTransitionsWhenOn = new List<Transitions>();
        [SerializeField, DefaultTransitionStates]
        List<Transitions> betterTransitionsWhenOff = new List<Transitions>();

        bool wasOn;

        protected override void OnEnable()
        {
            base.OnEnable();
            ValueChanged(base.isOn, true);
            DoStateTransition(SelectionState.Normal, true);
        }

        void Update()
        {
            if (wasOn != isOn)
            {
                ValueChanged(isOn);
            }
        }

        protected override void DoStateTransition(SelectionState state, bool instant)
        {
            base.DoStateTransition(state, instant);

            if (!(base.gameObject.activeInHierarchy))
                return;

            var stateTransitions = (isOn)
                ? betterTransitionsWhenOn
                : betterTransitionsWhenOff;

            foreach (var info in stateTransitions)
            {
                info.SetState(state.ToString(), instant);
            }

            foreach (var info in betterTransitions)
            {
                if (state != SelectionState.Disabled && isOn)
                {
                    var tglTr = betterToggleTransitions.FirstOrDefault(
                        (o) => o.TransitionStates != null && info.TransitionStates != null
                            && o.TransitionStates.Target == info.TransitionStates.Target
                            && o.Mode == info.Mode);

                    if (tglTr != null)
                    {
                        continue;
                    }
                }

                info.SetState(state.ToString(), instant);
            }
        }

        private void ValueChanged(bool on)
        {
            ValueChanged(on, false);
        }

        private void ValueChanged(bool on, bool immediate)
        {
            wasOn = on;
            foreach (var state in betterToggleTransitions)
            {
                state.SetState((on) ? "On" : "Off", immediate);
            }

            var stateTransitions = (on)
                ? betterTransitionsWhenOn
                : betterTransitionsWhenOff;

            foreach (var state in stateTransitions)
            {
                state.SetState(currentSelectionState.ToString(), immediate);
            }
        }

    }
}
