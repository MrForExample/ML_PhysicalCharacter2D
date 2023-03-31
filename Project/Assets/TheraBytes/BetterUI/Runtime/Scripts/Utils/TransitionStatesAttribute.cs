using System;

namespace TheraBytes.BetterUi
{
    public class DefaultTransitionStatesAttribute : TransitionStatesAttribute
    {
        public DefaultTransitionStatesAttribute()
            : base("Normal", "Highlighted", "Pressed",
#if UNITY_2019_1_OR_NEWER
            "Selected",
#endif
            "Disabled")
        { }
    }

    public class TransitionStatesAttribute : Attribute
    {
        string[] states;
        public string[] States { get { return states; } }

        public TransitionStatesAttribute(params string[] states)
        {
            this.states = states;
        }
    }
}
