using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace SquidGame
{
    public interface SquidInterface
    {
        public bool canAgentMove {get;set;}
        public void TurnOnOrOffSupport(bool supportOn);
        public void TurnOnOrOffJoints(bool jointsOn);
    }
}
