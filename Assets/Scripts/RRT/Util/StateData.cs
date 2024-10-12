using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class StateData
{
    public struct State
    {
        public State(Vector3 position, Vector3 orientation) // Contains both position and orientation of the robot (aka the state/pose)
        {
            Position = position;
            Orientation = orientation;
        }

        public Vector3 Position { get; set; }
        public Vector3 Orientation { get; set; }
    }
}