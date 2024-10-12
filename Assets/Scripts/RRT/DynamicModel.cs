using UnityEngine;
using UnityStandardAssets.Vehicles.Car;
using System;
using static StateData;

public class DynamicModel
{
    public float maxV = 13f; // max velocity of car
    public float dt = 0.02f; // time step is usually 0.01f in the car controller 
    public float L = 4.47f; // wheelbase of car

    public float yawRate; // yaw rate of car
    public float lateralVelocity; // lateral velocity of car
    public float longitudinalVelocity; // longitudinal velocity of car

    public float maxSteering = 25 * Mathf.Deg2Rad; // max steering angle of car

    public Vector3 position;
    public Vector3 orientation;
    public float heading; // it is still an orientation of car in float form

    public float gravity = 9.81f; // gravity constant
    public float slipAngle; // slip angle of car


    public DynamicModel(Vector3 position, Vector3 orientation) // here we give the current state of the car
    {
        this.position = position;
        this.orientation = orientation; // orientation of car
    }

    // Given current state in KinematicModel() constructor and next node in Update(), we calculate the next state/pose of the car:
    public void Update(Vector3 nextPosition) // updates position and orientation (orientation/heading) based on next node in path
    {
        Vector3 targetDirection = (nextPosition - position).normalized;

        float heading = Vector3.SignedAngle(orientation, Vector3.right, Vector3.up) * Mathf.Deg2Rad; // calculate heading from orientation vector
        Vector3 orthogonalDirection = Vector3.Cross(orientation.normalized, Vector3.up).normalized; // calculate orthogonal vector to orientation
        float steering = Vector3.Dot(targetDirection, orthogonalDirection); // calculate steering angle from orthogonal vector and vector to next node

        Vector3 oldPosition = position;
        // Vector3 oldOrientation = orientation;

        // slipAngle = L /

        // yawRate = maxV / L *

        // position.x += dt * maxV * (float)Math.Cos(heading) - dt * lateralVelocity * (float)Math.Sin(heading); // calculate new position for x based on heading and velocity
        // position.z += dt * maxV * (float)Math.Sin(heading) + dt * lateralVelocity * (float)Math.Cos(heading); // calculate new position for z based on heading and velocity
        // heading += dt * yawRate;

        orientation = new Vector3((float)Math.Cos(heading), 0, (float)Math.Sin(heading)); // orientation in vector form

        Debug.DrawLine(oldPosition, position, Color.red, 100f);
        // Debug.DrawLine(oldPosition + oldOrientation, position + orientation, Color.cyan, 100f);
    }

    public State GetState()
    {
        return new State(position, orientation);
    }

}
