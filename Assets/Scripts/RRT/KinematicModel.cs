using UnityEngine;
using UnityStandardAssets.Vehicles.Car;
using System;
using static StateData;

public class KinematicModel
{
    public float maxV = 13f; // max velocity of car
    public float maxDistance = 40f; // max distance of car
    public float dt = 0.02f; // time step is usually 0.01f in the car controller 
    public float L = 4.47f; // wheelbase of car

    // Initialize constants:
    // vehicle_length = 2.7f;
    // vehicle_width = 4.5f;
    // v = 3.5f;
    // dt = 1f;
    // R = 7.5f;
    // public float goal_radius = 7f;


    public float maxSteering = 25 * Mathf.Deg2Rad; // max steering angle of car

    public Vector3 position;
    public Vector3 orientation;
    public float heading; // it is still an orientation of car in float form
    public Tree<State> tree;

    public KinematicModel(Tree<State> tree, Vector3 position, Vector3 orientation) // here we give the current state of the car
    {
        this.tree = tree;
        this.position = position;
        this.orientation = orientation; // orientation of car (of the front wheels since it starts as transform.TransformDirection(Vector3.forward)?)
    }

    // Given current state in KinematicModel() constructor and next node in Update(), we calculate the next state/pose of the car:
    public void Update(Vector3 nextPosition) // updates position and orientation (orientation/heading) based on next node in path
    {
        Vector3 targetDirection = (nextPosition - position).normalized;

        float heading = Vector3.SignedAngle(orientation, Vector3.right, Vector3.up) * Mathf.Deg2Rad; // calculate heading from orientation vector. Determines the direction in which the agent vehicle should move.
        Vector3 orthogonalDirection = Vector3.Cross(orientation.normalized, Vector3.up).normalized; // calculate orthogonal vector to orientation
        float steering = Vector3.Dot(targetDirection, orthogonalDirection); // calculate steering angle from orthogonal vector and vector to next node

        Vector3 oldPosition = position;
        // Vector3 oldOrientation = orientation;

        // float minimum_turning_radius1 = L / (float)Mathf.Tan(maxSteering); // minimum turning radius of car
        // float minimum_turning_radius2 = L / (float)Mathf.Sin(maxSteering); // minimum turning radius of car

        // Debug.Log("Minimum turning radius 1: " + minimum_turning_radius1);
        // Debug.Log("Minimum turning radius 2: " + minimum_turning_radius2);

        // float theta = maxV / minimum_turning_radius; // calculate theta (angular velocity) based on max velocity and minimum turning radius

        // position.x += dt * (float)Math.Cos(theta);
        // position.z += dt * (float)Math.Sin(theta);
        // theta += dt / minimum_turning_radius;

        position.x += dt * maxV * (float)Math.Cos(heading); // calculate new position for x based on heading and velocity
        position.z += dt * maxV * (float)Math.Sin(heading); // calculate new position for z based on heading and velocity
        heading += dt * maxV / L * (float)Math.Tan(steering);

        orientation = new Vector3((float)Math.Cos(heading), 0, (float)Math.Sin(heading)); // orientation in vector form
        // orientation = new Vector3(-(float)Math.Sin(heading), 0, (float)Math.Cos(heading)); // orientation in vector form


        Debug.DrawLine(oldPosition, position, Color.red, 100f);
        // Debug.DrawLine(oldPosition + oldOrientation, position + orientation, Color.cyan, 100f);
    }

    public void MoveWithinDistance(Node<State> movableNode, Node<State> fixedNode)
    {
        // Get angle between nodes
        float steering = tree.GetAngle(movableNode, fixedNode); // aka heading (theta)

        if (IsWithinDistance(movableNode, fixedNode))
        {
            position = movableNode.position;
        }
        else
        {
            // Set movableNode to be the same angle as earlier, but with distance of maxDistance
            position.x += fixedNode.position.x + maxDistance * (float)Math.Cos(steering); // calculate new position for x based on heading and velocity
            position.z += fixedNode.position.z + maxDistance * (float)Math.Sin(steering); // calculate new position for z based on heading and velocity
        }

        orientation = new Vector3((float)Math.Cos(steering), 0, (float)Math.Sin(steering)); // orientation in vector form
    }

    private bool IsWithinDistance(Node<State> nodeA, Node<State> nodeB)
    {
        return tree.GetDistance(nodeA.position, nodeB.position, 2) <= maxDistance;
    }

    public State GetState()
    {
        return new State(position, orientation);
    }

}

// While both methods (Update() and MoveWithinDistance()) involve updating the agent's position and orientation, they have distinct purposes within the RRT 
// algorithm: one is for following a predefined path, while the other is for moving towards a target node while staying 
// within a certain distance. Therefore, they are not performing exactly the same thing, although they share some 
// similarities in terms of updating the agent's state. (???)






