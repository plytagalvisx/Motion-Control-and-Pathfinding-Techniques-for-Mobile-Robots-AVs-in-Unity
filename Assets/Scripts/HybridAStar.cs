#nullable enable

using UnityEngine.Assertions;
using Unity.Collections;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using UnityEditor;
using System.Linq;

public class HybridAStar
{
    public static TerrainInfo m_Terrain;
    public int max_iters = 500000;
    public float goal_radius = 7f;

    // Define constants:
    // These constants represent the kinematic constraints of the vehicle. (KinematicModel)
    // For instance, v is the maximum velocity of the vehicle.
    // dt is the time step, and R is the minimum turning radius of the vehicle.
    public float v; // = 5f;
    public float dt; // = 1f;
    public float R; // = 7.5f;

    public float vehicle_width; // = 4.5f;
    public float vehicle_length; // = 2.5f;

    // Define grid size
    public static float grid_size_xz; // = v*dt        *1.0f;
    public static float grid_size_theta; // = v/R*dt   *1.0f;

    // Hybrid A* data structures
    private Node start;
    private (float x, float z) goal_pos;

    private Dictionary<(float x_val, float z_val, float theta_val), Node> open_list;
    private Dictionary<(float x_val, float z_val, float theta_val), Node> closed_list;

    private PriorityQueue<Node, float> open_queue;

    // A* Constants and Datastructures
    public float AStarGridResolution; // = 1f; // Likely want this to be way smaller.
    public Dictionary<(float x, float z), float> AStarOpenList;
    public Dictionary<(float x, float z), float> AStarClosedList;

    // Define the action space, where item1 is the velocity and item2 is the turning radius.

    public List<(float v, float R)> action_space;

    // public float turning_cost; // = dt*0.2f;

    public float speed = 10f;
    public float mu = 1.3f; // Friction coefficient
    public float minimum_turning_radius = 10f; // Minimum turning radius. Note that this is based on some kind of friction coefficient and the speed of the car.

    public Vector3 GetDirectionFromHeading(float heading) // heading is the same as theta
    {
        // Convert the heading angle to a unit vector in the XZ plane
        float xComponent = Mathf.Cos(heading);
        float zComponent = Mathf.Sin(heading);
        // Create the direction vector in the XZ plane (assuming Y is up)
        Vector3 direction = new Vector3(xComponent, 0f, zComponent);
        return direction;
    }

    public HybridAStar(TerrainInfo terrain)
    {
        m_Terrain = terrain;
        float facingUpwardsAngle = Mathf.PI / 2f;
        start = new Node(m_Terrain.start_pos.x, m_Terrain.start_pos.z, facingUpwardsAngle, 0f, 0f, null); // theta = 90 degrees (agent is facing upwards)
        goal_pos = (m_Terrain.goal_pos.x, m_Terrain.goal_pos.z);

        // It works! The direction vector faces upwards since heading/theta is 90 degrees. 
        // Vector3 start_direction = GetDirectionFromHeading(start.theta);
        // Debug.Log("Start direction: " + start_direction);
        // Debug.DrawLine(new Vector3(start.x, 0, start.z), new Vector3(start.x, 0, start.z) + start_direction * 10f, Color.blue, 1000f);

        open_list = new Dictionary<(float x_val, float z_val, float theta_val), Node>();
        closed_list = new Dictionary<(float x_val, float z_val, float theta_val), Node>();
        open_queue = new PriorityQueue<Node, float>();

        // Initialize A* data structures
        AStarOpenList = new Dictionary<(float, float), float>();
        AStarClosedList = new Dictionary<(float, float), float>();

        // Initialize constants:
        // Calculate minimum turning radius based on the cars speed
        // v^2 / minimum_turning_radius = mu * g
        float gravity_constant = 9.81f;
        minimum_turning_radius = MathF.Pow(speed, 2) / (mu * gravity_constant);
        // Modify the speed and turning radius parameters
        v = speed; // 3.5f;
        dt = 1 / speed; // 1f;
        R = minimum_turning_radius; // 7.5f;

        vehicle_length = 3.5f; // 2.7f;
        vehicle_width = 5.5f; // 4.5f;

        // This has a strong affect on the path?
        AStarGridResolution = 2.5f; // 1f;

        initializer();

        // turning_cost = dt*0.2f;

        Debug.Log("Hybrid A* Initialized");
    }

    public void initializer()
    {
        // Initialize grid size
        grid_size_xz = v * dt * 1.0f;
        grid_size_theta = v / R * dt * 1.0f;

        // Initialize action space (control law/control input that influences state transition of the car)
        action_space = new List<(float, float)>() {
            (v, 0f), // Straight
            (v, R), // Right
            (v, -R), // Left
        };
    }

    public class Node
    {
        // public Vector3 position;
        // public Vector3 direction;

        public float x;
        public float z;
        public float theta;
        public float g;
        public float h;

        public Node? parent;

        public Node(float x_val, float z_val, float theta_val, float g_val, float h_val, Node? parent_val)
        {
            x = x_val;
            z = z_val;
            theta = theta_val; // represents heading/direction/orientation
            g = g_val;
            h = h_val;
            parent = parent_val;
        }

        public (float x_rounded, float z_rounded, float theta_rounded) RoundState()
        {
            float x_rounded = Mathf.Round(this.x / grid_size_xz) * grid_size_xz;
            float z_rounded = Mathf.Round(this.z / grid_size_xz) * grid_size_xz;
            float theta_rounded = Mathf.Round(this.theta / grid_size_theta) * grid_size_theta;

            return (x_rounded, z_rounded, theta_rounded);
        }
    }

    private float Heuristic(Node node)
    {
        // Use the A* distance as a heuristic
        float L = AStarDistance(node);
        // We want to return the time, not the length t = L/v:
        return L / v;
    }

    private bool CollisionCheck((float x, float y, float theta) node, float width, float length)
    {
        bool traversable = true;
        float node_x = node.Item1;
        float node_z = node.Item2;
        float node_theta = node.Item3;

        foreach (float dx in new List<float> { -width / 2, 0, width / 2 })
        {
            foreach (float dz in new List<float> { -length / 2, -length / 4, 0, length / 4, length / 2 })
            {

                // Dynamic Vehicle Model:
                float x = node_x + dx * Mathf.Cos(node_theta) - dz * Mathf.Sin(node_theta); // Rotates the point around the origin by theta?
                float z = node_z + dx * Mathf.Sin(node_theta) + dz * Mathf.Cos(node_theta);

                if (m_Terrain.traversability[m_Terrain.get_i_index(x), m_Terrain.get_j_index(z)] == 1)
                {
                    traversable = false;
                }
            }
        }
        return traversable;
    }

    // Get successor to node given action.
    private Node GetSuccessor(Node node, (float v, float R) action)
    {
        // Get the current state
        float x = node.x;
        float z = node.z;
        float theta = node.theta; // represents heading 

        // Get the action 
        float v = action.v;
        float R = action.R;

        // Calculate the new state, if R == 0, then the car is going straight.
        // Updates the position and heading of the car.
        float x_new, z_new, theta_new;
        if (R == 0)
        {
            // Kinematic Model:
            x_new = x + dt * v * MathF.Cos(theta); // position.x += dt * (float)Math.Cos(heading) * maxV;
            z_new = z + dt * v * MathF.Sin(theta);
            theta_new = theta;
        }
        else
        {
            x_new = x + R * (MathF.Sin((dt * v + R * theta) / R) - MathF.Sin(theta));
            z_new = z - R * (MathF.Cos((dt * v + R * theta) / R) - MathF.Cos(theta));
            theta_new = theta + (dt * v) / R;
        }

        // Heuristic And Cost is calculated in the search function.

        return new Node(x_new, z_new, theta_new, 0, 0, node);
    }

    private Node Search()
    {
        Debug.Log("Searching...");

        // Add the start state to list and queue
        open_list.Add(start.RoundState(), start);
        open_queue.Enqueue(start, start.g + start.h);

        int iters = 0;

        while (open_queue.Count > 0)
        {
            iters += 1;
            if (iters > max_iters)
            {
                Debug.Log("Max iters reached");
                return null;
            }

            // Get the node with the lowest cost
            Node current = open_queue.Dequeue();
            // Remove the node from the open list
            open_list.Remove(current.RoundState());
            // Add the node to the closed list
            closed_list.Add(current.RoundState(), current);

            // Check if the goal has been reached
            if (MathF.Pow(current.x - goal_pos.x, 2) + MathF.Pow(current.z - goal_pos.z, 2) < goal_radius * goal_radius)
            {
                return current;
            }

            // Expand the node (neighbouring search)
            foreach ((float v, float R) action in action_space)
            {
                Node successor = GetSuccessor(current, action);

                // Check if the successor is traversable
                if (!CollisionCheck((successor.x, successor.z, successor.theta), vehicle_width, vehicle_length))
                {
                    continue;
                }

                // Update heuristic and cost
                successor.h = Heuristic(successor);
                successor.g = current.g + dt;

                // If the car is turning, add an extra turning cost
                // if (R != 0) {
                //     successor.g += turning_cost;
                // }

                // Check if the successor is in the closed list
                if (closed_list.ContainsKey(successor.RoundState()))
                {
                    continue;
                }

                // Check if the successor is in the open list
                if (!open_list.ContainsKey(successor.RoundState()))
                {
                    open_list.Add(successor.RoundState(), successor);
                    open_queue.Enqueue(successor, successor.g + successor.h);

                    Debug.DrawLine(new Vector3(current.x, 0, current.z), new Vector3(successor.x, 0, successor.z), Color.green, 1000f);
                }
                // Else, if the g-value of the successor is lesser than the g-value of the node in the open list, update the node.
                else if (successor.g < open_list[successor.RoundState()].g)
                {
                    open_list[successor.RoundState()] = successor;

                    // Remove the node from the queue and add it again to update the priority.
                    // As C# does not have a decrease key function, update the priority by recreating the queue.
                    open_queue.Clear();
                    foreach (Node node in open_list.Values)
                    {
                        open_queue.Enqueue(node, node.g + node.h);
                    }

                    Debug.DrawLine(new Vector3(current.x, 0, current.z), new Vector3(successor.x, 0, successor.z), Color.red, 1000f);
                }

            }
        }
        return null;
    }

    public List<Vector3> GetPath(bool is_drone = false)
    {
        List<Vector3> path = new List<Vector3>();
        // If the path is for the drone, add 8 more start points in every direction to the open list.
        if (is_drone)
        {
            foreach (float alpha in new float[] { 45, 90, 135, 180, 225, 270, 315 })
            {
                Node node = new Node(start.x, start.z, start.theta + alpha, 0, 0, null);
                open_list[node.RoundState()] = node;
            }
        }
        Node current = Search();
        if (current == null)
        {
            // If no path is found, return the start position.
            Debug.Log("No path found");
            current = start;
        }
        while (current != null)
        {
            path.Add(new Vector3(current.x, 0, current.z));
            current = current.parent;
        }
        path.Reverse();
        return path;
    }

    // --------------------------------------------------
    //                  A STAR HEURISTIC
    // --------------------------------------------------

    private float EuclidianHeuristic((float x, float z) node)
    {
        return MathF.Sqrt(MathF.Pow(node.x - goal_pos.x, 2) + MathF.Pow(node.z - goal_pos.z, 2));
    }

    private float AStarDistance(Node node)
    {
        (float x, float z) node_state = (MathF.Round(node.x / AStarGridResolution) * AStarGridResolution, MathF.Round(node.z / AStarGridResolution) * AStarGridResolution);
        (float x, float z) goal_state = (MathF.Round(goal_pos.x / AStarGridResolution) * AStarGridResolution, MathF.Round(goal_pos.z / AStarGridResolution) * AStarGridResolution);

        // A star goes from the goal to the node, such that the closed_list is reusable.

        // If node_state is in closed list return the value
        if (AStarClosedList.ContainsKey(node_state))
        {
            return AStarClosedList[node_state];
        }

        // If node_state is equals to end_state return 0
        if (node_state == goal_state)
        {
            return 0;
        }

        // If the goal state is not in the closed list, then add it to the open list.
        if (!AStarClosedList.ContainsKey(goal_state))
        {
            AStarOpenList.Add(goal_state, 0);
        }

        // While there are nodes in openlist
        while (AStarOpenList.Count > 0)
        {
            // Get the node with the lowest cost
            float min_cost = float.MaxValue;
            (float x, float z) current = (0, 0);
            foreach ((float x, float z) n in AStarOpenList.Keys)
            {
                float cost = AStarOpenList[n] + EuclidianHeuristic(n);
                if (cost < min_cost)
                {
                    min_cost = cost;
                    current = n;
                }
            }

            // Add the node to the closed list
            AStarClosedList.Add(current, AStarOpenList[current]);
            // Remove the node from the open list
            AStarOpenList.Remove(current);

            // Check if the goal has been reached
            if (current == node_state)
            {
                return AStarClosedList[current];
            }

            // Expand the node
            foreach (float x in new float[] { -AStarGridResolution, 0, AStarGridResolution })
            {
                foreach (float z in new float[] { -AStarGridResolution, 0, AStarGridResolution })
                {
                    // Skip the current node
                    if (x == 0 && z == 0)
                    {
                        continue;
                    }

                    // Get the successor
                    (float x, float z) successor = (current.x + x, current.z + z);

                    // Check if the successor is traversable
                    float size = MathF.Min(vehicle_width, vehicle_length) * 1.5f;
                    if (!CollisionCheck((successor.x, successor.z, 0), size, size))
                    {
                        continue;
                    }

                    // Update cost
                    float cost = AStarClosedList[current] + MathF.Sqrt(MathF.Pow(x, 2) + MathF.Pow(z, 2));

                    // Check if the successor is in the closed list
                    if (AStarClosedList.ContainsKey(successor))
                    {
                        continue;
                    }
                    // If the sucessor is not in the open list, add it
                    else if (!AStarOpenList.ContainsKey(successor))
                    {
                        AStarOpenList.Add(successor, cost);
                    }
                    // Else, if the cost of the successor is lesser than the cost of the node in the open list, update the node.
                    else if (cost < AStarOpenList[successor])
                    {
                        AStarOpenList[successor] = cost;
                    }
                }
            }

        }
        // Debug.Log("AStarDistance: No path found");
        // Debug.Log("AStarDistance: Node: " + node_state);
        return float.MaxValue;
    }
}