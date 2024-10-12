using System;
using System.Collections.Generic;
using System.Data;
using UnityEngine;
using static StateData;

public class RRTSearchAlgorithm
{
    // private readonly float MAX_ANGLE = 0.33f;
    private readonly float GOAL_RANGE = 5f;
    private readonly float INFLATION_RADIUS = 4.0f;     // car max length is 4.47

    // Simulation (propagation/driving) steps:
    // Step size = 20
    // TerrainA = 300k inter +/- 40k
    // TerrainB = 
    // TerrainC = 2k iter
    private readonly int MAX_STEPS = 20; //30;
    private readonly int MAX_ITERATIONS = 400000; //20000; // Change this value if you want to run for longer time to find a path (reach goal)

    private Node<State> startNode;
    private Node<State> goalNode;

    private readonly PositionSampler sampler;

    public Tree<State> tree;
    private float carHeight = 3f;
    private float carWidth = 3f;
    public float radius = 50f;

    public bool pathFound = false;
    public bool visualizeTree = true;
    public float nodeRadius = 1f;


    private void Log(string msg) { Debug.Log($"[RRT]: {msg}"); }


    public RRTSearchAlgorithm(Tree<State> tree, Node<State> startNode, Node<State> goalNode, PositionSampler sampler)
    {
        this.tree = tree;
        this.startNode = startNode;
        this.goalNode = goalNode;
        this.sampler = sampler;
    }

    public List<Node<State>> FindPath()
    {
        Log("Finding Path ...");

        Node<State> goalNodeVar = null;
        int nodes = 0;
        int iterations = 0;

        while (goalNodeVar == null)
        {
            if (iterations >= MAX_ITERATIONS)
            {
                Log($"Iteration timeout at {MAX_ITERATIONS}");
                break;
            }

            iterations++;

            // Sample node (sampling a random position within the drivable space. Avoiding the obstacles.) (Sample() method)
            Vector3 randomPosition = sampler.SamplePosition();
            // SpawnDebugSphere(randomPosition, Color.blue);

            // Improvement: Goalbias. Consider sampling bias towards the goal in narrow gaps (to avoid getting stuck in local minima?), along optimal grid paths, etc.
            // RRT is often slow to plan a path as it randomly reaches a defined X_goal position. 
            // To speed up the algorithm, we can bias it toward the goal by modifying the SamplePosition() function.
            if (UnityEngine.Random.value > 0.9) // bias distribution towards the goal, pick the goal point with a 0.9 probability
                randomPosition = goalNode.position;

            Node<State> randomNode = new Node<State>(randomPosition, new State());

            // Get nearest node to the sampled random node (Nearest() method)
            Node<State> nearestNode = tree.findNearestNeighbour(randomNode);

            if (nearestNode == null)
            {
                Log("Could not find any nodes in tree! Make sure root is added.");
                return null;
            }

            // We get a new node where we drive our car from the nearest node towards the random position
            // aka New_state() or Steer() function; new_state(nearestNode, u, Δt), steer(nearestNode, randomNode):
            Node<State> newNode = SimulateModel(randomNode, nearestNode);
            //Node<State> newNode = VelocityConstraint(nearestNode.GetModelState(), next);

            // Check if this newNode node is valid depeding on model constraints
            // Checks if the new node is inside a block/obstacle or if it's too close to a wall/obstacle.
            if (ObstacleFree(nearestNode, newNode))
            {
                nodes++;

                // RRT part:
                // newNode.SetParent(nearestNode);
                // Debug.DrawLine(newNode.position, newNode.GetParent().position, Color.green, 100f);
                // Log($"Added node {nodes} at {newNode.position}");
                // SpawnDebugSphere(newNode.position, Color.red);

                // tree.AddNode(newNode);
                // nearestNode.AddChild(newNode);

                // Log($"Added node {nodes} at {newNode.position}");
                // Debug.DrawLine(newNode.position, newNode.position + newNode.GetModelState().Orientation, Color.yellow, 100f);

                // RRT* part (2011, original): Improves the RRT algorithm by re-wiring the tree where we check if the new node is closer to the other nodes in the tree. ???
                List<Node<State>> nodeNeighbors = ListOfNodesNearCarProximity(newNode.position, radius); // Gathering neighbouring nodes within vicinity of newNode
                tree.AddNode(newNode);
                Node<State> minNode = nearestNode;
                float minCost = Cost(nearestNode) + c(nearestNode, newNode);
                //Debug.Log("Searching for min node");
                foreach (Node<State> nodeNeighbor in nodeNeighbors)
                {
                    float newCost = Cost(nodeNeighbor) + c(nodeNeighbor, newNode);
                    if (ObstacleFree(nodeNeighbor, newNode) && newCost < minCost)
                    {
                        minNode = nodeNeighbor;
                        minCost = newCost;
                    }
                }
                minNode.AddChild(newNode); // OBS! AddChild also sets the parent of newNode to be minNode

                //Debug.Log("Re-wiring");
                foreach (Node<State> nodeNeighbor in nodeNeighbors)
                {
                    float newCost = Cost(newNode) + c(newNode, nodeNeighbor);
                    if (ObstacleFree(newNode, nodeNeighbor) && newCost < Cost(nodeNeighbor))
                    {
                        Node<State> parent = nodeNeighbor.GetParent();
                        parent.RemoveChild(nodeNeighbor);
                        newNode.AddChild(nodeNeighbor);
                    }
                }

                // Check if new node is within range for goal.
                if (Vector3.Distance(newNode.position, goalNode.position) < GOAL_RANGE)
                {
                    Log($"Goal found after {nodes} nodes and {iterations} iterations");
                    goalNodeVar = newNode;
                }
            }
        }

        List<Node<State>> rrt_path = GetPathToRootFromGoalNode(goalNodeVar);
        if (rrt_path != null)
        {
            pathFound = true;
            return rrt_path;
        }
        return null;

    }

    private List<Node<State>> ListOfNodesNearCarProximity(Vector3 position, float radius)
    {
        return tree.GetNodesWithinRadius(position, radius);
    }

    private bool ObstacleFree(Node<State> nodeA, Node<State> nodeB)
    {
        RaycastHit hit;
        return !Physics.SphereCast(
            nodeA.position,
            carHeight,
            nodeB.position - nodeA.position,
            out hit,
            tree.GetDistance(nodeA.position, nodeB.position),
            layerMask: LayerMask.GetMask("Obstacles")
        );
    }

    private float Cost(Node<State> node) // cost of moving from startNode to node (path cost)
    {
        return tree.GetCost(startNode, node);
    }

    private float c(Node<State> nodeA, Node<State> nodeB) // cost of moving from nodeA to nodeB (step cost)
    {
        return tree.GetDistance(nodeA.position, nodeB.position, 2);
    }


    // Compare the state velocity angle to next node position
    // If this angle > max steering angle, then no.
    // private Node<State> VelocityConstraint(State state, Vector3 next)
    // {
    //     Vector3 vel = state.Orientation; // Direction of a car is a velocity of a car.
    //     Vector3 targetDirection = next - state.Position;

    //     float angle = Vector3.Dot(vel.normalized, targetDirection.normalized);
    //     if (angle > MAX_ANGLE)
    //     {
    //         // Debug.DrawLine(next, next + targetDirection.normalized, Color.yellow, 100f);
    //         return new Node<State>(next, new State(next, targetDirection.normalized));
    //     }
    //     // Log($"Angle: {angle}");
    //     return null;
    // }

    // This method is used to simulate the car from the current position (State state) towards the next position (Vector3 next).
    // Incremental simulator: can be constructed by numerical integration of the kinematic model (using, for example Runge-Kutta techniques).
    // https://lavalle.pl/papers/LavKuf01.pdf
    private Node<State> SimulateModel(Node<State> nextNode, Node<State> currentNode) // Node Expansion + takes into account dynamic or kinematic constraints.
    {
        // Simulate car from the state towards next position.
        State currentState = currentNode.GetModelState();
        Vector3 pos = currentState.Position;
        Vector3 dir = currentState.Orientation;

        // Select model type here:
        KinematicModel model = new KinematicModel(tree, pos, dir);
        // DynamicModel model = new DynamicModel(tree, pos, dir);

        // Debug.DrawLine(pos, pos + dir, Color.magenta, 100f);

        for (int step = 0; step < 1000; step++)
        {
            // model.Update(nextNode.position); // updates position and orientation vectors based on next node/position in path
            model.MoveWithinDistance(nextNode, currentNode); // move car within distance of next node
        }

        // Debug.DrawLine(model.position, model.position + model.orientation, Color.yellow, 100f);

        State new_data = model.GetState();
        return new Node<State>(new_data.Position, new_data);
    }

    private bool IsCollisionFree(Node<State> newNode)
    {
        // First check if new node is inside a block/obstacle.
        if (sampler.IsInsideObstacle(newNode.position))
        {
            Log("Inside obstacle");
            return false;
        }
        // Naive apporach: Inflate the walls with a circle with of radius max car length
        if (IsObstacleNearby(newNode.position, INFLATION_RADIUS))
        {
            Log("Obstacle nearby");
            return false;
        }

        return true;
    }

    private bool IsObstacleNearby(Vector3 pos, float radius)
    {
        // To avoid colliding with the car you would normally use layer masks.
        // By not using layer masks we move down the sphere under the terrain
        // by radius number of units. The walls are under the ground which is why
        // this hack works.

        // avoid colliding with terrain
        float epsilon = 1.1f;

        // note that the sphere is a 3D sphere.
        if (Physics.CheckSphere(pos - epsilon * radius * Vector3.up, radius))
            return true;
        return false;
    }

    private List<Node<State>> GetPathToRootFromGoalNode(Node<State> node)
    {
        List<Node<State>> path = new List<Node<State>>();

        while (node.GetParent() != null)
        {
            path.Add(node);
            node = node.GetParent();
        }
        path.Add(node); // node = root here
        path.Reverse(); // we want path from start to goal
        return path;
    }

    // OBS! These functions below can be moved to a util class for reusability

    // private void SpawnDebugSphere(Vector3 pos, Color color)
    // {
    //     GameObject nodeMarker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
    //     nodeMarker.transform.position = pos;
    //     nodeMarker.GetComponent<SphereCollider>().enabled = false;
    //     nodeMarker.GetComponent<Renderer>().material.color = color;
    // }

    // private bool GoalCheck(Vector3 position)
    // {
    //     // TODO: Read goal distance parameter from GameManager
    //     if (Vector3.Distance(position, goalNode.position) < GOAL_RANGE)
    //         return true;

    //     return false;
    // }

    // void OnDrawGizmos()
    // {
    //     // if (automaticIteration)
    //     // {
    //     //     for (int i = 0; i < 10; i++)
    //     //     {
    //     //         if (rrtStar != null)
    //     //             rrtStar.RRTStarIteration();
    //     //     }
    //     // }

    //     Debug.Log("pathFound = " + pathFound);
    //     Debug.Log("Tree = " + tree);

    //     if (pathFound)
    //     {
    //         if (tree != null)
    //         {
    //             DrawTree();
    //         }

    //         // if (tree != null && visualizeBestPath)
    //         // {
    //         //     // path = tree.GetPath();
    //         //     if (RRTpath != null)
    //         //     {
    //         //         DrawPath(RRTpath, Color.blue);
    //         //     }
    //         // }

    //     }
    // }

    // private void DrawTree()
    // {
    //     DrawTreeRecursive(tree.root);
    // }

    // private void DrawTreeRecursive(Node<State> node)
    // {
    //     DrawNodeWithGizmos(node, Color.green);
    //     foreach (Node<State> child in node.children)
    //     {
    //         DrawPathBetweenNodes(node, child, Color.red);
    //         DrawTreeRecursive(child);
    //     }
    // }

    // private void DrawNodeWithGizmos(Node<State> node, Color color)
    // {
    //     Gizmos.color = color;
    //     Vector3 pos = node.position;
    //     pos.y = 0.5f;
    //     Gizmos.DrawSphere(pos, nodeRadius);

    //     // Draw the node's position
    //     //GUIStyle style = new GUIStyle();
    //     //style.normal.textColor = Color.white;
    //     //UnityEditor.Handles.Label(pos, node.position.ToString(), style);

    // }

    // private void DrawPathBetweenNodes(Node<State> nodeA, Node<State> nodeB, Color color)
    // {
    //     Gizmos.color = color;
    //     Vector3 posA = nodeA.position;
    //     Vector3 posB = nodeB.position;
    //     posA.y = 0.5f;
    //     posB.y = 0.5f;
    //     Gizmos.DrawLine(posA, posB);
    // }

}
