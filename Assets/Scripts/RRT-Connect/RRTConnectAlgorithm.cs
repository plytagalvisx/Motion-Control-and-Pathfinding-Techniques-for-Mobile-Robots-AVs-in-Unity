using System;
using System.Collections.Generic;
using System.Data;
using UnityEngine;

public class RRTConnectAlgorithm
{
    private readonly float GOAL_RANGE = 5f;
    private readonly float INFLATION_RADIUS = 4.0f;  // car max length is 4.47

    // DELTA_DISTANCE is a limit to how far a tree can be extended on each step.
    private readonly int DELTA_DISTANCE = 1; // The larger the value, the zigzaggy the path. The smaller the value, the precise the path.
    private readonly int MAX_STEPS = 20; //30;
    private readonly int MAX_ITERATIONS = 400000; // 20000; // Change this value if you want to run for longer time to find a path (reach goal)

    private RRTConnectNode startNode;
    private RRTConnectNode goalNode;

    private readonly PositionSampler sampler;

    public RRTConnectTree tree_start;
    public RRTConnectTree tree_goal;
    private float carHeight = 3f;
    private float carWidth = 3f;
    public float radius = 50f;

    public bool pathFound = false;
    public bool visualizeTree = true;
    public float nodeRadius = 1f;
    private readonly TerrainInfo info;

    private void Log(string msg) { Debug.Log($"[RRT-Connect]: {msg}"); }

    public RRTConnectAlgorithm(TerrainInfo info, RRTConnectTree tree_start, RRTConnectTree tree_goal, RRTConnectNode startNode, RRTConnectNode goalNode, PositionSampler sampler)
    {
        this.info = info;
        this.tree_start = tree_start;
        this.tree_goal = tree_goal;
        this.startNode = startNode;
        this.goalNode = goalNode;
        this.sampler = sampler;
    }

    public List<RRTConnectNode> FindPathRRTConnect()
    {
        int n = 0;
        while (n < MAX_ITERATIONS)
        {
            n++;
            Vector3 randomPosition = sampler.SamplePosition();
            RRTConnectNode randomNode = new RRTConnectNode(randomPosition);
            RRTConnectNode extended_s_node = new RRTConnectNode(Vector3.zero);
            RRTConnectNode extended_g_node = new RRTConnectNode(Vector3.zero);

            // Improvement: Nonuniform sampling strategies: goal biasing
            if (UnityEngine.Random.value > 0.9) // probability of using goal biasing
            {
                if (UnityEngine.Random.value <= 0.5) // unbiased coin flip whether to extend toward the start or toward the goal
                {
                    randomNode = RandomVertex(tree_goal);
                    extended_s_node = ExtendTree(tree_start, randomNode); // extend start tree at most DELTA_DISTANCE towards randomNode
                }
                else
                {
                    randomNode = RandomVertex(tree_start);
                    extended_g_node = ExtendTree(tree_goal, randomNode);  // extend goal tree at most DELTA_DISTANCE towards randomNode
                }
            }
            else
            {
                extended_s_node = ExtendTree(tree_start, randomNode); // extend start tree at most DELTA_DISTANCE towards randomNode
                extended_g_node = ExtendTree(tree_goal, randomNode);  // extend goal tree at most DELTA_DISTANCE towards randomNode
            }

            if ((Vector3.Distance(extended_s_node.position, extended_g_node.position) <= DELTA_DISTANCE) && VisibleRecurse(extended_s_node, extended_g_node)) // trees are close enough
            {
                return GetPathFromStartToGoal(extended_s_node, extended_g_node);
            }
        }
        return null;
    }

    public RRTConnectNode RandomVertex(RRTConnectTree tree)
    {
        return tree.GetNodes()[UnityEngine.Random.Range(0, tree.GetNodes().Count)];
    }

    public RRTConnectNode ExtendTree(RRTConnectTree tree, RRTConnectNode randomNode)
    {
        RRTConnectNode nearestNode = tree.findNearestNeighbour(randomNode);
        Vector3 newNodePos = nearestNode.position + Mathf.Min(1, DELTA_DISTANCE / Vector3.Distance(randomNode.position, nearestNode.position)) * (randomNode.position - nearestNode.position);
        RRTConnectNode newNode = new RRTConnectNode(newNodePos);
        if (VisibleRecurse(nearestNode, newNode))
        {
            tree.AddNode(newNode);
            nearestNode.AddChild(newNode);
            return newNode;
        }
        return nearestNode;
    }

    public bool Feasible(RRTConnectNode node)
    {
        int i = info.get_i_index(node.position.x);
        int j = info.get_j_index(node.position.z);
        // 0.0 = air/free, 1.0 = block/obstacle
        if (info.traversability[i, j] < 0.5f)
            return true; // free space
        return false; // obstacle
    }

    private bool VisibleRecurse(RRTConnectNode nodeA, RRTConnectNode nodeB, float distEpsilon = 0.001f)
    {
        if (Vector3.Distance(nodeA.position, nodeB.position) < distEpsilon)
        {
            return true;
        }
        RRTConnectNode m = new RRTConnectNode((nodeA.position + nodeB.position) / 2);
        if (!Feasible(m))
        {
            return false;
        }
        return VisibleRecurse(nodeA, m) && VisibleRecurse(m, nodeB);
    }

    // private List<RRTConnectNode> ListOfNodesNearCarProximity(Vector3 position, float radius)
    // {
    //     return tree.GetNodesWithinRadius(position, radius);
    // }

    // private bool ObstacleFree(RRTConnectNode nodeA, RRTConnectNode nodeB)
    // {
    //     RaycastHit hit;
    //     return !Physics.SphereCast(
    //         nodeA.position,
    //         carHeight,
    //         nodeB.position - nodeA.position,
    //         out hit,
    //         Vector3.Distance(nodeA.position, nodeB.position),
    //         layerMask: LayerMask.GetMask("Obstacles")
    //     );
    // }

    // private float Cost(RRTConnectNode node) // cost of moving from startNode to node (path cost)
    // {
    //     return tree.GetCost(startNode, node);
    // }

    // private float c(RRTConnectNode nodeA, RRTConnectNode nodeB) // cost of moving from nodeA to nodeB (step cost)
    // {
    //     return Vector3.Distance(nodeA.position, nodeB.position);
    // }

    private bool IsCollisionFree(RRTConnectNode newNode)
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

    // For basic RRT or RRT*:
    private List<RRTConnectNode> GetPathToRootFromGoalNode(RRTConnectNode node)
    {
        List<RRTConnectNode> path = new List<RRTConnectNode>();

        while (node.GetParent() != null)
        {
            Debug.Log("node: " + node.position);
            path.Add(node);
            node = node.GetParent();
        }
        path.Add(node); // node = root here
        path.Reverse(); // we want path from start to goal
        return path;
    }

    // For RRT-Connect:
    private List<RRTConnectNode> GetPathFromStartToGoal(RRTConnectNode startNode, RRTConnectNode goalNode)
    {
        List<RRTConnectNode> path = new List<RRTConnectNode>();

        path.Add(goalNode); // Add the root node
        // Get path from goal node to root
        while (goalNode != null)
        {
            path.Add(goalNode);
            goalNode = goalNode.GetParent();
        }
        path.Reverse(); // Reverse the path from goal to root

        // Get path from start node to root
        List<RRTConnectNode> startToRootPath = new List<RRTConnectNode>();
        startToRootPath.Add(startNode); // Add the root node
        while (startNode != null)
        {
            startToRootPath.Add(startNode);
            startNode = startNode.GetParent();
        }

        // Combine the two paths
        path.AddRange(startToRootPath.GetRange(0, startToRootPath.Count - 1)); // exclude the root node

        return path;
    }

}
