using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RRTStar
{
    public Tree2 tree;
    public Vector3 direction;
    public TerrainManager terrainManager;
    public float radius = 50;
    public float maxDistance = 40;
    public float probOfGoalNodeInSampling = 0.1f;


    private readonly int MAX_ITERATIONS = 400000;
    private readonly float GOAL_RANGE = 5f;
    private readonly float INFLATION_RADIUS = 4.0f;     // car max length is 4.47
    private float carHeight = 3f;
    private float carWidth = 3f;

    private TreeNode startNode;
    private TreeNode goalNode;
    public RRTStar(Tree2 tree, Vector3 direction, TreeNode startNode, TreeNode goalNode, TerrainManager terrainManager)
    {
        this.tree = tree;
        this.direction = direction;
        this.terrainManager = terrainManager;
        this.startNode = startNode;
        this.goalNode = goalNode;
    }

    // Approach 1: Compute the path and return it as List<TreeNode>.
    public List<TreeNode> RRTStarIterationReturnPath()
    {
        TreeNode randomNode;
        TreeNode nearestNode;
        TreeNode newNode;

        TreeNode goalnode = null;
        int iterations = 0;

        while (goalnode == null)
        {
            if (iterations >= MAX_ITERATIONS)
            {
                Debug.Log($"Iteration timeout at {MAX_ITERATIONS}");
                break;
            }
            iterations++;

            randomNode = Sample();
            nearestNode = Nearest(randomNode.position);

            if (nearestNode == null)
            {
                Debug.Log("Could not find any nodes in tree! Make sure root is added.");
                return null;
            }

            newNode = Steer(randomNode, nearestNode);

            if (ObstacleFree(nearestNode, newNode))
            {
                // RRT part:
                // tree.AddNode(newNode);
                // nearestNode.AddChild(newNode);

                // RRT* part:
                List<TreeNode> nodeNeighbors = Near(newNode.position, radius);
                tree.AddNode(newNode);
                TreeNode minNode = nearestNode;
                float minCost = Cost(nearestNode) + c(nearestNode, newNode);
                //Debug.Log("Searching for min node");
                foreach (TreeNode nodeNeighbor in nodeNeighbors)
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
                foreach (TreeNode nodeNeighbor in nodeNeighbors)
                {
                    float newCost = Cost(newNode) + c(newNode, nodeNeighbor);
                    if (ObstacleFree(newNode, nodeNeighbor) && newCost < Cost(nodeNeighbor))
                    {
                        TreeNode parent = nodeNeighbor.GetParent();
                        parent.RemoveChild(nodeNeighbor);
                        newNode.AddChild(nodeNeighbor);
                    }
                }

                if (Vector3.Distance(newNode.position, tree.goal.position) < GOAL_RANGE)
                {
                    Debug.Log($"Goal found after {iterations} iterations");
                    goalnode = newNode;
                }
            }
        }

        return GetPathToRootFromGoalNode(goalnode);
    }

    // Approach 2: Compute the path and update the Tree class object.
    public void RRTStarIterationUpdateTree()
    {
        TreeNode randomNode;
        TreeNode nearestNode;
        TreeNode newNode;
        int i = 0;
        do
        {
            randomNode = Sample();
            nearestNode = Nearest(randomNode.position);
            newNode = Steer(randomNode, nearestNode);
            i++;
        } while (!ObstacleFree(nearestNode, newNode) && i < 1000);

        if (ObstacleFree(nearestNode, newNode))
        {
            // RRT part:
            tree.AddNode(newNode);
            nearestNode.AddChild(newNode);

            // RRT* part:
            // List<TreeNode> nodeNeighbors = Near(newNode.position, radius);
            // tree.AddNode(newNode);
            // TreeNode minNode = nearestNode;
            // float minCost = Cost(nearestNode) + c(nearestNode, newNode);
            // // Searching for min node:
            // foreach (TreeNode nodeNeighbor in nodeNeighbors)
            // {
            //     float newCost = Cost(nodeNeighbor) + c(nodeNeighbor, newNode);
            //     if (ObstacleFree(nodeNeighbor, newNode) && newCost < minCost)
            //     {
            //         minNode = nodeNeighbor;
            //         minCost = newCost;
            //     }
            // }
            // minNode.AddChild(newNode);

            // // Re-wiring for RRT*:
            // foreach (TreeNode nodeNeighbor in nodeNeighbors)
            // {
            //     float newCost = Cost(newNode) + c(newNode, nodeNeighbor);
            //     if (ObstacleFree(newNode, nodeNeighbor) && newCost < Cost(nodeNeighbor))
            //     {
            //         TreeNode parent = nodeNeighbor.GetParent();
            //         parent.RemoveChild(nodeNeighbor);
            //         newNode.AddChild(nodeNeighbor);
            //     }
            // }
        }
    }

    public int iter = 0;
    private List<TreeNode> GetPathToRootFromGoalNode(TreeNode node)
    {
        List<TreeNode> path = new List<TreeNode>();
        TreeNode currentNode = node;
        while (currentNode != null)
        {
            if (iter == 1000)
            {
                break;
            }
            iter++;

            path.Add(currentNode);
            currentNode = currentNode.GetParent();
        }
        path.Reverse();
        return path;
    }

    private TreeNode Sample()
    {
        return GetRandomNode();
    }

    private TreeNode Nearest(Vector2 position)
    {
        return tree.GetClosestNode(position);
    }

    private List<TreeNode> Near(Vector2 position, float radius)
    {
        return tree.GetNodesWithinRadius(position, radius);
    }

    private TreeNode Steer(TreeNode nodeA, TreeNode nodeB)
    {
        // TreeNode closeNode = MoveWithinDistance(nodeA.GetCopy(), nodeB);
        TreeNode closeNode = MoveWithinDistance(nodeA, nodeB);
        return closeNode;
    }

    private float Cost(TreeNode node)
    {
        return tree.GetCost(startNode, node);
    }

    private float c(TreeNode nodeA, TreeNode nodeB)
    {
        return GetDistance(nodeA, nodeB);
    }

    private float GetDistance(TreeNode nodeA, TreeNode nodeB)
    {
        return tree.GetDistance(nodeA, nodeB, 2);
    }

    private TreeNode findNearestNode(TreeNode node)
    {
        return tree.GetClosestNode(node.position);
    }

    private TreeNode GetRandomNode()
    {
        // With probability probOfGoalNodeInSampling, return the goal node
        if (Random.Range(0f, 1f) < probOfGoalNodeInSampling)
        {
            return new TreeNode(goalNode.position);
        }
        // Get the bounding box of the terrain
        Vector2 maxPoint = new Vector2();
        maxPoint.x = terrainManager.myInfo.x_high;
        maxPoint.y = terrainManager.myInfo.z_high;
        //maxPoint.x = Mathf.Abs(terrainManager.myInfo.x_high - terrainManager.myInfo.x_low) * terrainManager.myInfo.x_N;
        //maxPoint.y = Mathf.Abs(terrainManager.myInfo.z_high - terrainManager.myInfo.z_low) * terrainManager.myInfo.z_N;
        Vector2 minPoint = new Vector2();
        minPoint.x = terrainManager.myInfo.x_low;
        minPoint.y = terrainManager.myInfo.z_low;

        // Get a random point within the bounding box
        Vector2 randomPoint = new Vector2(Random.Range(minPoint.x, maxPoint.x), Random.Range(minPoint.y, maxPoint.y));

        // Create a new node at the random point
        TreeNode node = new TreeNode(randomPoint);

        return node;
    }

    // OBS! I don't think this method is efficient for this RRT in terms of not checking the path between the two nodes; nearestNode and newNode.
    // Here we are only checking if the newNode is inside an obstacle or if there is an obstacle nearby.
    // Instead we should check if the path between the two nodes is traversable since we drive on a discretized grid/terrain/path.
    // In regard to this, ObstaceFree() method should be used instead of IsCollisionFree().
    private bool IsCollisionFree(TreeNode newNode)
    {
        // First check if new node is inside a block/obstacle.
        if (IsInsideObstacle(newNode.position))
        {
            Debug.Log("Inside obstacle");
            return false;
        }
        // Naive apporach: Inflate the walls with a circle with of radius max car length
        if (IsObstacleNearby(newNode.position, INFLATION_RADIUS))
        {
            Debug.Log("Obstacle nearby");
            return false;
        }

        return true;
    }

    public bool IsInsideObstacle(Vector2 position)
    {
        int i = terrainManager.myInfo.get_i_index(position.x);
        int j = terrainManager.myInfo.get_j_index(position.y);

        // 0.0 = air/free, 1.0 = block/obstacle
        if (terrainManager.myInfo.traversability[i, j] < 0.5f)
            return false;

        return true;
    }

    private bool IsObstacleNearby(Vector2 pos, float radius)
    {
        // To avoid colliding with the car you would normally use layer masks.
        // By not using layer masks we move down the sphere under the terrain
        // by radius number of units. The walls are under the ground which is why
        // this hack works.

        // avoid colliding with terrain
        float epsilon = 1.1f;

        // note that the sphere is a 3D sphere.
        Vector3 position = new Vector3(pos.x, 0, pos.y);
        if (Physics.CheckSphere(position - epsilon * radius * Vector3.up, radius))
            return true;
        return false;
    }


    private bool ObstacleFree(TreeNode nodeA, TreeNode nodeB)
    {
        RaycastHit hit;
        return !Physics.SphereCast(
            new Vector3(nodeA.position.x, 0, nodeA.position.y),
            carHeight,
            new Vector3(nodeB.position.x - nodeA.position.x, 0, nodeB.position.y - nodeA.position.y),
            out hit,
            GetDistance(nodeA, nodeB),
            layerMask: LayerMask.GetMask("Obstacles")
        );
    }

    // Moves movableNode to be within distance of fixedNode
    // private TreeNode MoveWithinDistance(TreeNode randomNode, TreeNode nearestNode)
    // {
    //     if (IsWithinDistance(randomNode, nearestNode))
    //         return randomNode;
    //     // Get angle between nodes
    //     float angle = tree.GetAngle(randomNode, nearestNode);

    //     // Set newNode to be the same angle as earlier, but with distance of maxDistance
    //     TreeNode newNode = new TreeNode(Vector2.zero);
    //     newNode.position = new Vector2(
    //         nearestNode.position.x + maxDistance * Mathf.Cos(angle),
    //         nearestNode.position.y + maxDistance * Mathf.Sin(angle)
    //     );

    //     return newNode;
    // }

    private TreeNode MoveWithinDistance(TreeNode movableNode, TreeNode fixedNode)
    {
        if (IsWithinDistance(movableNode, fixedNode))
            return movableNode;
        // Get angle between nodes
        float angle = tree.GetAngle(movableNode, fixedNode); // aka heading (theta)

        // Set movableNode to be the same angle as earlier, but with distance of maxDistance
        movableNode.position = new Vector2(
            fixedNode.position.x + maxDistance * Mathf.Cos(angle),
            fixedNode.position.y + maxDistance * Mathf.Sin(angle)
        );

        //movableNode.position = Vector2.MoveTowards(movableNode.position, fixedNode.position, maxDistance);
        return movableNode;
    }


    private bool IsWithinDistance(TreeNode nodeA, TreeNode nodeB)
    {
        return GetDistance(nodeA, nodeB) <= maxDistance;
    }
}