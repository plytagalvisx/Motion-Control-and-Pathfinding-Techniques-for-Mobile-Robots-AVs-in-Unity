using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class Graph2
{
    private List<Node2> nodes = new List<Node2>();
    private Dictionary<Vector2, Node2> nodeDict = new Dictionary<Vector2, Node2>();

    public Graph2() { }

    public List<Node2> GetNeighbours(Node2 node)
    {
        return node.neighbours;
    }

    public void AddNode(Node2 node)
    {
        nodes.Add(node);
        nodeDict.Add(node.position, node);
    }

    public void RemoveNode(Node2 node)
    {
        nodes.Remove(node);
    }

    public List<Node2> GetNodes()
    {
        return nodes;
    }

    public Node2 GetNodeAt(Vector2 position)
    {
        return nodeDict[position];
    }

    public Node2 FindNodeIter(Vector2 pos) // Find the node that contains a point (iterative)
    {
        foreach (Node2 node in nodes)
        {
            if (node.in_area(pos))
            {
                return node;
            }
        }
        return nodes[0];
    }

    public float GetDistance(Node2 nodeA, Node2 nodeB, int norm = 1)
    {
        if (norm == 1)
        {
            return Mathf.Abs(nodeA.position.x - nodeB.position.x) + Mathf.Abs(nodeA.position.y - nodeB.position.y);
        }
        else
        {
            return Mathf.Sqrt(Mathf.Pow(nodeA.position.x - nodeB.position.x, 2) + Mathf.Pow(nodeA.position.y - nodeB.position.y, 2));
        }
    }

    public float GetAngle(Node2 nodeA, Node2 nodeB)
    {
        return Mathf.Atan2(nodeB.position.y - nodeA.position.y, nodeB.position.x - nodeA.position.x);
    }

    public float GetCost(Node2 nodeA, Node2 nodeB) // Cost of moving from nodeA to nodeB
    {
        // Get the shortest path between nodeA and nodeB and return its length
        return GetPathLength(FindPathDjikstra(nodeA, nodeB)); // For A*/Djikstra (graph) based search

    }

    public Node2 GetClosestNode(Vector2 position)
    {
        Node2 closestNode = null;
        float closestDistance = Mathf.Infinity;
        foreach (Node2 node in nodes)
        {
            float distance = GetDistance(node, new Node2(position));
            if (distance < closestDistance)
            {
                closestDistance = distance;
                closestNode = node;
            }
        }
        return closestNode;
    }

    public float GetPathLength(List<Node2> path)
    {
        float length = 0;
        for (int i = 0; i < path.Count - 1; i++)
        {
            length += GetDistance(path[i], path[i + 1], 2);
        }
        return length;
    }

    public List<Node2> GetNodesWithinRadius(Vector2 position, float radius)
    {
        List<Node2> nodesWithinRadius = new List<Node2>();
        foreach (Node2 node in nodes)
        {
            if (GetDistance(node, new Node2(position)) <= radius)
            {
                nodesWithinRadius.Add(node);
            }
        }
        return nodesWithinRadius;
    }

    public List<Node2> FindPathDjikstra(Node2 startNode, Node2 endNode) // For A*/Djikstra (graph) based search
    {

        // Create a dictionary of nodes and their distances from the start node
        Dictionary<Node2, float> distances = new Dictionary<Node2, float>();
        foreach (Node2 node in nodes)
        {
            distances.Add(node, Mathf.Infinity);
        }
        distances[startNode] = 0;

        // Create a dictionary of nodes and their previous nodes
        Dictionary<Node2, Node2> previous = new Dictionary<Node2, Node2>();
        foreach (Node2 node in nodes)
        {
            previous.Add(node, null);
        }

        // Create a list of unvisited nodes
        List<Node2> unvisited = new List<Node2>();
        foreach (Node2 node in nodes)
        {
            unvisited.Add(node);
        }

        // While there are unvisited nodes
        while (unvisited.Count > 0)
        {
            // Find the unvisited node with the smallest distance
            Node2 smallestDistanceNode = null;
            float smallestDistance = Mathf.Infinity;
            foreach (Node2 node in unvisited)
            {
                if (distances[node] < smallestDistance)
                {
                    smallestDistance = distances[node];
                    smallestDistanceNode = node;
                }
            }

            // If the smallest distance node is the end node, we're done
            if (smallestDistanceNode == endNode)
            {
                break;
            }

            // Remove the smallest distance node from the unvisited list
            unvisited.Remove(smallestDistanceNode);

            // For each neighbour of the smallest distance node
            foreach (Node2 neighbour in smallestDistanceNode.neighbours)
            {
                // If the neighbour is unvisited
                if (unvisited.Contains(neighbour))
                {
                    // Calculate the distance to the neighbour
                    float distanceToNeighbour = distances[smallestDistanceNode] + GetDistance(smallestDistanceNode, neighbour);

                    // If the distance to the neighbour is less than the current distance to the neighbour
                    if (distanceToNeighbour < distances[neighbour])
                    {
                        // Update the distance to the neighbour
                        distances[neighbour] = distanceToNeighbour;

                        // Update the previous node of the neighbour
                        previous[neighbour] = smallestDistanceNode;
                    }
                }
            }
        }

        // Create a list of nodes to return
        List<Node2> path = new List<Node2>();
        // Start at the end node
        Node2 currentNode = endNode;
        // While the current node is not the start node
        while (currentNode != startNode)
        {
            // Add the current node to the path
            path.Add(currentNode);
            // Set the current node to the previous node
            currentNode = previous[currentNode];
        }

        // Add the start node to the path
        path.Add(startNode);
        // Reverse the path
        path.Reverse();
        // Return the path
        return path;
    }

    public List<Vector3> ConvertPathToVector3(List<Node2> path)
    {
        List<Vector3> vector3Path = new List<Vector3>();
        foreach (Node2 node in path)
        {
            Vector3 position = new Vector3(node.position.x, 0, node.position.y);
            vector3Path.Add(position);
        }
        return vector3Path;
    }

    public void LogGraph()
    {
        Debug.Log("===== Graph =====");
        foreach (Node2 node in nodes)
        {
            Debug.Log("Node2: " + node.position);
            foreach (Node2 neighbour in node.neighbours)
            {
                Debug.Log("Neighbour: " + neighbour.position);
            }
        }
        Debug.Log("===== End Graph =====");
    }
}