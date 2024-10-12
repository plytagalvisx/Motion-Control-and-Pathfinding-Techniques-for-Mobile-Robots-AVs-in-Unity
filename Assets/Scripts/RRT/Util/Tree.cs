using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;

public class Tree<T>
{
    public Node<T> root;
    public Node<T> goal;
    private List<Node<T>> nodes;
    public Tree(Node<T> root, Node<T> goal) //, T model)
    {
        this.root = root;
        this.goal = goal;
        nodes = new List<Node<T>> { root };
    }

    public Node<T> GetRoot() { return root; }
    public List<Node<T>> GetNodes() { return nodes; }

    public void AddNode(Node<T> node)
    {
        nodes.Add(node);
    }

    public Node<T> findNearestNeighbour(Node<T> node)
    {
        double minDist = double.MaxValue;
        Node<T> closest = null;

        foreach (Node<T> n in nodes)
        {
            double dist = Vector3.Distance(node.position, n.position);
            if (dist < minDist)
            {
                minDist = dist;
                closest = n;
            }
        }

        return closest;
    }

    public float GetDistance(Vector3 positionA, Vector3 positionB, int norm = 1)
    {
        if (norm == 1)
        {
            return Mathf.Abs(positionA.x - positionB.x) + Mathf.Abs(positionA.z - positionB.z);
        }
        else
        {
            return Mathf.Sqrt(Mathf.Pow(positionA.x - positionB.x, 2) + Mathf.Pow(positionA.z - positionB.z, 2));
        }
    }


    public List<Node<T>> GetNodesWithinRadius(Vector3 position, float radius)
    {
        List<Node<T>> nodesWithinRadius = new List<Node<T>>();
        foreach (Node<T> node in nodes)
        {
            if (GetDistance(node.position, position, 2) <= radius)
            {
                nodesWithinRadius.Add(node);
            }
        }
        return nodesWithinRadius;
    }

    public float GetCost(Node<T> fromNode, Node<T> toNode)
    {
        if (fromNode == toNode)
        {
            return 0;
        }
        // Get the shortest path between nodeA and nodeB and return its length
        List<Node<T>> path = FindPath(fromNode, toNode);
        if (path == null)
        {
            return Mathf.Infinity;
        }
        else
        {
            return GetPathLength(path);
        }
    }

    public List<Node<T>> FindPath(Node<T> fromNode, Node<T> toNode)
    {
        List<Node<T>> path = new List<Node<T>>();
        if (fromNode == toNode)
        {
            return path;
        }
        Node<T> currentNode = toNode;
        while (currentNode != fromNode)
        {
            path.Add(currentNode);
            currentNode = currentNode.GetParent();
        }
        if (path.Count == 0)
        {
            return null;
        }
        path.Add(fromNode);
        path.Reverse();
        return path;
    }

    public float GetPathLength(List<Node<T>> path)
    {
        if (path.Count == 0)
        {
            return 0;
        }

        float length = 0;
        for (int i = 0; i < path.Count - 1; i++)
        {
            length += GetDistance(path[i].position, path[i + 1].position, 2);
        }
        return length;
    }

    public float GetAngle(Node<T> nodeA, Node<T> nodeB)
    {
        return (float)Mathf.Atan2(nodeB.position.z - nodeA.position.z, nodeB.position.x - nodeA.position.x);
    }

}