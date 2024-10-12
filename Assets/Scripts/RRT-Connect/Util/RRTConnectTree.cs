using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;

public class RRTConnectTree
{
    public RRTConnectNode root;
    public List<RRTConnectNode> nodes;

    public RRTConnectTree(RRTConnectNode root)
    {
        this.root = root;
        this.nodes = new List<RRTConnectNode> { root };
    }

    public RRTConnectNode GetRoot() { return this.root; }
    public List<RRTConnectNode> GetNodes() { return this.nodes; }

    public void AddNode(RRTConnectNode node)
    {
        this.nodes.Add(node);
    }

    public RRTConnectNode findNearestNeighbour(RRTConnectNode node)
    {
        double minDist = double.MaxValue;
        RRTConnectNode closest = null;

        foreach (RRTConnectNode n in this.nodes)
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


    public List<RRTConnectNode> GetNodesWithinRadius(Vector3 position, float radius)
    {
        List<RRTConnectNode> nodesWithinRadius = new List<RRTConnectNode>();
        foreach (RRTConnectNode node in this.nodes)
        {
            if (GetDistance(node.position, position, 2) <= radius)
            {
                nodesWithinRadius.Add(node);
            }
        }
        return nodesWithinRadius;
    }

    public float GetCost(RRTConnectNode fromNode, RRTConnectNode toNode)
    {
        if (fromNode == toNode)
        {
            return 0;
        }
        // Get the shortest path between nodeA and nodeB and return its length
        List<RRTConnectNode> path = FindPath(fromNode, toNode);
        if (path == null)
        {
            return Mathf.Infinity;
        }
        else
        {
            return GetPathLength(path);
        }
    }

    public List<RRTConnectNode> FindPath(RRTConnectNode fromNode, RRTConnectNode toNode)
    {
        List<RRTConnectNode> path = new List<RRTConnectNode>();
        if (fromNode == toNode)
        {
            return path;
        }
        RRTConnectNode currentNode = toNode;
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

    public float GetPathLength(List<RRTConnectNode> path)
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

    public float GetAngle(RRTConnectNode nodeA, RRTConnectNode nodeB)
    {
        return (float)Mathf.Atan2(nodeB.position.z - nodeA.position.z, nodeB.position.x - nodeA.position.x);
    }

}