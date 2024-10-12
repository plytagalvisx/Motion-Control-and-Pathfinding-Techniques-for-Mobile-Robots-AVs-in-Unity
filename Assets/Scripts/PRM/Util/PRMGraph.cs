using System.Collections;
using System.Collections.Generic;
using UnityEngine;
public class PRMGraph
{
    public PRMNode root;
    private List<PRMNode> nodes = new List<PRMNode>();
    private Dictionary<Vector3, PRMNode> nodeDict = new Dictionary<Vector3, PRMNode>();

    public PRMGraph(PRMNode startNode, PRMNode goalNode)
    {
        root = startNode;
        nodes = new List<PRMNode> { startNode, goalNode };
        nodeDict.Add(startNode.position, startNode);
        nodeDict.Add(goalNode.position, goalNode);
    }

    public PRMNode GetRoot() { return root; }

    public List<PRMNode> GetNeighbours(PRMNode node)
    {
        return node.neighbours;
    }

    public void AddNode(PRMNode node)
    {
        nodes.Add(node);
        nodeDict.Add(node.position, node);
    }

    public void RemoveNode(PRMNode node)
    {
        nodes.Remove(node);
        nodeDict.Remove(node.position);
    }

    public List<PRMNode> GetNodes()
    {
        return nodes;
    }

    public PRMNode GetNodeAt(Vector3 position)
    {
        return nodeDict[position];
    }

    public float GetDistance(Vector3 nodeA, Vector3 nodeB, int norm = 1)
    {
        if (norm == 1)
        {
            return Mathf.Abs(nodeA.x - nodeB.x) + Mathf.Abs(nodeA.y - nodeB.y);
        }
        else
        {
            return Mathf.Sqrt(Mathf.Pow(nodeA.x - nodeB.x, 2) + Mathf.Pow(nodeA.y - nodeB.y, 2));
        }
    }

    public List<PRMNode> PRMGetNodesWithinRadius(Vector3 position, float radius)
    {
        List<PRMNode> nodesWithinRadius = new List<PRMNode>();
        foreach (PRMNode node in nodes)
        {
            if (GetDistance(node.position, position, 2) <= radius)
            {
                nodesWithinRadius.Add(node);
            }
        }
        return nodesWithinRadius;
    }

    public bool ContainsNode(PRMNode node)
    {
        return nodes.Contains(node) || nodeDict.ContainsKey(node.position);
    }

    public bool ContainsNodePosition(Vector3 position)
    {
        return nodeDict.ContainsKey(position);
    }


    public PRMNode findNearestNeighbour(PRMNode node)
    {
        double minDist = double.MaxValue;
        PRMNode closest = null;

        foreach (PRMNode n in nodes)
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

}
