using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Tree2
{
    public TreeNode root;
    public TreeNode goal;
    public List<TreeNode> nodes = new List<TreeNode>();
    private Dictionary<Vector2, TreeNode> nodeDict = new Dictionary<Vector2, TreeNode>();
    public List<TreeNode> path = new List<TreeNode>();

    public Tree2(TreeNode Root, TreeNode Goal)
    {
        root = Root;
        goal = Goal;
        root.SetParent(root);
        this.AddNode(root);
    }

    public void AddNode(TreeNode node)
    {
        if (!IsNode(node))
        {
            nodes.Add(node);
            nodeDict.Add(node.position, node);
        }
        if (node.position == goal.position)
        {
            goal = node;
        }
    }

    public void RemoveNode(TreeNode node)
    {
        nodes.Remove(node);
        nodeDict.Remove(node.position);
    }

    public List<TreeNode> GetNodes()
    {
        return nodes;
    }

    public bool IsNode(TreeNode node)
    {
        return nodeDict.ContainsKey(node.position);
    }

    public List<TreeNode> GetPath()
    {
        if (goal.GetParent() != null)
            return FindPath(root, goal);
        return null;
    }

    public TreeNode GetNodeAt(Vector2 position)
    {
        return nodeDict[position];
    }

    public float GetDistance(TreeNode nodeA, TreeNode nodeB, int norm = 1)
    {
        return GetDistance(nodeA.position, nodeB.position, norm);
    }

    public float GetDistance(Vector2 positionA, Vector2 positionB, int norm = 1)
    {
        if (norm == 1) // Manhattan distance
        {
            return Mathf.Abs(positionA.x - positionB.x) + Mathf.Abs(positionA.y - positionB.y);
        }
        else // Euclidean distance
        {
            return Mathf.Sqrt(Mathf.Pow(positionA.x - positionB.x, 2) + Mathf.Pow(positionA.y - positionB.y, 2));
        }
    }

    public float GetAngle(TreeNode nodeA, TreeNode nodeB)
    {
        return Mathf.Atan2(nodeB.position.y - nodeA.position.y, nodeB.position.x - nodeA.position.x);
    }

    public TreeNode GetClosestNode(Vector2 position) // aka nearest neighbour
    {
        TreeNode closestNode = null;
        float closestDistance = Mathf.Infinity;
        foreach (TreeNode node in nodes)
        {
            float distance = GetDistance(node, new TreeNode(position), 2);
            if (distance < closestDistance)
            {
                closestDistance = distance;
                closestNode = node;
            }
        }
        return closestNode;
    }

    public float GetCost(TreeNode fromNode, TreeNode toNode) // aka edge cost
    {
        if (fromNode == toNode)
        {
            return 0;
        }
        // Get the shortest path between nodeA and nodeB and return its length
        List<TreeNode> path = FindPath(fromNode, toNode); // For RRT (tree) based search
        if (path == null)
        {
            return Mathf.Infinity;
        }
        else
        {
            return GetPathLength(path);
        }
    }

    public List<TreeNode> FindPath(TreeNode fromNode, TreeNode toNode) // For RRT (tree) based search
    {
        List<TreeNode> path = new List<TreeNode>();
        if (fromNode == toNode)
        {
            return path;
        }
        TreeNode currentNode = toNode;
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

    public float GetPathLength(List<TreeNode> path)
    {
        if (path.Count == 0)
        {
            return 0;
        }

        float length = 0;
        for (int i = 0; i < path.Count - 1; i++)
        {
            length += GetDistance(path[i], path[i + 1], 2);
        }
        return length;
    }

    public List<TreeNode> GetNodesWithinRadius(Vector2 position, float radius)
    {
        List<TreeNode> nodesWithinRadius = new List<TreeNode>();
        foreach (TreeNode node in nodes)
        {
            if (GetDistance(node.position, position, 2) <= radius)
            {
                nodesWithinRadius.Add(node);
            }
        }
        return nodesWithinRadius;
    }

    public List<Vector3> ConvertPathToVector3(List<TreeNode> path)
    {
        List<Vector3> vector3Path = new List<Vector3>();
        foreach (TreeNode node in path)
        {
            vector3Path.Add(new Vector3(node.position.x, node.position.y, 0));
        }
        return vector3Path;
    }

    public void LogRoot()
    {
        root.LogPosition();
    }

    public void LogNodes()
    {
        foreach (TreeNode node in nodes)
        {
            node.LogPosition();
        }
    }

    public void LogPath(List<TreeNode> path)
    {
        Debug.Log("Path");
        foreach (TreeNode node in path)
        {
            node.LogPosition();
        }
    }


}