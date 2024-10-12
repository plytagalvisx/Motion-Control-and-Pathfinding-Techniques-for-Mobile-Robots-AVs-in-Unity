using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TreeNode // Behaves like a Node<StateData> in the RRT class
{
    public Vector2 position;
    public List<TreeNode> children = new List<TreeNode>();
    public TreeNode parent;
    public float cost;

    public TreeNode(Vector2 pos)
    {
        position = pos;
    }

    public void AddChild(TreeNode node)
    {
        // if not already child
        if (!children.Contains(node))
        {
            children.Add(node);
            node.SetParent(this); // the node that called AddChild() is the parent of the node that is being added
        }
    }

    public void RemoveChild(TreeNode node)
    {
        children.Remove(node);
        node.SetParent(null);
    }

    public List<TreeNode> GetChildren()
    {
        return children;
    }

    public void SetParent(TreeNode node) // is only being called in this TreeNode class, so it's not needed to be called in the RRT class.
                                         // calling AddChild() in RRT is enough.
    {
        if (node != null)
        {
            parent = node;
            cost = node.cost + 1;
        }
        else
        {
            parent = null;
            cost = 0;
        }
    }

    public TreeNode GetParent()
    {
        return parent;
    }

    public void RemoveParent()
    {
        parent = null;
    }

    public void AddParent(TreeNode node)
    {
        parent = node;
        node.AddChild(this);
    }

    public bool IsChild(TreeNode node)
    {
        return children.Contains(node);
    }

    public TreeNode GetCopy()
    {
        TreeNode copy = new TreeNode(position);
        copy.parent = parent;
        copy.children = children;
        return copy;
    }

    public void LogPosition()
    {
        Debug.Log("Node: " + position.x + ", " + position.y);
    }

    public void LogChildren()
    {
        Debug.Log("Node: " + position.x + ", " + position.y);
        foreach (TreeNode child in children)
        {
            child.LogPosition();
        }
    }

    public void LogParent()
    {
        Debug.Log("Node: " + position.x + ", " + position.y);
        if (parent != null)
        {
            parent.LogPosition();
        }
    }
}