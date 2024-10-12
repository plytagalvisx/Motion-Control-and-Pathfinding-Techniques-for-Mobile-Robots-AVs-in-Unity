using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;

public class RRTConnectNode
{
    public Vector3 position; // Position in world space
    public RRTConnectNode parent;
    public List<RRTConnectNode> children = new List<RRTConnectNode>();

    public RRTConnectNode(Vector3 position)
    {
        this.parent = null;
        this.position = position;
    }

    public RRTConnectNode(Vector3 position, RRTConnectNode parent)
    {
        this.parent = parent;
        this.position = position;
    }

    public void AddChild(RRTConnectNode newChild)
    {
        // if not already child
        if (!this.children.Contains(newChild))
        {
            this.children.Add(newChild);
            newChild.SetParent(this); // the node that called AddChild() is the parent of the node that is being added
        }
    }

    public void RemoveChild(RRTConnectNode child)
    {
        this.children.Remove(child);
        child.SetParent(null);
    }

    public List<RRTConnectNode> GetChildren() { return this.children; }

    public bool IsChild(RRTConnectNode child)
    {
        return this.children.Contains(child);
    }

    public void SetParent(RRTConnectNode newParent)
    {
        if (newParent != null)
        {
            this.parent = newParent;
        }
        else
        {
            this.parent = null;
        }
    }
    public RRTConnectNode GetParent() { return this.parent; }
    public void RemoveParent() { this.parent = null; }
    public void AddParent(RRTConnectNode newParent)
    {
        this.parent = newParent;
        newParent.AddChild(this);
    }

}
