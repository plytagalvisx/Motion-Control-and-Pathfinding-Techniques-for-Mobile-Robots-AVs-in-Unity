using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;

public class Node<T>
{
    public Vector3 position; // Position in world space
    private Node<T> parent;
    private T model; // What type of simulation model (Car or Drone)
    public List<Node<T>> children = new List<Node<T>>();

    public Node(Vector3 position)
    {
        this.position = position;
    }

    public Node(Vector3 position, T model)
    {
        parent = null;
        this.position = position;
        this.model = model; // In this case, it's a State type object containing (position and orientation vectors) (aka the pose)
    }

    public Node(Vector3 position, T model, Node<T> parent)
    {
        this.parent = parent;
        this.position = position;
        this.model = model; // aka the state/pose of the robot
    }

    public void AddChild(Node<T> newChild)
    {
        // if not already child
        if (!children.Contains(newChild))
        {
            children.Add(newChild);
            newChild.SetParent(this); // the node that called AddChild() is the parent of the node that is being added
        }
    }

    public void RemoveChild(Node<T> child)
    {
        children.Remove(child);
        child.SetParent(null);
    }

    public List<Node<T>> GetChildren() { return children; }

    public bool IsChild(Node<T> child)
    {
        return children.Contains(child);
    }

    public void SetParent(Node<T> newParent)
    {
        if (newParent != null)
        {
            parent = newParent;
        }
        else
        {
            parent = null;
        }
    }
    public Node<T> GetParent() { return parent; }
    public void RemoveParent() { parent = null; }
    public void AddParent(Node<T> newParent)
    {
        parent = newParent;
        newParent.AddChild(this);
    }

    public T GetModelState() { return model; }
}
