using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PRMNode
{
    public Vector3 position;
    public float distance;
    public List<PRMNode> neighbours = new List<PRMNode>();

    public PRMNode(Vector3 pos)
    {
        position = pos;
    }

    // Override Equals method to define equality comparison
    public override bool Equals(object obj)
    {
        if (obj == null || GetType() != obj.GetType())
        {
            return false;
        }

        PRMNode other = (PRMNode)obj;
        return position.Equals(other.position); // Compare positions for equality
    }

    public void AddNeighbour(PRMNode node)
    {
        neighbours.Add(node);
    }

    public void AddNeighbourSymmetric(PRMNode node)
    {
        neighbours.Add(node);
        node.neighbours.Add(this);
    }

    public bool hasNeighbour(PRMNode node)
    {
        return neighbours.Contains(node);
    }

    public void RemoveNeighbourSymmetric(PRMNode node)
    {
        neighbours.Remove(node);
        node.neighbours.Remove(this);
    }
}