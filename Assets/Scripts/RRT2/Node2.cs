using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Node2
{
    public Vector2 position;
    public Vector2 center;
    public Vector2 size;
    public List<Node2> neighbours = new List<Node2>();

    public Node2(Vector2 pos)
    {
        position = pos;
    }
    public Node2(Vector2 Nodeposition, Vector2 Nodecenter, Vector2 Nodesize)
    {
        position = Nodeposition;
        center = Nodecenter;
        size = Nodesize;
    }

    public List<Vector2> GetVertices() // represents the node as a rectangle with 4 vertices as corners of the cell in the grid
    {
        List<Vector2> vertices = new List<Vector2>();
        Vector2 vertex1 = new Vector2(center.x - size.x / 2, center.y - size.y / 2);
        Vector2 vertex2 = new Vector2(center.x - size.x / 2, center.y + size.y / 2);
        Vector2 vertex3 = new Vector2(center.x + size.x / 2, center.y - size.y / 2);
        Vector2 vertex4 = new Vector2(center.x + size.x / 2, center.y + size.y / 2);
        vertices.Add(vertex1);
        vertices.Add(vertex2);
        vertices.Add(vertex3);
        vertices.Add(vertex4);
        return vertices;
    }

    public bool in_area(Vector2 pos) // Check if a point is inside the node
    {
        List<Vector2> vertices = GetVertices();
        if (pos.x < vertices[0].x || pos.x > vertices[3].x)
        {
            return false;
        }
        if (pos.y < vertices[0].y || pos.y > vertices[3].y)
        {
            return false;
        }
        return true;
    }

    public void AddNeighbour(Node2 node)
    {
        neighbours.Add(node);
    }

    public void AddNeighbourSymmetric(Node2 node)
    {
        neighbours.Add(node);
        node.neighbours.Add(this);
    }

    public void RemoveNeighbour(Node2 node)
    {
        neighbours.Remove(node);
    }

    public void RemoveNeighbourSymmetric(Node2 node)
    {
        neighbours.Remove(node);
        node.neighbours.Remove(this);
    }

    public List<Node2> GetNeighbours()
    {
        return neighbours;
    }

    public bool IsNeighbour(Node2 node)
    {
        return neighbours.Contains(node);
    }

    public void LogPosition()
    {
        if (center == null)
        {
            Debug.Log("Node2: " + position.x + ", " + position.y);
        }
        else
        {
            Debug.Log(string.Format("Node2: ({0}, {1}), center: ({2}, {3}), size: ({4}, {5})",
                                    position.x, position.y,
                                    center.x, center.y,
                                    size.x, size.y));
        }
    }
}