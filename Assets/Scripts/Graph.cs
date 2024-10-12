using UnityEngine.Assertions;
using Unity.Collections;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using UnityEditor;
using System.Linq;

public class Graph<Node> // Node = Location
{
    // NameValueCollection would be a reasonable alternative here, if
    // you're always using string location/node types
    public Dictionary<Node, List<Node>> edges = new Dictionary<Node, List<Node>>();

    public List<Node> GetNeighbors(Node node) // For Graph map
    {
        return edges[node];
    }

    // Get the list of all nodes in the graph
    public List<Node> GetNodes()
    {
        return new List<Node>(edges.Keys);
    }

    public void AddNode(Node node)
    {
        // edges[node] = new List<Node>();
        if (!edges.ContainsKey(node))
        {
            edges.Add(node, new List<Node>());
        }
    }

    public void AddNeighbourSymmetric(Node source, Node target)
    {
        if (!edges.ContainsKey(source))
        {
            AddNode(source);
        }
        if (!edges.ContainsKey(target))
        {
            AddNode(target);
        }
        edges[source].Add(target);
        edges[target].Add(source);
    }
};

public struct Node
{
    // Implementation notes: I am using the default Equals but it can
    // be slow. You'll probably want to override both Equals and
    // GetHashCode in a real project.
    public readonly int x, y; // x and z coordinates of the node
    public Node(int x, int y)
    {
        this.x = x;
        this.y = y;
    }
}