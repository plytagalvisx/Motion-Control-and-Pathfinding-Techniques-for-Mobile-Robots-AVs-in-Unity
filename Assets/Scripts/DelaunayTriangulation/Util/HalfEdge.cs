using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class HalfEdge
{
    // The vertex the edge points to 
    public Vertex v;
    // The face this edge is a part of (aka the face)
    public Triangle t;

    // The next edge
    public HalfEdge nextEdge;
    // The previous
    public HalfEdge prevEdge;
    // The edge going in the opposite direction
    public HalfEdge oppositeEdge;

    // This structure assumes we have a vertex class with a reference to a half edge going from that vertex
    // and a face (triangle) class with a reference to a half edge which is a part of this face 
    public HalfEdge(Vertex v)
    {
        this.v = v;
    }
}
