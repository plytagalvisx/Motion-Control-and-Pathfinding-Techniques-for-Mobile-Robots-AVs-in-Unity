// using System;
// using System.Collections.Generic;
// using System.Data;
// using UnityEngine;
// public class DelaunayTriangulation2
// {
//     private readonly TerrainInfo info;
//     private readonly PositionSampler sampler;
//     private void Log(string msg) { Debug.Log($"[PRM]: {msg}"); }

//     // Create mVoronoi

//     public DelaunayTriangulation2(TerrainInfo info, PositionSampler sampler)
//     {
//         this.info = info;
//         this.sampler = sampler;
//     }

//     public class Triangle
//     {
//         public Vertex a;
//         public Vertex b;
//         public Vertex c;

//         public HalfEdge ab;
//         public Triangle(Vertex a, Vertex b, Vertex c)
//         {
//             this.a = a;
//             this.b = b;
//             this.c = c;
//         }
//     }

//     // Generate Delaunay Triangulation:
//     public List<Triangle> GenerateDelaunayTriangulation(List<Vector3> points)
//     {
//         List<Triangle> triangles = new List<Triangle>();

//         // Create a super triangle that bounds all the points
//         float minX = float.MaxValue;
//         float minY = float.MaxValue;
//         float maxX = float.MinValue;
//         float maxY = float.MinValue;

//         for (int i = 0; i < points.Count; i++)
//         {
//             if (points[i].x < minX)
//                 minX = points[i].x;
//             if (points[i].y < minY)
//                 minY = points[i].y;
//             if (points[i].x > maxX)
//                 maxX = points[i].x;
//             if (points[i].y > maxY)
//                 maxY = points[i].y;
//         }

//         float dx = maxX - minX;
//         float dy = maxY - minY;
//         float deltaMax = Math.Max(dx, dy);
//         float midx = (minX + maxX) / 2;
//         float midy = (minY + maxY) / 2;

//         Vertex p1 = new Vertex(new Vector3(midx - 20 * deltaMax, midy - deltaMax, 0));
//         Vertex p2 = new Vertex(new Vector3(midx, midy + 20 * deltaMax, 0));
//         Vertex p3 = new Vertex(new Vector3(midx + 20 * deltaMax, midy - deltaMax, 0));

//         triangles.Add(new Triangle(p1, p2, p3));

//         for (int i = 0; i < points.Count; i++)
//         {
//             List<Triangle> badTriangles = new List<Triangle>();
//             for (int j = 0; j < triangles.Count; j++)
//             {
//                 if (triangles[j].CircumCircleContains(points[i]))
//                 {
//                     badTriangles.Add(triangles[j]);
//                 }
//             }

//             List<Edge> polygon = new List<Edge>();
//             for (int j = 0; j < badTriangles.Count; j++)
//             {
//                 for (int k = 0; k < 3; k++)
//                 {
//                     Edge edge = new Edge(badTriangles[j].vertices[k], badTriangles[j].vertices[(k + 1) % 3]);
//                     if (!polygon.Contains(edge))
//                     {
//                         polygon.Add(edge);
//                     }
//                     else
//                     {
//                         polygon.Remove(edge);
//                     }
//                 }
//             }

//             for (int j = 0; j < badTriangles.Count; j++)
//             {
//                 triangles.Remove(badTriangles[j]);
//             }

//             for (int j = 0; j < polygon.Count; j++)
//             {
//                 triangles.Add(new Triangle(polygon[j].a, polygon[j].b, new Vertex(points[i])));
//             }
//         }

//         for (int i = 0; i < triangles.Count; i++)
//         {
//             if (triangles[i].ContainsVertex(p1) || triangles[i].ContainsVertex(p2) || triangles[i].ContainsVertex(p3))
//             {
//                 triangles.RemoveAt(i);
//                 i--;
//             }
//         }

//         return triangles;
//     }
// }