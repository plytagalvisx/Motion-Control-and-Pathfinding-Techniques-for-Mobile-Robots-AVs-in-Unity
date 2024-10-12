// using System;
// using System.Collections.Generic;
// using System.Data;
// using UnityEngine;
// public class GeneralizedVoronoiDiagram
// {
//     private readonly TerrainInfo info;
//     private readonly PositionSampler sampler;
//     private void Log(string msg) { Debug.Log($"[PRM]: {msg}"); }

//     // Create mVoronoi

//     public GeneralizedVoronoiDiagram(TerrainInfo info, PositionSampler sampler)
//     {
//         this.info = info;
//         this.sampler = sampler;
//     }


//     // public List<Vector3> GenerateVoronoiDiagram(int num_samples)
//     // {
//     //     List<Vector3> positions = sampler.SamplePositions(num_samples);
//     //     List<VoronoiCell> voronoiCells = new List<VoronoiCell>();

//     //     foreach (Vector3 pos in positions)
//     //     {
//     //         voronoiCells.Add(new VoronoiCell(pos));
//     //     }

//     //     for (int i = 0; i < voronoiCells.Count; i++)
//     //     {
//     //         for (int j = i + 1; j < voronoiCells.Count; j++)
//     //         {
//     //             Vector3 sitePos1 = voronoiCells[i].sitePos;
//     //             Vector3 sitePos2 = voronoiCells[j].sitePos;

//     //             Vector3 midPoint = (sitePos1 + sitePos2) / 2;

//     //             VoronoiEdge edge = new VoronoiEdge(sitePos1, sitePos2, midPoint);

//     //             voronoiCells[i].edges.Add(edge);
//     //             voronoiCells[j].edges.Add(edge);
//     //         }
//     //     }

//     //     List<Vector3> voronoiVertices = new List<Vector3>();

//     //     foreach (VoronoiCell cell in voronoiCells)
//     //     {
//     //         foreach (VoronoiEdge edge in cell.edges)
//     //         {
//     //             voronoiVertices.Add(edge.sitePos);
//     //         }
//     //     }

//     //     return voronoiVertices;

//     // }

//     // public List<Vector3> GenerateVoronoiDiagram(int num_samples)
//     // {
//     //     List<VoronoiCell> voronoiCells = new List<VoronoiCell>();
//     //     List<VoronoiEdge> voronoiEdges = new List<VoronoiEdge>();
//     // }

//     // public class VoronoiEdge
//     // {
//     //     //These are the voronoi vertices
//     //     public Vector3 v1;
//     //     public Vector3 v2;

//     //     //All positions within a voronoi cell is closer to this position than any other position in the diagram
//     //     public Vector3 sitePos;

//     //     public VoronoiEdge(Vector3 v1, Vector3 v2, Vector3 sitePos)
//     //     {
//     //         this.v1 = v1;
//     //         this.v2 = v2;

//     //         this.sitePos = sitePos;
//     //     }
//     // }

//     // public class VoronoiCell
//     // {
//     //     //All positions within a voronoi cell is closer to this position than any other position in the diagram
//     //     public Vector3 sitePos;

//     //     public List<VoronoiEdge> edges = new List<VoronoiEdge>();

//     //     public VoronoiCell(Vector3 sitePos)
//     //     {
//     //         this.sitePos = sitePos;
//     //     }
//     // }

//     public List<VoronoiCell> DelaunayToVoronoi(List<Vector3> sites)
//     {
//         // https://www.habrador.com/tutorials/math/13-voronoi/
//         Debug.Log("Voronoi Diagram");
//         return null;
//     }

// }

// public class DelaunayTriangulation
// {
//     private readonly TerrainInfo info;
//     private readonly PositionSampler sampler;
//     public int seed = 0;
//     private void Log(string msg) { Debug.Log($"[PRM]: {msg}"); }

//     // Create mVoronoi

//     public DelaunayTriangulation(TerrainInfo info, PositionSampler sampler)
//     {
//         this.info = info;
//         this.sampler = sampler;
//     }

//     // Generate Delaunay Triangulation:
//     // public List<Triangle> GenerateDelaunayTriangulation(List<Vector3> points) // with halfedges
//     // {
//     //     List<Triangle> triangles = new List<Triangle>();

//     //     // Create a super triangle that bounds all the points
//     //     float minX = float.MaxValue;
//     //     float minY = float.MaxValue;
//     //     float maxX = float.MinValue;
//     //     float maxY = float.MinValue;

//     //     for (int i = 0; i < points.Count; i++)
//     //     {
//     //         if (points[i].x < minX)
//     //             minX = points[i].x;
//     //         if (points[i].y < minY)
//     //             minY = points[i].y;
//     //         if (points[i].x > maxX)
//     //             maxX = points[i].x;
//     //         if (points[i].y > maxY)
//     //             maxY = points[i].y;
//     //     }

//     //     float dx = maxX - minX;
//     //     float dy = maxY - minY;
//     //     float deltaMax = Math.Max(dx, dy);
//     //     float midx = (minX + maxX) / 2;
//     //     float midy = (minY + maxY) / 2;

//     //     Vertex p1 = new Vertex(new Vector3(midx - 20 * deltaMax, midy - deltaMax, 0));
//     //     Vertex p2 = new Vertex(new Vector3(midx, midy + 20 * deltaMax, 0));
//     //     Vertex p3 = new Vertex(new Vector3(midx + 20 * deltaMax, midy - deltaMax, 0));

//     //     triangles.Add(new Triangle(p1, p2, p3));

//     //     foreach (Vector3 point in points)
//     //     {
//     //         List<Triangle> badTriangles = new List<Triangle>();
//     //         foreach (Triangle triangle in triangles)
//     //         {
//     //             if (IsPointInsideCircumcircle(triangle, point))
//     //             {
//     //                 badTriangles.Add(triangle);
//     //             }
//     //         }

//     //         List<HalfEdge> polygon = new List<HalfEdge>();
//     //         foreach (Triangle triangle in badTriangles)
//     //         {
//     //             foreach (Vertex vertex in new Vertex[] { triangle.a, triangle.b, triangle.c })
//     //             {
//     //                 bool isShared = false;
//     //                 foreach (Triangle otherTriangle in badTriangles)
//     //                 {
//     //                     if (otherTriangle != triangle && otherTriangle.a == vertex || otherTriangle.b == vertex || otherTriangle.c == vertex)
//     //                     {
//     //                         isShared = true;
//     //                         break;
//     //                     }
//     //                 }
//     //                 if (!isShared)
//     //                 {
//     //                     polygon.Add(new HalfEdge(vertex));
//     //                 }
//     //             }
//     //         }

//     //         foreach (Triangle triangle in badTriangles)
//     //         {
//     //             triangles.Remove(triangle);
//     //         }

//     //         foreach (HalfEdge edge in polygon)
//     //         {
//     //             triangles.Add(new Triangle(edge.source, edge.nextEdge.source, new Vertex(point)));
//     //         }
//     //     }

//     //     List<Triangle> superTriangles = new List<Triangle>();
//     //     foreach (Triangle triangle in triangles)
//     //     {
//     //         if (triangle.a == p1 || triangle.a == p2 || triangle.a == p3 || triangle.b == p1 || triangle.b == p2 || triangle.b == p3 || triangle.c == p1 || triangle.c == p2 || triangle.c == p3)
//     //         {
//     //             superTriangles.Add(triangle);
//     //         }
//     //     }

//     //     foreach (Triangle triangle in superTriangles)
//     //     {
//     //         triangles.Remove(triangle);
//     //     }

//     //     return triangles;

//     // }

//     // public bool IsPointInsideCircumcircle(Triangle triangle, Vector3 point)
//     // {
//     //     float ax = triangle.a.position.x;
//     //     float ay = triangle.a.position.y;
//     //     float bx = triangle.b.position.x;
//     //     float by = triangle.b.position.y;
//     //     float cx = triangle.c.position.x;
//     //     float cy = triangle.c.position.y;
//     //     float px = point.x;
//     //     float py = point.y;

//     //     float ax_ = ax - px;
//     //     float ay_ = ay - py;
//     //     float bx_ = bx - px;
//     //     float by_ = by - py;
//     //     float cx_ = cx - px;
//     //     float cy_ = cy - py;

//     //     float d = (ax * ax + ay * ay) - (px * px + py * py);
//     //     float e = (bx * bx + by * by) - (px * px + py * py);
//     //     float f = (cx * cx + cy * cy) - (px * px + py * py);

//     //     float det = ax_ * (by_ * f - e * cy_) - bx_ * (ay_ * f - d * cy_) + cx_ * (ay_ * e - d * by_);

//     //     return det > 0;
//     // }

//     // public List<Triangle> GenerateDelaunayTriangulation2(List<Vector3> points) // without halfedges
//     // {
//     //     List<Triangle> triangles = new List<Triangle>();

//     //     // Create a super triangle that bounds all the points
//     //     float minX = float.MaxValue;
//     //     float minY = float.MaxValue;
//     //     float maxX = float.MinValue;
//     //     float maxY = float.MinValue;

//     //     for (int i = 0; i < points.Count; i++)
//     //     {
//     //         if (points[i].x < minX)
//     //             minX = points[i].x;
//     //         if (points[i].y < minY)
//     //             minY = points[i].y;
//     //         if (points[i].x > maxX)
//     //             maxX = points[i].x;
//     //         if (points[i].y > maxY)
//     //             maxY = points[i].y;
//     //     }

//     //     float dx = maxX - minX;
//     //     float dy = maxY - minY;
//     //     float deltaMax = Math.Max(dx, dy);
//     //     float midx = (minX + maxX) / 2;
//     //     float midy = (minY + maxY) / 2;

//     //     Vertex p1 = new Vertex(new Vector3(midx - 20 * deltaMax, midy - deltaMax, 0));
//     //     Vertex p2 = new Vertex(new Vector3(midx, midy + 20 * deltaMax, 0));
//     //     Vertex p3 = new Vertex(new Vector3(midx + 20 * deltaMax, midy - deltaMax, 0));

//     //     triangles.Add(new Triangle(p1, p2, p3));

//     //     for (int i = 0; i < points.Count; i++)
//     //     {
//     //         List<Triangle> badTriangles = new List<Triangle>();
//     //         for (int j = 0; j < triangles.Count; j++)
//     //         {
//     //             if (triangles[j].CircumCircleContains(points[i]))
//     //             {
//     //                 badTriangles.Add(triangles[j]);
//     //             }
//     //         }

//     //         List<Edge> polygon = new List<Edge>();
//     //         for (int j = 0; j < badTriangles.Count; j++)
//     //         {
//     //             for (int k = 0; k < 3; k++)
//     //             {
//     //                 Edge edge = new Edge(badTriangles[j].a, badTriangles[j].b);
//     //                 if (!polygon.Contains(edge))
//     //                 {
//     //                     polygon.Add(edge);
//     //                 }
//     //                 else
//     //                 {
//     //                     polygon.Remove(edge);
//     //                 }
//     //             }
//     //         }

//     //         for (int j = 0; j < badTriangles.Count; j++)
//     //         {
//     //             triangles.Remove(badTriangles[j]);
//     //         }

//     //         for (int j = 0; j < polygon.Count; j++)
//     //         {
//     //             triangles.Add(new Triangle(polygon[j].a, polygon[j].b, new Vertex(points[i])));
//     //         }
//     //     }

//     //     for (int i = 0; i < triangles.Count; i++)
//     //     {
//     //         if (triangles[i].ContainsVertex(p1) || triangles[i].ContainsVertex(p2) || triangles[i].ContainsVertex(p3))
//     //         {
//     //             triangles.RemoveAt(i);
//     //             i--;
//     //         }
//     //     }

//     //     return triangles;
//     // }


//     // // https://ics.uci.edu/~goodrich/teach/geom/notes/DT.pdf:
//     // public List<Triangle> DelaunayTriangulation(List<Vector3> P)
//     // {
//     //     // Input: A set P of n+1 points in the plane.
//     //     // Output: A Delaunay triangulation of P.

//     //     // Let p0 be the lexicographically highest point of P, that is, the rightmost among the points with largest z-coordinate:
//     //     Vector3 p0 = P[0];

//     //     for (int i = 1; i < P.Count; i++)
//     //     {
//     //         if (P[i].z > p0.z || (P[i].z == p0.z && P[i].x > p0.x))
//     //         {
//     //             p0 = P[i];
//     //         }
//     //     }

//     //     // Let p1 and p2 be two points in R2 sufficiently far away and such that P is contained in the triangle p0p1p2:
//     //     Vector3 p1 = new Vector3(p0.x + 100, p0.y, 0);
//     //     Vector3 p2 = new Vector3(p0.x, p0.y + 100, 0);

//     //     // Initialize T as the triangulation consisting of the single triangle p0p1p2:
//     //     Triangle t0 = new Triangle(new Vertex(p0), new Vertex(p1), new Vertex(p2));
//     //     List<Triangle> T = new List<Triangle>();
//     //     T.Add(t0);

//     //     // Compute random permutation p1, p2, ..., pn of P \ {p0}:
//     //     List<Vector3> P_ = new List<Vector3>(P);
//     //     P_.Remove(p0);
//     //     P_.Shuffle();

//     //     for (int r = 1; r < P_.Count; r++)
//     //     {
//     //         do
//     //         { /* Insert pr into T: */
//     //             // Find a triangle pipjpk in T containing pr:
//     //             Triangle t = null;
//     //             // If pr lies in the interior of the triangle pipjpk, then Add edges from pr to the three vertices of pipjpk, thereby splitting pipjpk into three triangles.
//     //             // LegalizeEdge(pr, pipj, T)
//     //             // LegalizeEdge(pr, pjpk, T)
//     //             // LegalizeEdge(pr, pkpi, T)
//     //             if (t.CircumCircleContains(P_[r]))
//     //             {
//     //                 Vertex pi = t.a;
//     //                 Vertex pj = t.b;
//     //                 Vertex pk = t.c;

//     //                 Triangle t1 = new Triangle(pi, pj, new Vertex(P_[r]));
//     //                 Triangle t2 = new Triangle(pj, pk, new Vertex(P_[r]));
//     //                 Triangle t3 = new Triangle(pk, pi, new Vertex(P_[r]));

//     //                 T.Remove(t);
//     //                 T.Add(t1);
//     //                 T.Add(t2);
//     //                 T.Add(t3);

//     //                 // Legalize the new edges of t1, t2, and t3.
//     //                 LegalizeEdge(T, t1, pi, pj, P_[r]);
//     //                 LegalizeEdge(T, t2, pj, pk, P_[r]);
//     //                 LegalizeEdge(T, t3, pk, pi, P_[r]);

//     //                 break;
//     //             }
//     //             else /* pr lies on an edge of pipjpk, say the edge pipj */
//     //             {
//     //                 // Add edges from pr to pk and to the third vertex pl of the other triangle that is incident to pipj, thereby splitting the two triangles incident to pipj into four triangles:
//     //                 // LegalizeEdge(pr, pipl, T)
//     //                 // LegalizeEdge(pr, plpj, T)
//     //                 // LegalizeEdge(pr, pjpk, T)
//     //                 // LegalizeEdge(pr, pkpi, T)
//     //             }
//     //         } while (true);
//     //     }

//     //     // Discard p1 and p2 with all their incident edges from T:

//     //     return T;

//     // }

//     // public void LegalizeEdge(Vertex pr, Edge pipj, List<Triangle>)
//     // {
//     //     /* The point being inserted is pr, and pipj is the edge of T that may need to be flipped. */
//     //     if (IsEdgeIllegal(pipj, pr))
//     //     {
//     //         // Let pipjpk be the triangle adjacent to prpipj along pipj.
//     //         /* Flip pipj */ // Replace pipj with prpk.
//     //         // LegalizeEdge(pr, pkpi, T)
//     //         // LegalizeEdge(pr, pkpj, T)
//     //     }
//     // }


//     // public void LegalizeEdge(List<Triangle> T, Triangle t, Vertex pi, Vertex pj, Vector3 pr)
//     // {
//     //     // If the edge pipj is illegal, then flip it and recursively legalize the two edges that are now illegal.
//     //     if (IsEdgeIllegal(t, pi, pj, pr))
//     //     {
//     //         Triangle t1 = null;
//     //         Triangle t2 = null;

//     //         foreach (Triangle triangle in T)
//     //         {
//     //             if (triangle.ContainsVertex(pi) && triangle.ContainsVertex(pr))
//     //             {
//     //                 t1 = triangle;
//     //             }
//     //             if (triangle.ContainsVertex(pj) && triangle.ContainsVertex(pr))
//     //             {
//     //                 t2 = triangle;
//     //             }
//     //         }

//     //         Vertex pk = t1.a == pi ? t1.b : t1.a;
//     //         Vertex pl = t2.a == pj ? t2.b : t2.a;

//     //         Triangle t3 = new Triangle(pi, pl, new Vertex(pr));
//     //         Triangle t4 = new Triangle(pj, pk, new Vertex(pr));

//     //         T.Remove(t1);
//     //         T.Remove(t2);
//     //         T.Add(t3);
//     //         T.Add(t4);

//     //         LegalizeEdge(T, t3, pi, pl, pr);
//     //         LegalizeEdge(T, t4, pj, pk, pr);
//     //     }
//     // }


//     public List<Triangle> GenerateDelaunayTriangulationCurrentlyUsed() // List<Vector3> P)
//     {

//         int numberOfPoints = 150;
//         List<Vector3> P = new List<Vector3>();

//         //Generate random numbers with a seed
//         // UnityEngine.Random.InitState(seed);

//         // float max = info.x_high;
//         // float min = info.x_low;

//         // for (int i = 0; i < numberOfPoints; i++)
//         // {
//         //     float randomX = UnityEngine.Random.Range(min, max);
//         //     float randomZ = UnityEngine.Random.Range(min, max);

//         //     P.Add(new Vector3(randomX, 0f, randomZ));
//         // }

//         P = sampler.SamplePositions(numberOfPoints);

//         //Points outside of the screen for voronoi which has some cells that are infinite
//         // float bigSize = info.x_high * 5f;
//         // P.Add(new Vector3(0f, 0f, bigSize));
//         // P.Add(new Vector3(0f, 0f, -bigSize));
//         // P.Add(new Vector3(bigSize, 0f, 0f));
//         // P.Add(new Vector3(-bigSize, 0f, 0f));

//         // Draw points of P:
//         foreach (Vector3 point in P)
//         {
//             Debug.DrawLine(point, point + Vector3.up * 10, Color.red, 1000f);
//         }


//         // Initialize variables
//         // List<Triangle> T = new List<Triangle>();

//         // // Find the lexicographically highest point
//         // Vector3 p0 = P[0];
//         // foreach (Vector3 p in P)
//         // {
//         //     if (p.z > p0.z || (p.z == p0.z && p.x > p0.x))
//         //         p0 = p;
//         // }

//         // // Find p1 and p2 sufficiently far away within the [x_low, x_high] and [z_low, z_high] ranges
//         // float maxX = info.x_high;
//         // float maxZ = info.z_high;

//         // // foreach (Vector3 p in P)
//         // // {
//         // //     maxX = Math.Max(maxX, p.x);
//         // //     maxZ = Math.Max(maxZ, p.z);
//         // // }
//         // Vector3 p1 = new Vector3(maxX, 0, 10);
//         // Vector3 p2 = new Vector3(10, 0, maxZ);


//         // Initialize T with the initial triangle
//         // T.Add(new Triangle(new Vertex(p0), new Vertex(p1), new Vertex(p2)));



//         // Find the minimum and maximum x and z coordinates of the sampled points
//         float minX = float.MaxValue, minZ = float.MaxValue;
//         float maxX = float.MinValue, maxZ = float.MinValue;
//         foreach (Vector3 point in P)
//         {
//             minX = Mathf.Min(minX, point.x);
//             minZ = Mathf.Min(minZ, point.z);
//             maxX = Mathf.Max(maxX, point.x);
//             maxZ = Mathf.Max(maxZ, point.z);
//         }

//         // Calculate p1 and p2 based on the minimum and maximum coordinates
//         Vector3 p0 = new Vector3(maxX, 0, maxZ);
//         Vector3 p1 = new Vector3(minX - 100f, 0, minZ - 100f); // Adjust the offset as needed
//         Vector3 p2 = new Vector3(maxX + 100f, 0, minZ - 100f); // Adjust the offset as needed

//         // Ensure that p0, p1, and p2 form a triangle containing all points in P
//         if (!IsTriangleContainingAllPoints(p0, p1, p2, P))
//         {
//             // Adjust p1 and p2 positions until they contain all points
//             p1 += new Vector3(10f, 0, 10f); // Adjust the step size as needed
//             p2 += new Vector3(-10f, 0, 10f); // Adjust the step size as needed
//         }

//         // Create the triangle list containing p0, p1, and p2
//         List<Triangle> T = new List<Triangle>();
//         T.Add(new Triangle(new Vertex(p0), new Vertex(p1), new Vertex(p2)));


//         // Draw the triangles:
//         foreach (Triangle triangle in T)
//         {
//             Debug.DrawLine(triangle.a.position, triangle.b.position, Color.green, 1000f);
//             Debug.DrawLine(triangle.b.position, triangle.c.position, Color.green, 1000f);
//             Debug.DrawLine(triangle.c.position, triangle.a.position, Color.green, 1000f);
//         }

//         // Compute random permutation of P \ {p0}
//         List<Vector3> permutedP = new List<Vector3>(P);
//         permutedP.Remove(p0);
//         System.Random rng = new System.Random();
//         permutedP.Sort((a, b) => rng.Next(-1, 2));

//         // Perform Delaunay triangulation
//         foreach (Vector3 pr in permutedP)
//         {
//             List<Triangle> badTriangles = new List<Triangle>();
//             foreach (Triangle triangle in T)
//             {
//                 if (IsPointInsideCircumcircle(triangle, new Vertex(pr)))
//                 {
//                     badTriangles.Add(triangle);
//                 }
//             }

//             List<Edge> polygon = new List<Edge>();
//             foreach (Triangle triangle in badTriangles)
//             {
//                 foreach (Vertex vertex in new Vertex[] { triangle.a, triangle.b, triangle.c })
//                 {
//                     foreach (Triangle neighbor in T)
//                     {
//                         if (neighbor != triangle && neighbor.ContainsVertex(vertex))
//                         {
//                             bool sharedEdge = false;
//                             foreach (Edge edge in polygon)
//                             {
//                                 if (edge.Equals(new Edge(vertex, neighbor.GetOppositeVertex(triangle, vertex))))
//                                 {
//                                     sharedEdge = true;
//                                     break;
//                                 }
//                             }
//                             if (!sharedEdge)
//                             {
//                                 polygon.Add(new Edge(vertex, neighbor.GetOppositeVertex(triangle, vertex)));
//                             }
//                         }
//                     }
//                 }
//             }

//             foreach (Triangle triangle in badTriangles)
//             {
//                 T.Remove(triangle);
//             }

//             foreach (Edge edge in polygon)
//             {
//                 T.Add(new Triangle(edge.a, edge.b, new Vertex(pr)));
//             }
//         }

//         // Remove triangles containing p1 or p2
//         List<Triangle> toRemove = new List<Triangle>();
//         foreach (Triangle triangle in T)
//         {
//             if (triangle.ContainsVertex(new Vertex(p1)) || triangle.ContainsVertex(new Vertex(p2)))
//             {
//                 toRemove.Add(triangle);
//             }
//         }
//         foreach (Triangle triangle in toRemove)
//         {
//             T.Remove(triangle);
//         }

//         // Draw the triangles
//         // foreach (Triangle triangle in T)
//         // {
//         //     Debug.DrawLine(triangle.a.position, triangle.b.position, Color.green, 1000f);
//         //     Debug.DrawLine(triangle.b.position, triangle.c.position, Color.green, 1000f);
//         //     Debug.DrawLine(triangle.c.position, triangle.a.position, Color.green, 1000f);
//         // }

//         return T;
//     }

//     // private bool IsPointInsideCircumcircle2(Triangle triangle, Vertex point)
//     // {
//     //     double ax = triangle.a.x - point.x;
//     //     double ay = triangle.a.y - point.y;
//     //     double bx = triangle.b.x - point.x;
//     //     double by = triangle.b.y - point.y;
//     //     double cx = triangle.c.x - point.x;
//     //     double cy = triangle.c.y - point.y;

//     //     double ab = ax * by - ay * bx;
//     //     double bc = bx * cy - by * cx;
//     //     double ca = cx * ay - cy * ax;

//     //     double dot = ax * (triangle.a.x + point.x) + ay * (triangle.a.y + point.y);
//     //     double det = ab + bc + ca;

//     //     return det * dot > 0;
//     // }

//     private bool IsPointInsideCircumcircle(Triangle triangle, Vertex point)
//     {
//         Vector2 center = (triangle.a.position + triangle.b.position + triangle.c.position) / 3;
//         float radiusSquared = Mathf.Pow(Vector2.Distance(center, triangle.a.position), 2);

//         float dx = center.x - point.position.x;
//         float dy = center.y - point.position.y;
//         float distSquared = dx * dx + dy * dy;

//         return distSquared <= radiusSquared;
//     }

//     private bool IsTriangleContainingAllPoints(Vector3 p0, Vector3 p1, Vector3 p2, List<Vector3> points)
//     {
//         foreach (Vector3 point in points)
//         {
//             if (!IsPointInsideTriangle(p0, p1, p2, point))
//                 return false;
//         }
//         return true;
//     }

//     private bool IsPointInsideTriangle(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 point)
//     {
//         Vector3 e0 = p1 - p0;
//         Vector3 e1 = p2 - p0;
//         Vector3 e2 = point - p0;

//         float dot00 = Vector3.Dot(e0, e0);
//         float dot01 = Vector3.Dot(e0, e1);
//         float dot02 = Vector3.Dot(e0, e2);
//         float dot11 = Vector3.Dot(e1, e1);
//         float dot12 = Vector3.Dot(e1, e2);

//         float invDenom = 1f / (dot00 * dot11 - dot01 * dot01);
//         float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
//         float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

//         return (u >= 0) && (v >= 0) && (u + v < 1);
//     }


// }

// public class Edge
// {
//     public Vertex a;
//     public Vertex b;
//     public Edge(Vertex a, Vertex b)
//     {
//         this.a = a;
//         this.b = b;
//     }

//     public override bool Equals(object obj)
//     {
//         Edge other = (Edge)obj;
//         return (a == other.a && b == other.b) || (a == other.b && b == other.a);
//     }

//     public override int GetHashCode()
//     {
//         return a.GetHashCode() + b.GetHashCode();
//     }

// }

// // public class Triangle
// // {
// //     public Vertex a;
// //     public Vertex b;
// //     public Vertex c;

// //     public HalfEdge halfedge;
// //     public Triangle(Vertex a, Vertex b, Vertex c)
// //     {
// //         this.a = a;
// //         this.b = b;
// //         this.c = c;
// //     }

// //     // public bool CircumCircleContains(Vector3 point)
// //     // {
// //     //     float ax = a.position.x;
// //     //     float ay = a.position.y;
// //     //     float bx = b.position.x;
// //     //     float by = b.position.y;
// //     //     float cx = c.position.x;
// //     //     float cy = c.position.y;
// //     //     float px = point.x;
// //     //     float py = point.y;

// //     //     float ax_ = ax - px;
// //     //     float ay_ = ay - py;
// //     //     float bx_ = bx - px;
// //     //     float by_ = by - py;
// //     //     float cx_ = cx - px;
// //     //     float cy_ = cy - py;

// //     //     float d = (ax * ax + ay * ay) - (px * px + py * py);
// //     //     float e = (bx * bx + by * by) - (px * px + py * py);
// //     //     float f = (cx * cx + cy * cy) - (px * px + py * py);

// //     //     float det = ax_ * (by_ * f - e * cy_) - bx_ * (ay_ * f - d * cy_) + cx_ * (ay_ * e - d * by_);

// //     //     return det > 0;
// //     // }

// //     public bool ContainsVertex(Vertex vertex)
// //     {
// //         return a == vertex || b == vertex || c == vertex;
// //     }

// //     public Vertex GetOppositeVertex(Triangle triangle, Vertex vertex)
// //     {
// //         if (a != vertex && triangle.ContainsVertex(a))
// //             return a;
// //         if (b != vertex && triangle.ContainsVertex(b))
// //             return b;
// //         if (c != vertex && triangle.ContainsVertex(c))
// //             return c;
// //         return null;
// //     }

// // }

// public class Face
// {
//     public Site site;
//     public HalfEdge outerComponent;
// }

// public class Site
// {
//     public int index;
//     public Vector3 point;
//     public Face face;
//     public Site(Vector3 point)
//     {
//         this.point = point;
//     }
// }

// public class Vertex
// {
//     public Vector3 position;
//     public Vertex(Vector3 position)
//     {
//         this.position = position;
//     }
// }

// public class HalfEdge
// {
//     public Vertex source;
//     // public HalfEdge oppositeEdge; // twin edge
//     public HalfEdge twinEdge; // twin edge (2)
//     public Face incidentFace;
//     public HalfEdge nextEdge; // nextEdge
//     public HalfEdge prevEdge; // prevEdge

//     public HalfEdge(Vertex source)
//     {
//         this.source = source;
//     }
// }

// public class VoronoiEdge
// {
//     //These are the voronoi vertices
//     public Vector3 v1;
//     public Vector3 v2;

//     //All positions within a voronoi cell is closer to this position than any other position in the diagram
//     public Vector3 sitePos;

//     public VoronoiEdge(Vector3 v1, Vector3 v2, Vector3 sitePos)
//     {
//         this.v1 = v1;
//         this.v2 = v2;

//         this.sitePos = sitePos;
//     }
// }

// public class VoronoiCell
// {
//     //All positions within a voronoi cell is closer to this position than any other position in the diagram
//     public Vector3 sitePos;

//     public List<VoronoiEdge> edges = new List<VoronoiEdge>();

//     public VoronoiCell(Vector3 sitePos)
//     {
//         this.sitePos = sitePos;
//     }
// }

