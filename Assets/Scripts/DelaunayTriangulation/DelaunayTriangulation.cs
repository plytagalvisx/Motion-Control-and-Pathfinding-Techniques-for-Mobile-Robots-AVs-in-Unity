using System;
using System.Collections.Generic;
using System.Data;
using UnityEngine;
using System.Linq;

public class DelaunayTriangulation
{
    private readonly TerrainInfo info;
    private readonly PositionSampler sampler;
    public int seed = 0;
    private void Log(string msg) { Debug.Log($"[Delaunay Triangulation]: {msg}"); }

    public DelaunayTriangulation(TerrainInfo info, PositionSampler sampler)
    {
        this.info = info;
        this.sampler = sampler;
    }

    // The main delaunay triangulation algorithm:
    // Alternative 1. Triangulate with some algorithm - then flip edges until we have a delaunay triangulation
    public List<Triangle> DelaunayTriangulateByFlippingEdges(List<Vector3> sites) // List<Vector3> sites)
    {
        // UnityEngine.Random.InitState(seed);
        // int numberOfPoints = 150;
        // List<Vector3> sites = sampler.SamplePositions(numberOfPoints);

        // foreach (Vector3 point in sites)
        // {
        //     Debug.DrawLine(point, point + Vector3.up * 10, Color.red, 1000f);
        // }

        // Step 1. Triangulate the points with some algorithm
        // Convert Vector3 to vertex
        List<Vertex> vertices = new List<Vertex>();

        for (int i = 0; i < sites.Count; i++)
        {
            vertices.Add(new Vertex(sites[i]));
        }

        // IncrementalTriangulation is a specialized convex hull algorithm?
        // Let's say you have points in the plane you want to triangulate. According to Wikipedia, 
        // there are three algorithms you can use: Triangle Splitting Algorithm, Incremental Algorithm, 
        // and Delaunay Triangulation Algorithm.
        // As usual there are more than one way to achieve a Delaunay Triangulation, and we are here going 
        // to use the simplest one. We are first going to use one of the algortihms from Triangulation of random points (e.g. Incremental Algorithm), 
        // so we will have some triangles that we need to change by flipping edges. To make it easier to flip edges, 
        // we have to change data structure from triangles to half-edge.

        // Triangulate the convex hull of the sites (aka points)
        List<Triangle> triangles = IncrementalTriangulation(vertices);
        // List<Triangle> triangles = TriangleSplitting(vertices);

        // Display the triangles
        // foreach (Triangle t in triangles)
        // {
        //     Debug.DrawLine(t.v1.position, t.v2.position, Color.blue, 1000f);
        //     Debug.DrawLine(t.v2.position, t.v3.position, Color.blue, 1000f);
        //     Debug.DrawLine(t.v3.position, t.v1.position, Color.blue, 1000f);
        // }

        // Step 2. Change the structure from triangle to half-edge to make it faster to flip edges
        List<HalfEdge> halfEdges = TransformFromTriangleToHalfEdge(triangles);

        // Step 3. Flip edges until we have a delaunay triangulation
        int safety = 0;
        int flippedEdges = 0;
        while (true)
        {
            safety += 1;
            if (safety > 100000)
            {
                Debug.Log("Stuck in endless loop");
                break;
            }

            bool hasFlippedEdge = false;
            // Search through all edges to see if we can flip an edge
            for (int i = 0; i < halfEdges.Count; i++)
            {
                HalfEdge thisEdge = halfEdges[i];

                // Is this edge sharing an edge, otherwise its a border, and then we cant flip the edge
                if (thisEdge.oppositeEdge == null)
                {
                    continue;
                }

                // The vertices belonging to the two triangles, c-a are the edge vertices, b belongs to this triangle
                Vertex a = thisEdge.v;
                Vertex b = thisEdge.nextEdge.v;
                Vertex c = thisEdge.prevEdge.v;
                Vertex d = thisEdge.oppositeEdge.nextEdge.v;

                Vector2 aPos = a.GetPos2D_XZ();
                Vector2 bPos = b.GetPos2D_XZ();
                Vector2 cPos = c.GetPos2D_XZ();
                Vector2 dPos = d.GetPos2D_XZ();

                // Use the circle test to test if we need to flip this edge
                if (IsPointInsideOutsideOrOnCircle(aPos, bPos, cPos, dPos) < 0f)
                {
                    // Are these the two triangles that share this edge forming a convex quadrilateral?
                    // Otherwise the edge cant be flipped
                    if (IsQuadrilateralConvex(aPos, bPos, cPos, dPos))
                    {
                        // If the new triangle after a flip is not better, then dont flip
                        // This will also stop the algoritm from ending up in an endless loop
                        if (IsPointInsideOutsideOrOnCircle(bPos, cPos, dPos, aPos) < 0f)
                        {
                            continue;
                        }

                        // Here we Flip the edge
                        flippedEdges += 1;
                        hasFlippedEdge = true;
                        FlipEdge(thisEdge);
                    }
                }
            }

            // We have searched through all edges and havent found an edge to flip, so we have a Delaunay triangulation!
            if (!hasFlippedEdge)
            {
                // Debug.Log("Found a delaunay triangulation");
                break;
            }
        }

        // Debug.Log("Flipped edges: " + flippedEdges);

        // Dont have to convert from half edge to triangle because the algorithm will modify the objects, which belongs to the 
        // original triangles, so the triangles have the data we need (meaning we can use the triangles directly)

        return triangles;
    }



    // Triangulate random points by first generating the convex hull of the points, then triangulate the convex hull
    // and then add the other points and split the triangle the point is in
    public List<Triangle> TriangleSplitting(List<Vertex> points)
    {
        // Generate the convex hull - will also remove the points from points list which are not on the hull
        List<Vertex> pointsOnConvexHull = JarvisMarchAlgorithm(points);

        // Triangulate the convex hull
        List<Triangle> triangles = TriangulateConvexPolygon(pointsOnConvexHull);

        // Add the remaining points and split the triangles
        for (int i = 0; i < points.Count; i++)
        {
            Vertex currentPoint = points[i];

            // 2d space
            Vector2 p = new Vector2(currentPoint.position.x, currentPoint.position.z);

            // Which triangle is this point in?
            for (int j = 0; j < triangles.Count; j++)
            {
                Triangle t = triangles[j];

                Vector2 p1 = new Vector2(t.v1.position.x, t.v1.position.z);
                Vector2 p2 = new Vector2(t.v2.position.x, t.v2.position.z);
                Vector2 p3 = new Vector2(t.v3.position.x, t.v3.position.z);

                if (IsPointInTriangle(p1, p2, p3, p))
                {
                    // Create 3 new triangles
                    Triangle t1 = new Triangle(t.v1, t.v2, currentPoint);
                    Triangle t2 = new Triangle(t.v2, t.v3, currentPoint);
                    Triangle t3 = new Triangle(t.v3, t.v1, currentPoint);

                    // Remove the old triangle
                    triangles.Remove(t);

                    // Add the new triangles
                    triangles.Add(t1);
                    triangles.Add(t2);
                    triangles.Add(t3);
                    break;
                }
            }
        }
        return triangles;
    }
    public List<Vertex> JarvisMarchAlgorithm(List<Vertex> points)
    {
        //If we have just 3 points, then they are the convex hull, so return those
        if (points.Count == 3)
        {
            //These might not be ccw, and they may also be colinear
            return points;
        }

        //If fewer points, then we cant create a convex hull
        if (points.Count < 3)
        {
            return null;
        }

        //The list with points on the convex hull
        List<Vertex> convexHull = new List<Vertex>();

        //Step 1. Find the vertex with the smallest x coordinate
        //If several have the same x coordinate, find the one with the smallest z
        Vertex startVertex = points[0];
        Vector3 startPos = startVertex.position;
        for (int i = 1; i < points.Count; i++)
        {
            Vector3 testPos = points[i].position;

            //Because of precision issues, we use Mathf.Approximately to test if the x positions are the same
            if (testPos.x < startPos.x || (Mathf.Approximately(testPos.x, startPos.x) && testPos.z < startPos.z))
            {
                startVertex = points[i];
                startPos = startVertex.position;
            }
        }

        //This vertex is always on the convex hull
        convexHull.Add(startVertex);

        points.Remove(startVertex);

        //Step 2. Loop to generate the convex hull
        Vertex currentPoint = convexHull[0];

        //Store colinear points here - better to create this list once than each loop
        List<Vertex> colinearPoints = new List<Vertex>();

        int counter = 0;

        while (true)
        {
            //After 2 iterations we have to add the start position again so we can terminate the algorithm
            //Cant use convexhull.count because of colinear points, so we need a counter
            if (counter == 2)
            {
                points.Add(convexHull[0]);
            }

            //Pick next point randomly
            Vertex nextPoint = points[UnityEngine.Random.Range(0, points.Count)];

            //To 2d space so we can see if a point is to the left is the vector ab
            Vector2 a = currentPoint.GetPos2D_XZ();
            Vector2 b = nextPoint.GetPos2D_XZ();

            //Test if there's a point to the right of ab, if so then it's the new b
            for (int i = 0; i < points.Count; i++)
            {
                //Dont test the point we picked randomly
                if (points[i].Equals(nextPoint)) continue;

                Vector2 c = points[i].GetPos2D_XZ();

                //Where is c in relation to a-b
                // < 0 -> to the right
                // = 0 -> on the line
                // > 0 -> to the left
                float relation = IsAPointLeftOfVectorOrOnTheLine(a, b, c);

                //Colinear points
                //Cant use exactly 0 because of floating point precision issues
                //This accuracy is smallest possible, if smaller points will be missed if we are testing with a plane
                float accuracy = 0.00001f;
                if (relation < accuracy && relation > -accuracy)
                {
                    colinearPoints.Add(points[i]);
                }
                //To the right = better point, so pick it as next point on the convex hull
                else if (relation < 0f)
                {
                    nextPoint = points[i];
                    b = nextPoint.GetPos2D_XZ();

                    //Clear colinear points
                    colinearPoints.Clear();
                }
                //To the left = worse point so do nothing
            }

            //If we have colinear points
            if (colinearPoints.Count > 0)
            {
                colinearPoints.Add(nextPoint);

                //Sort this list, so we can add the colinear points in correct order
                colinearPoints = colinearPoints.OrderBy(n => Vector3.SqrMagnitude(n.position - currentPoint.position)).ToList();
                convexHull.AddRange(colinearPoints);
                currentPoint = colinearPoints[colinearPoints.Count - 1];

                //Remove the points that are now on the convex hull
                for (int i = 0; i < colinearPoints.Count; i++)
                {
                    points.Remove(colinearPoints[i]);
                }
                colinearPoints.Clear();
            }
            else
            {
                convexHull.Add(nextPoint);
                points.Remove(nextPoint);
                currentPoint = nextPoint;
            }

            //Have we found the first point on the hull? If so we have completed the hull
            if (currentPoint.Equals(convexHull[0]))
            {
                //Then remove it because it is the same as the first point, and we want a convex hull with no duplicates
                convexHull.RemoveAt(convexHull.Count - 1);

                break;
            }

            counter += 1;
        }

        return convexHull;
    }

    //Where is p in relation to a-b
    // < 0 -> to the right
    // = 0 -> on the line
    // > 0 -> to the left
    public float IsAPointLeftOfVectorOrOnTheLine(Vector2 a, Vector2 b, Vector2 p)
    {
        float determinant = (a.x - p.x) * (b.y - p.y) - (a.y - p.y) * (b.x - p.x);

        return determinant;
    }

    public List<Triangle> TriangulateConvexPolygon(List<Vector3> points)
    {
        List<Vertex> convexHullpoints = new List<Vertex>();
        for (int i = 0; i < points.Count; i++)
        {
            convexHullpoints.Add(new Vertex(points[i]));
        }

        List<Triangle> triangles = new List<Triangle>();

        for (int i = 2; i < convexHullpoints.Count; i++)
        {
            Vertex a = convexHullpoints[0];
            Vertex b = convexHullpoints[i - 1];
            Vertex c = convexHullpoints[i];

            triangles.Add(new Triangle(a, b, c));
        }

        return triangles;
    }
    public List<Triangle> TriangulateConvexPolygon(List<Vertex> convexHullpoints)
    {
        List<Triangle> triangles = new List<Triangle>();
        for (int i = 2; i < convexHullpoints.Count; i++)
        {
            Vertex a = convexHullpoints[0];
            Vertex b = convexHullpoints[i - 1];
            Vertex c = convexHullpoints[i];

            triangles.Add(new Triangle(a, b, c));
        }
        return triangles;
    }
    //From http://totologic.blogspot.se/2014/01/accurate-point-in-triangle-test.html
    //p is the testpoint, and the other points are corners in the triangle
    public bool IsPointInTriangle(Vector2 p1, Vector2 p2, Vector2 p3, Vector2 p)
    {
        bool isWithinTriangle = false;

        //Based on Barycentric coordinates
        float denominator = ((p2.y - p3.y) * (p1.x - p3.x) + (p3.x - p2.x) * (p1.y - p3.y));

        float a = ((p2.y - p3.y) * (p.x - p3.x) + (p3.x - p2.x) * (p.y - p3.y)) / denominator;
        float b = ((p3.y - p1.y) * (p.x - p3.x) + (p1.x - p3.x) * (p.y - p3.y)) / denominator;
        float c = 1 - a - b;

        //The point is within the triangle or on the border if 0 <= a <= 1 and 0 <= b <= 1 and 0 <= c <= 1
        //if (a >= 0f && a <= 1f && b >= 0f && b <= 1f && c >= 0f && c <= 1f)
        //{
        //    isWithinTriangle = true;
        //}

        //The point is within the triangle
        if (a > 0f && a < 1f && b > 0f && b < 1f && c > 0f && c < 1f)
        {
            isWithinTriangle = true;
        }

        return isWithinTriangle;
    }


    // Incremental Algorithm:
    // According to Wikipedia, we should sort the points according to x-coordinates.
    // The first three points determine a triangle. Consider the next point in the ordered set 
    // and connect it with all previously considered points which are visible to the point. 
    // Continue this process of adding points until all has been processed.
    // The trick here is how can we say an edge is visible to a point? What I came up with is 
    // that we should find the center of an edge belonging to a triangle and connect this center 
    // point with the point we want to add. If this edge doesn't intersect another edge, then it's visible. 
    public List<Triangle> IncrementalTriangulation(List<Vertex> points)
    {
        if (points == null)
        {
            UnityEngine.Random.InitState(seed);
            int numberOfPoints = 150;
            List<Vector3> pointsVec3 = sampler.SamplePositions(numberOfPoints);

            foreach (Vector3 point in pointsVec3)
            {
                Debug.DrawLine(point, point + Vector3.up * 10, Color.red, 1000f);
            }

            points = new List<Vertex>();
            foreach (Vector3 point in pointsVec3)
            {
                points.Add(new Vertex(point));
            }
        }

        List<Triangle> triangles = new List<Triangle>();

        // Sort the points along x-axis
        // OrderBy is always sorting in ascending order - use OrderByDescending to get in the other order
        points = points.OrderBy(n => n.position.x).ToList();

        // The first 3 vertices are always forming a triangle.
        Triangle newTriangle = new Triangle(points[0].position, points[1].position, points[2].position);
        triangles.Add(newTriangle);

        // All edges that form the triangles, so we have something to test against
        List<Edge> edges = new List<Edge>();
        edges.Add(new Edge(newTriangle.v1, newTriangle.v2));
        edges.Add(new Edge(newTriangle.v2, newTriangle.v3));
        edges.Add(new Edge(newTriangle.v3, newTriangle.v1));

        // Add the other triangles one by one
        // Starts at 3 because we have already added 0,1,2
        for (int i = 3; i < points.Count; i++)
        {
            Vector3 currentPoint = points[i].position;

            // The edges we add this loop or we will get stuck in an endless loop
            List<Edge> newEdges = new List<Edge>();

            // Is this edge visible? We only need to check if the midpoint of the edge is visible 
            for (int j = 0; j < edges.Count; j++)
            {
                Edge currentEdge = edges[j];
                Vector3 midPoint = (currentEdge.v1.position + currentEdge.v2.position) / 2f;
                Edge edgeToMidpoint = new Edge(currentPoint, midPoint);

                // Check if this line is intersecting
                bool canSeeEdge = true;
                for (int k = 0; k < edges.Count; k++)
                {
                    // Dont compare the edge with itself
                    if (k == j) continue;
                    if (AreEdgesIntersecting(edgeToMidpoint, edges[k]))
                    {
                        canSeeEdge = false;
                        break;
                    }
                }

                // This is a valid triangle
                if (canSeeEdge)
                {
                    Edge edgeToPoint1 = new Edge(currentEdge.v1, new Vertex(currentPoint));
                    Edge edgeToPoint2 = new Edge(currentEdge.v2, new Vertex(currentPoint));

                    newEdges.Add(edgeToPoint1);
                    newEdges.Add(edgeToPoint2);

                    Triangle newTri = new Triangle(edgeToPoint1.v1, edgeToPoint1.v2, edgeToPoint2.v1);
                    triangles.Add(newTri);
                }
            }

            for (int j = 0; j < newEdges.Count; j++)
            {
                edges.Add(newEdges[j]);
            }
        }

        // foreach (Edge edge in edges)
        // {
        //     Debug.DrawLine(edge.v1.position, edge.v2.position, Color.red, 1000f);
        // }

        return triangles;
    }
    public bool Feasible(Vector3 node)
    {
        int i = info.get_i_index(node.x);
        int j = info.get_j_index(node.z);
        // 0.0 = air/free, 1.0 = block/obstacle
        if (info.traversability[i, j] < 0.5f)
            return true; // free space
        return false; // obstacle
    }

    // Approximate dynamic collision checking method: (Vector3 version)
    private bool VisibleRecurse(Vector3 nodeA, Vector3 nodeB, float distEpsilon = 0.001f)
    {
        if (Vector3.Distance(nodeA, nodeB) < distEpsilon)
        {
            return true;
        }
        Vector3 m = (nodeA + nodeB) / 2;
        if (!Feasible(m))
        {
            return false;
        }
        return VisibleRecurse(nodeA, m) && VisibleRecurse(m, nodeB);
    }

    private bool IsEdgeInsideObstacle(Vector3 nodeA, Vector3 nodeB) // Edge did hit an obstacle
    {
        RaycastHit hit;
        return Physics.SphereCast(
            nodeA,
            10f,
            nodeB - nodeA,
            out hit,
            Vector3.Distance(nodeA, nodeB),
            layerMask: LayerMask.GetMask("Obstacles")
        );
    }


    private bool AreEdgesIntersecting(Edge edge1, Edge edge2)
    {
        Vector2 l1_p1 = new Vector2(edge1.v1.position.x, edge1.v1.position.z);
        Vector2 l1_p2 = new Vector2(edge1.v2.position.x, edge1.v2.position.z);

        Vector2 l2_p1 = new Vector2(edge2.v1.position.x, edge2.v1.position.z);
        Vector2 l2_p2 = new Vector2(edge2.v2.position.x, edge2.v2.position.z);

        bool isIntersecting = AreLinesIntersecting(l1_p1, l1_p2, l2_p1, l2_p2, true);

        return isIntersecting;
    }

    // Are two lines intersecting?
    // http://thirdpartyninjas.com/blog/2008/10/07/line-segment-intersection/
    public bool AreLinesIntersecting(Vector2 l1_p1, Vector2 l1_p2, Vector2 l2_p1, Vector2 l2_p2, bool shouldIncludeEndPoints)
    {
        bool isIntersecting = false;

        float denominator = (l2_p2.y - l2_p1.y) * (l1_p2.x - l1_p1.x) - (l2_p2.x - l2_p1.x) * (l1_p2.y - l1_p1.y);

        // Make sure the denominator is > 0, if not the lines are parallel
        if (denominator != 0f)
        {
            float u_a = ((l2_p2.x - l2_p1.x) * (l1_p1.y - l2_p1.y) - (l2_p2.y - l2_p1.y) * (l1_p1.x - l2_p1.x)) / denominator;
            float u_b = ((l1_p2.x - l1_p1.x) * (l1_p1.y - l2_p1.y) - (l1_p2.y - l1_p1.y) * (l1_p1.x - l2_p1.x)) / denominator;

            // Are the line segments intersecting if the end points are the same
            if (shouldIncludeEndPoints)
            {
                // Is intersecting if u_a and u_b are between 0 and 1 or exactly 0 or 1
                if (u_a >= 0f && u_a <= 1f && u_b >= 0f && u_b <= 1f)
                {
                    isIntersecting = true;
                }
            }
            else
            {
                // Is intersecting if u_a and u_b are between 0 and 1
                if (u_a > 0f && u_a < 1f && u_b > 0f && u_b < 1f)
                {
                    isIntersecting = true;
                }
            }

        }

        return isIntersecting;
    }

    // From triangle where each triangle has one vertex to half edge
    public List<HalfEdge> TransformFromTriangleToHalfEdge(List<Triangle> triangles)
    {
        // Make sure the triangles have the same orientation
        OrientTrianglesClockwise(triangles);

        // First create a list with all possible half-edges
        List<HalfEdge> halfEdges = new List<HalfEdge>(triangles.Count * 3);

        for (int i = 0; i < triangles.Count; i++)
        {
            Triangle t = triangles[i];

            HalfEdge he1 = new HalfEdge(t.v1);
            HalfEdge he2 = new HalfEdge(t.v2);
            HalfEdge he3 = new HalfEdge(t.v3);

            he1.nextEdge = he2;
            he2.nextEdge = he3;
            he3.nextEdge = he1;

            he1.prevEdge = he3;
            he2.prevEdge = he1;
            he3.prevEdge = he2;

            // The vertex needs to know of an edge going from it
            he1.v.halfEdge = he2;
            he2.v.halfEdge = he3;
            he3.v.halfEdge = he1;

            // The face the half-edge is connected to
            t.halfEdge = he1;

            he1.t = t;
            he2.t = t;
            he3.t = t;

            // Add the half-edges to the list
            halfEdges.Add(he1);
            halfEdges.Add(he2);
            halfEdges.Add(he3);
        }

        // Find the half-edges going in the opposite direction
        for (int i = 0; i < halfEdges.Count; i++)
        {
            HalfEdge he = halfEdges[i];

            Vertex goingToVertex = he.v;
            Vertex goingFromVertex = he.prevEdge.v;

            for (int j = 0; j < halfEdges.Count; j++)
            {
                // Dont compare with itself
                if (i == j) continue;
                HalfEdge heOpposite = halfEdges[j];

                // Is this edge going between the vertices in the opposite direction
                if (goingFromVertex.position == heOpposite.v.position && goingToVertex.position == heOpposite.prevEdge.v.position)
                {
                    he.oppositeEdge = heOpposite;
                    break;
                }
            }
        }

        return halfEdges;
    }

    // Orient triangles so they have the correct orientation
    public void OrientTrianglesClockwise(List<Triangle> triangles)
    {
        for (int i = 0; i < triangles.Count; i++)
        {
            Triangle tri = triangles[i];

            Vector2 v1 = new Vector2(tri.v1.position.x, tri.v1.position.z);
            Vector2 v2 = new Vector2(tri.v2.position.x, tri.v2.position.z);
            Vector2 v3 = new Vector2(tri.v3.position.x, tri.v3.position.z);

            if (!IsTriangleOrientedClockwise(v1, v2, v3))
            {
                tri.ChangeOrientation();
            }
        }
    }

    // Is a triangle oriented clockwise
    // Is a triangle in 2d space oriented clockwise or counter-clockwise
    // https://math.stackexchange.com/questions/1324179/how-to-tell-if-3-connected-points-are-connected-clockwise-or-counter-clockwise
    // https://en.wikipedia.org/wiki/Curve_orientation
    public bool IsTriangleOrientedClockwise(Vector2 p1, Vector2 p2, Vector2 p3)
    {
        bool isClockWise = true;

        float determinant = p1.x * p2.y + p3.x * p1.y + p2.x * p3.y - p1.x * p3.y - p3.x * p2.y - p2.x * p1.y;

        if (determinant > 0f)
        {
            isClockWise = false;
        }

        return isClockWise;
    }

    // Is a point d inside, outside or on the same circle as a, b, c
    // https://gamedev.stackexchange.com/questions/71328/how-can-i-add-and-subtract-convex-polygons
    // Returns positive if inside, negative if outside, and 0 if on the circle
    public float IsPointInsideOutsideOrOnCircle(Vector2 aVec, Vector2 bVec, Vector2 cVec, Vector2 dVec)
    {
        // This first part will simplify how we calculate the determinant
        float a = aVec.x - dVec.x;
        float d = bVec.x - dVec.x;
        float g = cVec.x - dVec.x;

        float b = aVec.y - dVec.y;
        float e = bVec.y - dVec.y;
        float h = cVec.y - dVec.y;

        float c = a * a + b * b;
        float f = d * d + e * e;
        float i = g * g + h * h;

        float determinant = (a * e * i) + (b * f * g) + (c * d * h) - (g * e * c) - (h * f * a) - (i * d * b);

        return determinant;
    }

    // Is a quadrilateral convex? Assume no 3 points are colinear and the shape doesnt look like an hourglass
    public bool IsQuadrilateralConvex(Vector2 a, Vector2 b, Vector2 c, Vector2 d)
    {
        bool isConvex = false;

        bool abc = IsTriangleOrientedClockwise(a, b, c);
        bool abd = IsTriangleOrientedClockwise(a, b, d);
        bool bcd = IsTriangleOrientedClockwise(b, c, d);
        bool cad = IsTriangleOrientedClockwise(c, a, d);

        if (abc && abd && bcd & !cad)
        {
            isConvex = true;
        }
        else if (abc && abd && !bcd & cad)
        {
            isConvex = true;
        }
        else if (abc && !abd && bcd & cad)
        {
            isConvex = true;
        }
        // The opposite sign, which makes everything inverted
        else if (!abc && !abd && !bcd & cad)
        {
            isConvex = true;
        }
        else if (!abc && !abd && bcd & !cad)
        {
            isConvex = true;
        }
        else if (!abc && abd && !bcd & !cad)
        {
            isConvex = true;
        }

        return isConvex;
    }

    //Flip an edge
    public void FlipEdge(HalfEdge one)
    {
        // The data we need
        // This edge's triangle
        HalfEdge two = one.nextEdge;
        HalfEdge three = one.prevEdge;
        // The opposite edge's triangle
        HalfEdge four = one.oppositeEdge;
        HalfEdge five = one.oppositeEdge.nextEdge;
        HalfEdge six = one.oppositeEdge.prevEdge;
        // The vertices
        Vertex a = one.v;
        Vertex b = one.nextEdge.v;
        Vertex c = one.prevEdge.v;
        Vertex d = one.oppositeEdge.nextEdge.v;


        // Flip

        // Change vertex
        a.halfEdge = one.nextEdge;
        c.halfEdge = one.oppositeEdge.nextEdge;

        // Change half-edge
        // Half-edge - half-edge connections
        one.nextEdge = three;
        one.prevEdge = five;

        two.nextEdge = four;
        two.prevEdge = six;

        three.nextEdge = five;
        three.prevEdge = one;

        four.nextEdge = six;
        four.prevEdge = two;

        five.nextEdge = one;
        five.prevEdge = three;

        six.nextEdge = two;
        six.prevEdge = four;

        // Half-edge - vertex connection
        one.v = b;
        two.v = b;
        three.v = c;
        four.v = d;
        five.v = d;
        six.v = a;

        // Half-edge - triangle connection
        Triangle t1 = one.t;
        Triangle t2 = four.t;

        one.t = t1;
        three.t = t1;
        five.t = t1;

        two.t = t2;
        four.t = t2;
        six.t = t2;

        // Opposite-edges are not changing!

        // Triangle connection
        t1.v1 = b;
        t1.v2 = c;
        t1.v3 = d;

        t2.v1 = b;
        t2.v2 = d;
        t2.v3 = a;

        t1.halfEdge = three;
        t2.halfEdge = four;
    }

    // private bool ObstacleFreeEdge(Vector3 nodeA, Vector3 nodeB)
    // {
    //     RaycastHit hit;
    //     return !Physics.SphereCast(
    //         nodeA,
    //         3f,
    //         nodeB - nodeA,
    //         out hit,
    //         Vector3.Distance(nodeA, nodeB),
    //         layerMask: LayerMask.GetMask("Obstacles")
    //     );
    // }

}
