using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;
using System;

public class VisibilityGraph : MonoBehaviour
{

    public int margin = 6; // The margin around the terrain to be considered for the visibility graph
    public float waypointsDistance = 10;  // Distance between waypoints for augmented visibility graph
    public GameObject terrain_manager_game_object;

    // Use this for initialization
    void Start()
    {
    }

    // Update is called once per frame.
    void Update() { }

    /*
        The purpose of the GetValidCorners() method is to find and return a list of valid corners in a terrain represented by a 
        2D traversability matrix. The method takes into account the traversability of the terrain, as well as the adjacency of 
        cells to determine which corners are valid. The list of valid corners can be used for purposes, such as 
        pathfinding and navigation. The use of this method is to find the optimal path for a car/drone to take by only 
        considering the valid corners as potential waypoints avoiding obstacles.
    */
    public List<Vector3> GetValidCorners()
    {
        TerrainManager terrainManager = terrain_manager_game_object.GetComponent<TerrainManager>();
        float[,] paddedTraversabilityMatrix = terrainManager.myInfo.GetPaddedTraversability();
        float xStep = (terrainManager.myInfo.x_high - terrainManager.myInfo.x_low) / terrainManager.myInfo.x_N;
        float zStep = (terrainManager.myInfo.z_high - terrainManager.myInfo.z_low) / terrainManager.myInfo.z_N;
        float y = terrainManager.myInfo.start_pos.y;
        int rows = paddedTraversabilityMatrix.GetLength(0);
        int columns = paddedTraversabilityMatrix.GetLength(1);
        List<Vector3> validCorners = new List<Vector3>();

        int[] cornerSteps = new int[] { -1, -1, 1, 1, -1 };
        int[] adjacentSteps = new int[] { 0, -1, 0, 1, 0, -1 };
        for (int r = 1; r < rows - 1; r++)
        {
            for (int c = 1; c < columns - 1; c++)
            {
                if (IsTraversable(paddedTraversabilityMatrix, r, c) == true) { continue; }
                for (int i = 0; i < 4; i++)
                { // For each corner of the cell
                    if (IsTraversable(paddedTraversabilityMatrix, r + cornerSteps[i], c + cornerSteps[i + 1]) == false) { continue; }
                    Cell c1 = new Cell(r + adjacentSteps[i], c + adjacentSteps[i + 1]);
                    Cell c2 = new Cell(r + adjacentSteps[i + 1], c + adjacentSteps[i + 2]);
                    if (AreAdjacentCellsTraversable(paddedTraversabilityMatrix, c1, c2) == true)
                    {
                        Vector3 center = GetCenterOfCell(r, c, xStep, zStep, terrainManager);
                        Vector3 corner = GetCornerOfCell(center, cornerSteps, i, xStep, y, zStep);
                        validCorners.Add(corner);
                    }
                }
            }
        }
        return validCorners;
    }
    private bool IsTraversable(float[,] traversability, int row, int col)
    {
        return traversability[row, col] == 0.0f;
    }
    private bool AreAdjacentCellsTraversable(float[,] traversability, Cell c1, Cell c2)
    {
        return (IsTraversable(traversability, c1.row, c1.col) == true && IsTraversable(traversability, c2.row, c2.col) == true) || (IsTraversable(traversability, c1.row, c1.col) == false && IsTraversable(traversability, c2.row, c2.col) == false);
    }
    private bool AreAdjacentCells(Vector3 c1, Vector3 c2)
    {
        return (c1.x == c2.x && Math.Abs(c1.z - c2.z) == 1) || (c1.z == c2.z && Math.Abs(c1.x - c2.x) == 1);
    }

    private Vector3 GetCenterOfCell(int row, int col, float xStep, float zStep, TerrainManager terrainManager)
    {
        return new Vector3(terrainManager.myInfo.x_low + (row - 1 + 0.5f) * xStep, terrainManager.myInfo.start_pos.y, terrainManager.myInfo.z_low + (col - 1 + 0.5f) * zStep);
    }
    private Vector3 GetCornerOfCell(Vector3 center, int[] cornerSteps, int cornerIndex, float xStep, float y, float zStep)
    {
        return new Vector3(center.x + cornerSteps[cornerIndex] * (xStep / 2 + margin / Mathf.Sqrt(2)), y, center.z + cornerSteps[cornerIndex + 1] * (zStep / 2 + margin / Mathf.Sqrt(2)));
    }

    /*
        Returns a list of waypoints along the shortest planned path that the algorithm is 
        using to navigate from the start position to the goal position.
    */
    public List<Vector3> GetShortestPlannedPathWayPoints()
    {
        TerrainManager terrainManager = terrain_manager_game_object.GetComponent<TerrainManager>();
        List<Vector3> pathPoints = new List<Vector3>();
        List<Vector3> corners = new List<Vector3> { terrainManager.myInfo.start_pos, terrainManager.myInfo.goal_pos };
        corners.AddRange(GetValidCorners());

        List<int> pathIndexes = AStarAlgorithm.GetShortestPath(GetAdjacencyMatrix(corners), GetHeuristic(corners, terrainManager.myInfo.goal_pos));
        foreach (int pathIndex in pathIndexes)
        {
            pathPoints.Add(corners[pathIndex]);
        }
        return pathPoints;
    }

    /*
        GetHeuristic() method returns a list of heuristic values for each vertex in the graph.
    */
    private float[] GetHeuristic(List<Vector3> corners, Vector3 goal)
    {
        float[] heuristic = new float[corners.Count];
        for (int i = 0; i < corners.Count; i++)
        {
            heuristic[i] = EuclideanDistance(corners[i], goal);
        }
        return heuristic;
    }

    /*
        A heuristic helper method that computes the Euclidean distance between two vertices.
    */
    private float EuclideanDistance(Vector3 p1, Vector3 p2)
    {
        return Mathf.Sqrt(Mathf.Pow(p1.x - p2.x, 2) + Mathf.Pow(p1.z - p2.z, 2));
    }

    private float ManhattanDistance(Vector3 p1, Vector3 p2)
    {
        return Mathf.Abs(p1.x - p2.x) + Mathf.Abs(p1.z - p2.z);
    }

    /*
        Explanation: Let's say we want to move point A 1.5 units towards point B.
        A and B could be anywhere and at any distance between them (so magnitude could be anything).
        So to get what we want, we do B - A (subtraction), then we normalise it (the vector) and then multiply by 1.5 units.
    */
    public List<Vector3> GetAugmentedPath(List<Vector3> pathPoints, float units = 1.5f)
    { // units is the distance magnitude between two points
        List<Vector3> augmentedPath = new List<Vector3>();
        for (int i = 0; i < pathPoints.Count - 1; i++)
        {
            Vector3 directionVector = pathPoints[i + 1] - pathPoints[i]; // 
            Vector3 unitDirectionVector = directionVector.normalized;
            int NumOfPointsThatFitOnDirVector = (int)Math.Ceiling(directionVector.magnitude / units); // number of points that fit between two points in a desiredDirectionVector based on the given units
            Vector3 desiredUnitDirectionVector = unitDirectionVector * units;
            for (int j = 0; j < NumOfPointsThatFitOnDirVector; j++)
            {
                augmentedPath.Add(pathPoints[i] + desiredUnitDirectionVector * j);
            }
        }
        return augmentedPath;
    }

    /*
        The GetSmoothedPath() method takes a list of path points and returns a new list of smoothed path points
        by averaging the positions of the original path points in a window of a specified size (margin). 
        This allows for more accurate navigation along the path.
    */
    public List<Vector3> getSmoothedPath(List<Vector3> myPathVG)
    {
        List<Vector3> smoothedPath = new List<Vector3>();
        for (int i = 0; i < myPathVG.Count; i++)
        {
            Vector3 currentPoint = myPathVG[i];
            Vector3 averagePoint = Vector3.zero;
            int pointsInAverage = 0;
            for (int j = Math.Max(0, i - margin); j <= Math.Min(myPathVG.Count - 1, i + margin); j++)
            {
                averagePoint += myPathVG[j];
                pointsInAverage++;
            }
            averagePoint /= pointsInAverage;
            smoothedPath.Add(averagePoint);
        }
        return smoothedPath;
    }

    /*
        This method is generating an adjacency matrix for a given list of Vector3 corners. The adjacency matrix is a square matrix with a size equal to the number of corners, and it is used to represent the edges between the corners in a graph.
        The method starts by creating a 2D array called adjacenies, with the same size as the number of corners. It then iterates through each corner i and for each corner j, where j is greater than i.
        It then calculates the direction vector from corner i to corner j, and the normal vector of that direction, which is a vector that is perpendicular to the direction vector.
        The method then checks if there is an obstacle along the line between corner i and corner j by using the Physics.Linecast() method. If there is an obstacle, the corresponding element in the adjacency matrix is set to -1, indicating that there is no connection between those corners.
        Otherwise, if the line between the two corners is clear, it calculates the distance between the two corners and sets the corresponding element in the adjacency matrix to that distance.
        Finally, the method returns the adjacency matrix containing the distances between the corners that are connected by a clear line of sight.
    */
    float[,] GetAdjacencyMatrix(List<Vector3> corners)
    {
        int n = corners.Count;
        float[,] adjacencies = new float[n, n];

        for (int i = 0; i < n; i++)
        {
            for (int j = i + 1; j < n; j++)
            {
                if (IsLineFree(corners[i], corners[j]))
                {
                    float dist = Vector3.Distance(corners[i], corners[j]);
                    adjacencies[i, j] = dist;
                    adjacencies[j, i] = dist;
                }
                else
                {
                    adjacencies[i, j] = -1;
                    adjacencies[j, i] = -1;
                }
            }
        }
        return adjacencies;
    }

    bool IsLineFree(Vector3 a, Vector3 b)
    {
        Vector3 direction = b - a;
        Vector3 normal = new Vector3(-direction.z, direction.y, direction.x).normalized;
        float step = (margin - 0.1f) / 2;
        int[] signs = new int[] { -1, 0, 1 };
        foreach (int sign in signs)
        {
            if (Physics.Linecast(a + sign * step * normal, b + sign * step * normal))
            {
                return false;
            }
        }
        return true;
    }
}