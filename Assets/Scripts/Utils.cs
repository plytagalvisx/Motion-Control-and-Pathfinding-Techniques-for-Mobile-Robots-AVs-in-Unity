using UnityEngine.Assertions;
using Unity.Collections;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using UnityEditor;
using System.Linq;

public class Utils
{
    public static TerrainInfo terrain;

    public Utils(TerrainInfo terrain_)
    {
        terrain = terrain_;
    }

    public static bool isTraversable(float[,] traversability, int row, int col)
    {
        return traversability[row, col] < 0.5f;
    }


    // Heuristic function that tells us how close we are to the goal position/location
    public static float Heuristic(Vector3 a, Vector3 b)
    {
        // Manhattan distance on a square grid
        return Mathf.Abs(a.x - b.x) + Mathf.Abs(a.z - b.z);
    }

    // Produces a different path than Heuristic() function
    public static float Heuristic2(Vector3 a, Vector3 b)
    {
        // returns the heuristic cost from start to goal
        return Vector3.Distance(a, b);
    }

    public static float CostToEnterTile(Vector3 source, Vector3 target) // This is literally a heuristic function
    {
        float distance = Vector3.Distance(source, target);
        return distance;
    }

    public static Vector3[] GetNeighbors(Vector3 current) // For grid map terrain
    {
        // Check if traversable and return neighbors
        float[,] traversabilityMatrix = terrain.GetTraversability();
        int rows = traversabilityMatrix.GetLength(0);
        int columns = traversabilityMatrix.GetLength(1);
        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < columns; c++)
            {
                int i = terrain.get_i_index(current.x); // from world coordinates to matrix indices. In other words, from continuous coordinates to discrete coordinates.
                int j = terrain.get_j_index(current.z);
                if (isTraversable(traversabilityMatrix, i, j))
                {
                    Vector3 left = new Vector3(current.x - 1, current.y, current.z);
                    Vector3 right = new Vector3(current.x + 1, current.y, current.z);
                    Vector3 up = new Vector3(current.x, current.y, current.z + 1);
                    Vector3 down = new Vector3(current.x, current.y, current.z - 1);
                    return new Vector3[] { left, right, up, down };
                }
            }
        }
        return new Vector3[] { };
    }

    public static Vector3[] GetNeighborsForInflatedCspaceMAp(Vector3 current) // For inflated obstacle C-space map
    {
        int robot_radius = 1;
        float[,] grid_map = terrain.GetTraversability();
        float[,] cspace_map = CreateCspaceMapAndInflateIt(grid_map, robot_radius);
        int rows = cspace_map.GetLength(0);
        int columns = cspace_map.GetLength(1);

        for (int r = 0; r < rows; r++) // rows
        {
            for (int c = 0; c < columns; c++) // columns
            {
                if (isTraversable(cspace_map, r, c))
                {
                    Vector3 left = new Vector3(current.x - 1, current.y, current.z);
                    Vector3 right = new Vector3(current.x + 1, current.y, current.z);
                    Vector3 up = new Vector3(current.x, current.y, current.z + 1);
                    Vector3 down = new Vector3(current.x, current.y, current.z - 1);
                    return new Vector3[] { left, right, up, down };
                }
            }
        }
        return new Vector3[] { };
    }


    // Create a simple graph containing Vector3 positions
    public static Graph<Vector3> CreateSimpleGraph()
    {
        Graph<Vector3> graph = new Graph<Vector3>();
        graph.edges = new Dictionary<Vector3, List<Vector3>>();
        float[,] traversabilityMatrix = terrain.GetTraversability();
        int rows = traversabilityMatrix.GetLength(0);
        int columns = traversabilityMatrix.GetLength(1);

        // Vector3 s_left = new Vector3(start_pos.x - 5, start_pos.y, start_pos.z);
        // Vector3 s_right = new Vector3(start_pos.x + 5, start_pos.y, start_pos.z);
        // Vector3 s_up = new Vector3(start_pos.x, start_pos.y, start_pos.z + 5);
        // Vector3 s_down = new Vector3(start_pos.x, start_pos.y, start_pos.z - 5);
        // graph.edges[start_pos].Add(s_left);
        // graph.edges[start_pos].Add(s_right);
        // graph.edges[start_pos].Add(s_up);
        // graph.edges[start_pos].Add(s_down);

        // Vector3 g_left = new Vector3(goal_pos.x - 5, goal_pos.y, goal_pos.z);
        // Vector3 g_right = new Vector3(goal_pos.x + 5, goal_pos.y, goal_pos.z);
        // Vector3 g_up = new Vector3(goal_pos.x, goal_pos.y, goal_pos.z + 5);
        // Vector3 g_down = new Vector3(goal_pos.x, goal_pos.y, goal_pos.z - 5);
        // graph.edges[goal_pos].Add(g_left);
        // graph.edges[goal_pos].Add(g_right);
        // graph.edges[goal_pos].Add(g_up);
        // graph.edges[goal_pos].Add(g_down);

        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < columns; c++)
            {
                if (isTraversable(traversabilityMatrix, r, c))
                {
                    Debug.Log("r: " + r + ", c: " + c);
                    Debug.Log("x: " + terrain.get_x_pos(r) + ", z: " + terrain.get_z_pos(c));
                    Vector3 current = new Vector3(terrain.get_x_pos(r), 0.0f, terrain.get_z_pos(c)); // from matrix indices to world coordinates
                    graph.edges[current] = new List<Vector3>();
                    if (r > 0 && isTraversable(traversabilityMatrix, r - 1, c))
                    {
                        Vector3 left = new Vector3(terrain.get_x_pos(r - 1), 0.0f, terrain.get_z_pos(c));
                        graph.edges[current].Add(left);
                    }
                    if (r < rows - 1 && isTraversable(traversabilityMatrix, r + 1, c))
                    {
                        Vector3 right = new Vector3(terrain.get_x_pos(r + 1), 0.0f, terrain.get_z_pos(c));
                        graph.edges[current].Add(right);
                    }
                    if (c > 0 && isTraversable(traversabilityMatrix, r, c - 1))
                    {
                        Vector3 down = new Vector3(terrain.get_x_pos(r), 0.0f, terrain.get_z_pos(c - 1));
                        graph.edges[current].Add(down);
                    }
                    if (c < columns - 1 && isTraversable(traversabilityMatrix, r, c + 1))
                    {
                        Vector3 up = new Vector3(terrain.get_x_pos(r), 0.0f, terrain.get_z_pos(c + 1));
                        graph.edges[current].Add(up);
                    }

                    // Vector3 currentPosition = new Vector3(terrain.get_x_pos(r), 0.0f, terrain.get_z_pos(c));
                    // graph.edges[currentPosition] = new List<Vector3>();
                    // for (int i = -1; i <= 1; i += 2) 
                    // {
                    //     int nr = r + i; // neighbor row
                    //     if (nr >= 0 && nr < rows && isTraversable(traversabilityMatrix, nr, c)) {
                    //         Vector3 neighborPosition = new Vector3(terrain.get_x_pos(nr), 0.0f, terrain.get_z_pos(c));
                    //         graph.edges[currentPosition].Add(neighborPosition);
                    //     }
                    //     int nc = c + i; // neighbor column
                    //     if (nc >= 0 && nc < columns && isTraversable(traversabilityMatrix, r, nc)) {
                    //         Vector3 neighborPosition = new Vector3(terrain.get_x_pos(r), 0.0f, terrain.get_z_pos(nc));
                    //         graph.edges[currentPosition].Add(neighborPosition);
                    //     }
                    // }
                }
            }
        }
        return graph;
    }

    // Create a simple grid containing float values
    // public static float[,] CreateSimpleFloatGrid()
    // {
    //     float[,] grid = new float[terrain.x_N, terrain.z_N];
    //     float[,] traversabilityMatrix = terrain.GetTraversability();
    //     int rows = traversabilityMatrix.GetLength(0);
    //     int columns = traversabilityMatrix.GetLength(1);

    //     for (int r = 0; r < rows; r++) // rows
    //     {
    //         for (int c = 0; c < columns; c++) // columns
    //         {
    //             if (isTraversable(traversabilityMatrix, r, c))
    //             {
    //                 grid[r, c] = 0.0f;
    //             }
    //             else
    //             {
    //                 grid[r, c] = 1.0f;
    //             }
    //         }
    //     }

    //     return grid;
    // }

    // Create a simple grid containing Vector3 positions
    // public static List<Vector3> CreateSimpleVector3Grid()
    // {
    //     List<Vector3> grid = new List<Vector3>();
    //     float[,] traversabilityMatrix = terrain.GetTraversability();
    //     int rows = traversabilityMatrix.GetLength(0);
    //     int columns = traversabilityMatrix.GetLength(1);

    //     for (int r = 0; r < rows; r++) // rows
    //     {
    //         for (int c = 0; c < columns; c++) // columns
    //         {
    //             if (isTraversable(traversabilityMatrix, r, c))
    //             {
    //                 Vector3 current = new Vector3(terrain.get_x_pos(r), 0.0f, terrain.get_z_pos(c));
    //                 grid.Add(current);
    //             }
    //         }
    //     }

    //     return grid;
    // }

    // Create a C-space (configuration space) map
    public static float[,] CreateCspaceMapAndInflateIt(float[,] grid_map, int robot_radius)
    {
        float[,] cspace_map = new float[grid_map.GetLength(0), grid_map.GetLength(1)];
        int rows = grid_map.GetLength(0);
        int columns = grid_map.GetLength(1);

        // Insert the grid map into the cspace map
        for (int r = 0; r < grid_map.GetLength(0); r++) // rows
        {
            for (int c = 0; c < grid_map.GetLength(1); c++) // columns
            {
                cspace_map[r, c] = grid_map[r, c];
            }
        }

        // Inflate the obstacles
        for (int x = 0; x < rows; x++) // rows
        {
            for (int y = 0; y < columns; y++) // columns
            {
                if (grid_map[x, y] == 1.0f)
                {
                    for (int xr = -robot_radius; xr <= robot_radius; xr++)
                    {
                        for (int yr = -robot_radius; yr <= robot_radius; yr++)
                        {
                            int inflating_cell_x = x + xr; // neighbor row
                            int inflating_cell_y = y + yr; // neighbor column
                            if (inflating_cell_x >= 0 && inflating_cell_x < rows && inflating_cell_y >= 0 && inflating_cell_y < columns)
                            {
                                float distToRobotRadius = Mathf.Sqrt(Mathf.Pow(xr, 2) + Mathf.Pow(yr, 2));
                                if (grid_map[inflating_cell_x, inflating_cell_y] == 0.0f && distToRobotRadius <= robot_radius) // if the cell is not an obstacle and is within the robot radius
                                {
                                    cspace_map[inflating_cell_x, inflating_cell_y] = 1.0f;
                                }
                            }
                        }
                    }
                }
            }
        }

        return cspace_map;
    }

}
