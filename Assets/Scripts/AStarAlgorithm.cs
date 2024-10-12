using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Linq;

public class AStarAlgorithm
{

    /*
        This method uses the A* algorithm to find the shortest path between a given start point and a goal point, through a set of (padded) 
        corners represented by an adjacency matrix. The method takes in two inputs, an adjacency matrix of floats and a heuristic array of 
        floats. It returns a list of integers that represent the shortest path. The method uses a HashSet to keep track of the unvisited 
        and visited nodes, and it uses the List.Insert() method to build the path backwards from the goal point to the origin point.
    */
    public static List<int> GetShortestPath(float[,] adjacencies, float[] heuristic)
    {
        int n = adjacencies.GetLength(0); // Number of nodes
        float[] f = new float[n]; // Total cost (distance + heuristic). I.e. the list of F scores. fScore[v] = gScore[v] + hScore[v]
        float[] g = new float[n]; // Distance from start point. I.e. the list of G scores. gScore[v] is the distance from the start vertex to vertex v
        int[] prev = new int[n]; // Previous node

        var open = new HashSet<int>();  // Queue of unvisited nodes
        var closed = new HashSet<int>();  // Queue of visited nodes

        for (int i = 0; i < n; i++)
        {
            g[i] = Mathf.Infinity;
            f[i] = Mathf.Infinity;
            prev[i] = -1;
            open.Add(i);
        }

        g[0] = 0;
        f[0] = heuristic[0];

        while (open.Count > 0)
        { // While there are unvisited nodes
            // Get the (current) node with the lowest f value
            int current = open.OrderBy(x => f[x]).First(); // Get the node with the lowest f value
            open.Remove(current);
            closed.Add(current);

            for (int neighbor = 0; neighbor < n; neighbor++)
            {
                if (neighbor == current) continue;
                if (adjacencies[current, neighbor] > 0)
                { // If there is an edge between the current node and the neighbor (if they are neighbors)
                    float tentative_g = g[current] + adjacencies[current, neighbor];
                    if (tentative_g < g[neighbor])
                    {
                        g[neighbor] = tentative_g;
                        f[neighbor] = g[neighbor] + heuristic[neighbor];
                        prev[neighbor] = current;
                    }
                }
            }
        }

        // Reconstruct the path
        var shortest_path = new List<int>();
        int point = 1;
        while (point != 0)
        {
            if (point == -1 || point > prev.Length - 1)
            {
                throw new System.Exception();
            }
            shortest_path.Insert(0, point);
            point = prev[point];
        }
        shortest_path.Insert(0, 0); // Add start point

        return shortest_path;
    }
}