using UnityEngine.Assertions;
using Unity.Collections;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using UnityEditor;
using System.Linq;

public class DijkstrasAlgorithm
{
    public static Vector3 start_pos;
    public static Vector3 goal_pos;

    public DijkstrasAlgorithm(Vector3 start, Vector3 goal)
    {
        start_pos = start;
        goal_pos = goal;
    }

    public static Dictionary<Vector3, Vector3> Search() // aka Uniform Cost Search
    {
        // create priority queue
        PriorityQueue<Vector3, float> frontier = new PriorityQueue<Vector3, float>(); // Using a priority queue instead of a regular queue changes the way the frontier expands.
        Dictionary<Vector3, Vector3> came_from = new Dictionary<Vector3, Vector3>();
        Dictionary<Vector3, float> cost_so_far = new Dictionary<Vector3, float>();
        frontier.Enqueue(start_pos, 0);
        came_from[start_pos] = Vector3.zero;
        cost_so_far[start_pos] = 0;

        while (frontier.Count > 0)
        {
            Vector3 current = frontier.Dequeue();
            if (current == goal_pos)
            {
                break;
            }
            foreach (Vector3 next in Utils.GetNeighbors(current))
            {
                float new_cost = cost_so_far[current] + Utils.CostToEnterTile(current, next); // Movement costs other than 1 allow us to explore more interesting graphs, not only grids. Movement costs can also be used to avoid or prefer areas based on proximity to enemies or allies.
                if (!cost_so_far.ContainsKey(next) || new_cost < cost_so_far[next]) // OBS! Movement costs on the maps become arbitrary weights on graph edges. The heuristics don’t translate as easily to arbitrary maps; you have to design a heuristic for each type of graph. For planar maps, distances are a good choice.
                {
                    cost_so_far[next] = new_cost;
                    float priority = new_cost;
                    frontier.Enqueue(next, priority);
                    came_from[next] = current;
                }
            }
        }
        return came_from;
    }
}