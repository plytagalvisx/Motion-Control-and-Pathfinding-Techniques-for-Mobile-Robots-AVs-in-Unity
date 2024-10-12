using UnityEngine.Assertions;
using Unity.Collections;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using UnityEditor;
using System.Linq;

public class ImprovedBreadthFirstSearch
{
    public static Vector3 start_pos;
    public static Vector3 goal_pos;

    public ImprovedBreadthFirstSearch(Vector3 start, Vector3 goal)
    {
        start_pos = start;
        goal_pos = goal;
    }

    // We want to use BreadthFirstSearch for finding paths, so let’s modify the loop to keep track of where we came from for every location that’s been reached.
    public static Dictionary<Vector3, Vector3> Search()
    {
        Queue<Vector3> frontier = new Queue<Vector3>();
        Dictionary<Vector3, Vector3> came_from = new Dictionary<Vector3, Vector3>();
        frontier.Enqueue(start_pos);
        came_from[start_pos] = Vector3.zero;

        while (frontier.Count > 0)
        {
            Vector3 current = frontier.Dequeue();
            if (current == goal_pos)
            {
                break;
            }
            foreach (Vector3 next in Utils.GetNeighbors(current))
            {
                if (!came_from.ContainsKey(next))
                {
                    frontier.Enqueue(next);
                    came_from[next] = current; // came_from for each location points to the place where we came from. Like "breadcrumbs".
                }
            }
        }
        return came_from;
    }
}