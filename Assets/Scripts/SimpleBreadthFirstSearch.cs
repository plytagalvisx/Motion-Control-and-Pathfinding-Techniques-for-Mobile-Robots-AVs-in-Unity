using UnityEngine.Assertions;
using Unity.Collections;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using UnityEditor;
using System.Linq;

public class SimpleBreadthFirstSearch
{
    public static Vector3 start_pos;
    public static Vector3 goal_pos;

    public SimpleBreadthFirstSearch(Vector3 start, Vector3 goal)
    {
        start_pos = start;
        goal_pos = goal;
    }

    // SimpleBreadthFirstSearch doesn’t actually construct the paths; it only tells us how to visit everything on the grid map.
    public static HashSet<Vector3> Search()
    {
        Queue<Vector3> frontier = new Queue<Vector3>();
        HashSet<Vector3> reached = new HashSet<Vector3>();
        frontier.Enqueue(start_pos);
        reached.Add(start_pos);

        while (frontier.Count > 0)
        {
            Vector3 current = frontier.Dequeue();
            if (current == goal_pos)
            {
                break;
            }
            foreach (Vector3 next in Utils.GetNeighbors(current))
            {
                if (!reached.Contains(next))
                {
                    frontier.Enqueue(next);
                    reached.Add(next);
                }
            }
        }
        return reached;
    }
}