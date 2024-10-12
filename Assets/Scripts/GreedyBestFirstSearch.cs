using UnityEngine.Assertions;
using Unity.Collections;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using UnityEditor;
using System.Linq;

public class GreedyBestFirstSearch
{
    public Vector3 start_pos;
    public Vector3 goal_pos;

    public GreedyBestFirstSearch(Vector3 start, Vector3 goal)
    {
        start_pos = start;
        goal_pos = goal;
    }

    // In DijkstrasAlgorithm() we used the actual distance from the start for the priority queue ordering. 
    // Here instead, in GreedyBestFirstSearch(), we’ll use the estimated distance to the goal for the priority queue ordering.
    // The location/node closest to the goal will be explored first.
    // This method produces paths that aren’t the shortest. So this algorithm runs faster when there aren’t a lot of obstacles, 
    // but the paths aren’t as good. To fix this we use the A* algorithm.
    public Dictionary<Vector3, Vector3> Search()
    {
        PriorityQueue<Vector3, float> frontier = new PriorityQueue<Vector3, float>();
        Dictionary<Vector3, Vector3> came_from = new Dictionary<Vector3, Vector3>();
        frontier.Enqueue(start_pos, 0);
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
                    float priority = Utils.Heuristic(goal_pos, next);
                    frontier.Enqueue(next, priority);
                    came_from[next] = current;
                }
            }
        }
        return came_from;
    }
}