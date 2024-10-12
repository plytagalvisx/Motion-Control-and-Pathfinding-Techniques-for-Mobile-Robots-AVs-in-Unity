using UnityEngine.Assertions;
using Unity.Collections;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using UnityEditor;
using System.Linq;

public class SimpleAStarAlgorithms
{
    public Vector3 start_pos;
    public Vector3 goal_pos;

    public SimpleAStarAlgorithms(Vector3 start, Vector3 goal)
    {
        start_pos = start;
        goal_pos = goal;
    }

    // DijkstrasAlgorithm() works well to find the shortest path, but it wastes time exploring in directions that aren’t promising. 
    // GreedyBestFirstSearch() explores in promising directions but it may not find the shortest path. 
    // The AStarAlgorithm() uses both the actual distance from the start and the estimated distance to the goal.
    // DijkstrasAlgorithm() calculates the distance from the start point. 
    // GreedyBestFirstSearch() estimates the distance to the goal point. 
    // AStarAlgorithm() is using the sum of those two distances.
    public Dictionary<Vector3, Vector3> SearchOnGridMap()
    {
        PriorityQueue<Vector3, float> frontier = new PriorityQueue<Vector3, float>();
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
                float new_cost = cost_so_far[current] + Utils.CostToEnterTile(current, next); // g_cost = current_node_cost + step_cost 
                                                                                              // Example: 
                                                                                              // g_cost = current_node.cost + self.delta_t * self.time_steps
                                                                                              // h_cost = self.calc_heuristic(goal_node, next_node)
                                                                                              // f_cost = g_cost + h_cost 
                if (!cost_so_far.ContainsKey(next) || new_cost < cost_so_far[next])
                {
                    cost_so_far[next] = new_cost;
                    float priority = new_cost + Utils.Heuristic(goal_pos, next); // f_cost = g_cost + h_cost. As the heuristic becomes smaller, A* turns into Dijkstra’s Algorithm. As the heuristic becomes larger, A* turns into Greedy Best First Search.
                    frontier.Enqueue(next, priority);
                    came_from[next] = current;
                }
            }
        }
        return came_from;
    }

    // A Star Search on a simple graph
    public Dictionary<Vector3, Vector3> SearchOnSimpleGraph(Graph<Vector3> graph)
    {
        PriorityQueue<Vector3, float> frontier = new PriorityQueue<Vector3, float>(); // Represents an open_set
        Dictionary<Vector3, Vector3> came_from = new Dictionary<Vector3, Vector3>(); // Represents a closed_set
        Dictionary<Vector3, float> cost_so_far = new Dictionary<Vector3, float>(); // Represents g_score
        frontier.Enqueue(start_pos, 0); // Enqueue the start position with a priority of 0
        came_from[start_pos] = Vector3.zero; // The start position has no parent
        cost_so_far[start_pos] = 0; // The start position has a cost of 0

        while (frontier.Count > 0)
        {
            Vector3 current = frontier.Dequeue();
            if (current == goal_pos)
            {
                break;
            }
            foreach (Vector3 next in graph.GetNeighbors(current))
            {
                float new_cost = cost_so_far[current] + Utils.CostToEnterTile(current, next);
                if (!cost_so_far.ContainsKey(next) || new_cost < cost_so_far[next])
                {
                    cost_so_far[next] = new_cost;
                    float priority = new_cost + Utils.Heuristic(goal_pos, next); // As the heuristic becomes smaller, A* turns into Dijkstra’s Algorithm. As the heuristic becomes larger, A* turns into Greedy Best First Search.
                    frontier.Enqueue(next, priority);
                    came_from[next] = current;
                }
            }
        }
        return came_from;
    }

    // A Star Search on a PRM graph
    public Dictionary<Vector3, Vector3> SearchOnPRMGraph(PRMGraph graph)
    {
        PriorityQueue<Vector3, float> frontier = new PriorityQueue<Vector3, float>();
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
            foreach (PRMNode next_node in graph.GetNeighbours(graph.GetNodeAt(current)))
            {
                Vector3 next = next_node.position;
                float new_cost = cost_so_far[current] + Utils.CostToEnterTile(current, next);
                if (!cost_so_far.ContainsKey(next) || new_cost < cost_so_far[next])
                {
                    cost_so_far[next] = new_cost;
                    float priority = new_cost + Utils.Heuristic(goal_pos, next); // As the heuristic becomes smaller, A* turns into Dijkstra’s Algorithm. As the heuristic becomes larger, A* turns into Greedy Best First Search.
                    frontier.Enqueue(next, priority);
                    came_from[next] = current;
                }
            }
        }
        return came_from;
    }

    // A Star Search on a Basic lazy PRM graph without precomputed costs of optimal paths
    public Dictionary<PRMNode, PRMNode> SearchOnPRMGraph(PRMGraph graph, PRMNode startNode, PRMNode goalNode)
    {
        PriorityQueue<PRMNode, float> frontier = new PriorityQueue<PRMNode, float>();
        Dictionary<PRMNode, PRMNode> came_from = new Dictionary<PRMNode, PRMNode>();
        Dictionary<PRMNode, float> cost_so_far = new Dictionary<PRMNode, float>();
        frontier.Enqueue(startNode, 0);
        came_from[startNode] = startNode;
        cost_so_far[startNode] = 0;

        while (frontier.Count > 0)
        {
            PRMNode current = frontier.Dequeue();
            if (current == goalNode)
            {
                break;
            }
            foreach (PRMNode next_node in graph.GetNeighbours(current))
            {
                float new_cost = cost_so_far[current] + Utils.CostToEnterTile(current.position, next_node.position);
                if (!cost_so_far.ContainsKey(next_node) || new_cost < cost_so_far[next_node])
                {
                    cost_so_far[next_node] = new_cost;
                    float priority = new_cost + Utils.Heuristic(goalNode.position, next_node.position); // As the heuristic becomes smaller, A* turns into Dijkstra’s Algorithm. As the heuristic becomes larger, A* turns into Greedy Best First Search.
                    frontier.Enqueue(next_node, priority);
                    came_from[next_node] = current;
                }
            }
        }
        return came_from;
    }


    // Dictionary to store precomputed costs of optimal paths to the goal node
    private Dictionary<PRMNode, float> optimalPathCostsToGoal;

    // Method to precompute costs of optimal paths to the goal node
    private void PrecomputeOptimalPathCostsToGoal(PRMGraph graph, PRMNode goalNode)
    {
        // Use Dijkstra's algorithm or A* search with admissible heuristic to compute costs
        // For simplicity, let's assume Dijkstra's algorithm here
        optimalPathCostsToGoal = Dijkstra(graph, goalNode);
    }

    // https://realitybytes.blog/2017/07/11/graph-based-path-planning-dijkstras-algorithm/
    private Dictionary<PRMNode, float> Dijkstra(PRMGraph graph, PRMNode goalNode)
    {
        // For each node, n, in the graph:
        foreach (PRMNode node in graph.GetNodes())
        {
            if (node == graph.GetRoot())
            {
                node.distance = 0;
            }
            else
            {
                node.distance = float.MaxValue;
            }
        }

        // Create an empty list and add the start node to it with distance 0:
        List<PRMNode> nodes = new List<PRMNode>();
        nodes.Add(graph.GetRoot());

        while (nodes.Count > 0)
        {
            // Sort the list by distance, with the smallest distance at the front:
            nodes.Sort((x, y) => x.distance.CompareTo(y.distance));
            PRMNode current = nodes[0];
            nodes.RemoveAt(0);

            // For each neighbor of the current node:
            foreach (PRMNode neighbor in graph.GetNeighbours(current))
            {
                // Calculate the distance to the neighbor through the current node:
                // neighbor.distance = current.distance + length of edge from neighbor to current
                float new_distance = current.distance + Utils.CostToEnterTile(current.position, neighbor.position);

                // If the new distance is less than the neighbor's current distance:
                if (neighbor.distance > new_distance)
                {
                    // Update the neighbor's distance:
                    neighbor.distance = new_distance;

                    // Add the neighbor to the list if it's not already there:
                    if (!nodes.Contains(neighbor))
                    {
                        nodes.Add(neighbor);
                    }
                }
            }
        }

        // return graph.GetNodes().ToDictionary(node => node, node => node.distance);
        Dictionary<PRMNode, float> optimalPathCostsToGoal = new Dictionary<PRMNode, float>();
        foreach (PRMNode node in nodes)
        {
            optimalPathCostsToGoal[node] = node.distance;
        }
        return optimalPathCostsToGoal;
    }

    // https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
    private Dictionary<PRMNode, float> Dijkstra2(PRMGraph graph, PRMNode goalNode)
    {
        // Create two lists: dist, prev, and Q:
        Dictionary<PRMNode, float> dist = new Dictionary<PRMNode, float>();
        Dictionary<PRMNode, PRMNode> prev = new Dictionary<PRMNode, PRMNode>();
        List<PRMNode> Q = new List<PRMNode>();
        foreach (PRMNode node in graph.GetNodes())
        {
            // Debug.Log("Graph node: " + node.position);
            dist[node] = Mathf.Infinity;
            prev[node] = null;
            Q.Add(node);
        }
        dist[graph.GetRoot()] = 0;
        // dist[goalNode] = float.MaxValue;

        while (Q.Count > 0)
        {
            // Find the node with the smallest distance in Q:
            PRMNode u = null;
            float min_dist = Mathf.Infinity;
            foreach (PRMNode node in Q)
            {
                if (dist[node] < min_dist)
                {
                    u = node;
                    min_dist = dist[node];
                }
            }
            Q.Remove(u);

            // For each neighbor of u:
            foreach (PRMNode v in graph.GetNeighbours(u))
            {
                // if (!dist.ContainsKey(v))
                // {
                //     dist[v] = Mathf.Infinity;
                // }

                float distance = Vector3.Distance(u.position, v.position);
                float totalDistance = dist[u] + distance;

                if (v.position == goalNode.position)
                {
                    // Debug.Log("v is goalNode");
                    if (totalDistance < dist[goalNode])
                    {
                        dist[goalNode] = totalDistance;
                        prev[goalNode] = u;
                    }
                }
                else
                {
                    // Debug.Log("v is not goalNode");
                    // if (!dist.ContainsKey(v))
                    // {
                    //     Debug.Log("goalNode: " + goalNode.position);
                    //     Debug.Log("v: " + v.position);
                    // }
                    if (totalDistance < dist[v])
                    {
                        dist[v] = totalDistance;
                        prev[v] = u;
                    }
                }
                // Debug.Log("u: " + u.position);
                // if (!dist.ContainsKey(v))
                // {
                //     Debug.Log("v: " + v.position);
                // }
                // if (dist.ContainsKey(goalNode))
                // {
                //     Debug.Log("goalNode is in dist!");
                // }
                // else
                // {
                //     Debug.Log("goalNode not found in dist");
                // }

                // float distance = Utils.CostToEnterTile(u.position, v.position);
                // float totalDistance = dist[u] + distance;
                // if (totalDistance < dist[v])
                // {
                //     dist[v] = totalDistance;
                //     prev[v] = u;
                // }

                // float alt = dist[u] + Utils.CostToEnterTile(u.position, v.position);
                // if (alt < dist[v])
                // {
                //     dist[v] = alt;
                //     prev[v] = u;
                // }
            }
        }

        // construct the path from the start node to the goal node using dist and prev:
        Dictionary<PRMNode, float> optimalPathCostsToGoal = new Dictionary<PRMNode, float>();
        PRMNode u2 = goalNode;
        if (prev[u2] != null || u2 == graph.GetRoot())
        {
            while (u2 != null)
            {
                optimalPathCostsToGoal[u2] = dist[u2];
                u2 = prev[u2];
            }
        }
        // while (u2 != null)
        // {
        //     optimalPathCostsToGoal[u2] = dist[u2];
        //     u2 = prev[u2];
        // }
        return optimalPathCostsToGoal;
    }


    // A* Search with heuristic using precomputed costs of optimal paths
    // Another speed improvement is to use the costs of optimal paths to 𝑔 in the 
    // original PRM as a heuristic for A search (used in the Batch Informed Trees algorithm (BIT*) sampling-based planner).
    public Dictionary<PRMNode, PRMNode> SearchOnLazyBasicPRMWithAstarOptimalHeuristic(PRMGraph graph, PRMNode startNode, PRMNode goalNode)
    {
        PriorityQueue<PRMNode, float> frontier = new PriorityQueue<PRMNode, float>();
        Dictionary<PRMNode, PRMNode> came_from = new Dictionary<PRMNode, PRMNode>();
        Dictionary<PRMNode, float> cost_so_far = new Dictionary<PRMNode, float>();
        PrecomputeOptimalPathCostsToGoal(graph, goalNode); // Precompute optimal path costs to goal
        frontier.Enqueue(startNode, 0);
        came_from[startNode] = null;
        cost_so_far[startNode] = 0;

        while (frontier.Count > 0)
        {
            PRMNode current = frontier.Dequeue();
            if (current == goalNode)
            {
                break;
            }
            foreach (PRMNode next_node in graph.GetNeighbours(current))
            {
                float new_cost = cost_so_far[current] + Utils.CostToEnterTile(current.position, next_node.position);
                if (!cost_so_far.ContainsKey(next_node) || new_cost < cost_so_far[next_node])
                {
                    cost_so_far[next_node] = new_cost;
                    // Use default heuristic value for nodes not in optimalPathCostsToGoal
                    float heuristic = optimalPathCostsToGoal.ContainsKey(next_node) ? optimalPathCostsToGoal[next_node] : Utils.Heuristic(goalNode.position, next_node.position);
                    float priority = new_cost + heuristic; // Use heuristic
                    frontier.Enqueue(next_node, priority);
                    came_from[next_node] = current;
                }
            }
        }
        return came_from;
    }

    // A Star Search on an inflated obstacle C-Space map:
    public Dictionary<Vector3, Vector3> SearchOnInflatedObstacleCSpace()
    {
        PriorityQueue<Vector3, float> frontier = new PriorityQueue<Vector3, float>();
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
            foreach (Vector3 next in Utils.GetNeighborsForInflatedCspaceMAp(current))
            {
                float new_cost = cost_so_far[current] + Utils.CostToEnterTile(current, next);
                if (!cost_so_far.ContainsKey(next) || new_cost < cost_so_far[next])
                {
                    cost_so_far[next] = new_cost;
                    float priority = new_cost + Utils.Heuristic(goal_pos, next); // As the heuristic becomes smaller, A* turns into Dijkstra’s Algorithm. As the heuristic becomes larger, A* turns into Greedy Best First Search.
                    frontier.Enqueue(next, priority);
                    came_from[next] = current;
                }
            }
        }
        return came_from;
    }
}