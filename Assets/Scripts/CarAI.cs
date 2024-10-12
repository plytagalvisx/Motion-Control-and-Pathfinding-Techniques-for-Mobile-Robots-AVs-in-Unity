using UnityEngine.Assertions;
using Unity.Collections;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using System;
using UnityEditor;
using System.Linq;
using static StateData;
using UnityEngine.Profiling;

// Here we mostly implement Informed (Heuristic) Search Strategies.
namespace UnityStandardAssets.Vehicles.Car
{

    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        public GameObject[] carFriends;
        public GameObject[] obstacles;
        private CarController m_Car; // the car controller we want to use
        public GameObject terrain_manager_game_object;
        public TerrainInfo terrain;
        TerrainManager terrain_manager;
        private List<Vector3> myPathVG = new List<Vector3>();
        private Rigidbody myRigidBody;
        private int next = 1;
        public bool nextPoint = false;
        public bool isFinished = false;
        public Vector3 start_pos;
        public Vector3 goal_pos;
        public Vector3 ahead;
        public Vector3 ahead2;
        public Vector3 ahead3;
        public Vector3 obstacleCenter;
        List<PRMNode> myPRMpath;
        List<Vector3> myPath;
        public Vector3 avoidance_force;
        public int seed = 0;


        // (nr.1) RRT parameters:
        private RRTSearchAlgorithm rrt;
        private PositionSampler sampler;
        private Node<State> target;
        private int index;
        private List<Node<State>> rrt_path;
        public float nodeRadius = 1f;


        // Alternative (nr.2) RRT parameters:
        private Tree2 tree;
        private RRTStar rrtStar;
        private List<TreeNode> RRTpath;
        private List<Vector3> rrt_path_vec3;
        // private List<Vector3> rrt_path;
        // private Vector3 target;


        // PRM parameters:
        private PRMGraph prmgraph;
        private PRMPathPlanner prm;
        private bool pathFound = false;

        // RRT Connect parameters:
        private RRTConnectAlgorithm rrt_connect;
        private List<RRTConnectNode> rrt_connect_path;


        // private Vector3 getCenterOfCell(int row, int col) {
        //     float xStep = (terrain.x_high - terrain.x_low) / terrain.x_N;
        //     float zStep = (terrain.z_high - terrain.z_low) / terrain.z_N;
        //     return new Vector3(terrain.x_low + (row - 1 + 0.5f) * xStep, terrain.start_pos.y, terrain.z_low + (col - 1 + 0.5f) * zStep);
        // }

        // private List<Vector3> getValidCenters() {
        //     // float[,] traversabilityMatrix = terrain.GetPaddedTraversability();
        //     float[,] traversabilityMatrix = terrain.GetTraversability();
        //     int rows = traversabilityMatrix.GetLength(0);
        //     int columns = traversabilityMatrix.GetLength(1);
        //     List<Vector3> validCenters = new List<Vector3>();

        //     for (int i = 0; i < rows; i++) {
        //         for (int j = 0; j < columns; j++) {
        //             if (isTraversable(traversabilityMatrix, i, j)) { 
        //                 Vector3 center = getCenterOfCell(i, j);
        //                 Vector3 fakeCenter = new Vector3(terrain.get_x_pos(i), 0.0f, terrain.get_z_pos(j));
        //                 // Debug.Log("center.x: " + center.x + " center.z: " + center.z);
        //                 // Debug.Log("get_x_pos(i): " + fakeCenter.x + " get_z_pos(j): " + fakeCenter.z);
        //                 validCenters.Add(center);
        //             }
        //         }
        //     }
        //     return validCenters;
        // }

        public List<Vector3> GetVector3Path(List<TreeNode> path)
        {
            List<Vector3> vector3Path = new List<Vector3>();
            Debug.Log("GetVector3Path");
            Debug.Log("First position: " + path[0].position.ToString());
            foreach (TreeNode node in path)
            {
                vector3Path.Add(new Vector3(node.position.x, 0, node.position.y));
            }
            return vector3Path;
        }

        public List<Vector3> GetVector3Path(List<Node<State>> path)
        {
            List<Vector3> vector3Path = new List<Vector3>();
            foreach (Node<State> node in path)
            {
                vector3Path.Add(node.position);
            }
            return vector3Path;
        }

        // private void OnDrawGizmos() // Draws only Informed (Heuristic) Search Strategies
        // {
        // Draw SimpleBreadthFirstSearch from start to goal
        // HashSet<Vector3> reached = SimpleBreadthFirstSearch();
        // Gizmos.color = Color.red;
        // foreach (var wp in reached) {
        //     Gizmos.DrawCube(wp, new Vector3(0.8f, 0.8f, 0.8f));
        // }

        // Draw ImprovedBreadthFirstSearch
        // Vector3 current = goal_pos;
        // List<Vector3> path = new List<Vector3>();
        // Dictionary<Vector3, Vector3> came_from = ImprovedBreadthFirstSearch();
        // while(current != start_pos)
        // {
        //     path.Add(current);
        //     current = came_from[current];
        // }
        // path.Add(start_pos);
        // path.Reverse();
        // Gizmos.color = Color.red;
        // foreach (var wp in path) {
        //     Gizmos.DrawCube(wp, new Vector3(0.8f, 0.8f, 0.8f));
        // }

        // Draw DijkstrasAlgorithm
        // Vector3 current = goal_pos;
        // List<Vector3> path = new List<Vector3>();
        // Dictionary<Vector3, Vector3> came_from = DijkstrasAlgorithm();
        // while(current != start_pos)
        // {
        //     path.Add(current);
        //     current = came_from[current];
        // }
        // path.Add(start_pos);
        // path.Reverse();
        // Gizmos.color = Color.red;
        // foreach (var wp in path) {
        //     Gizmos.DrawCube(wp, new Vector3(0.8f, 0.8f, 0.8f));
        // }

        // Draw GreedyBestFirstSearch
        // Vector3 current = goal_pos;
        // List<Vector3> path = new List<Vector3>();
        // Dictionary<Vector3, Vector3> came_from = GreedyBestFirstSearch();
        // while(current != start_pos)
        // {
        //     path.Add(current);
        //     current = came_from[current];
        // }
        // path.Add(start_pos);
        // path.Reverse();
        // Gizmos.color = Color.red;
        // foreach (var wp in path) {
        //     Gizmos.DrawCube(wp, new Vector3(0.8f, 0.8f, 0.8f));
        // }

        // Draw AStarAlgorithm on a traversable grid map (terrain and not graph)
        // Vector3 current = goal_pos;
        // List<Vector3> path = new List<Vector3>();
        // Dictionary<Vector3, Vector3> came_from = AStarAlgorithm();
        // while (current != start_pos)
        // {
        //     path.Add(current);
        //     current = came_from[current];
        // }
        // path.Add(start_pos);
        // path.Reverse();
        // Gizmos.color = Color.red;
        // foreach (var wp in path)
        // {
        //     Gizmos.DrawCube(wp, new Vector3(0.8f, 0.8f, 0.8f));
        // }

        // Draw AStarSearch on a simple graph (not terrain?)
        // Graph<Vector3> graph = Utils.CreateSimpleGraph();
        // SimpleAStarAlgorithms aStar = new SimpleAStarAlgorithms(start_pos, goal_pos);
        // Dictionary<Vector3, Vector3> came_from = aStar.SearchOnSimpleGraph(graph);
        // Vector3 current = goal_pos;
        // List<Vector3> path = new List<Vector3>();
        // while (current != start_pos)
        // {
        //     path.Add(current);
        //     current = came_from[current];
        // }
        // path.Add(start_pos);
        // path.Reverse();
        // Gizmos.color = Color.red;
        // foreach (var wp in path)
        // {
        //     Gizmos.DrawCube(wp, new Vector3(0.8f, 0.8f, 0.8f));
        //     if (path.IndexOf(wp) != 0)
        //     {
        //         Gizmos.DrawLine(wp, path[path.IndexOf(wp) - 1]);
        //     }
        // }

        // Draw graph:
        // foreach (var wp in graph.edges.Keys)
        // {
        //     Gizmos.color = Color.red;
        //     Gizmos.DrawCube(wp, new Vector3(0.8f, 0.8f, 0.8f));
        //     List<Vector3> graphNeighbors = graph.Neighbors(wp);
        //     foreach (var neighbor in graphNeighbors)
        //     {
        //         Gizmos.color = Color.blue;
        //         // Gizmos.DrawCube(neighbor, new Vector3(0.8f, 0.8f, 0.8f));
        //         Gizmos.DrawLine(wp, neighbor);
        //     }
        //     // foreach (var neighbor in graph.edges[wp]) {
        //     //     Gizmos.color = Color.blue;
        //     //     Gizmos.DrawCube(neighbor, new Vector3(0.8f, 0.8f, 0.8f));
        //     //     Gizmos.DrawLine(wp, neighbor);
        //     // }
        // }

        // Draw BestFirstBidirectionalSearch (Not working)
        // Graph<Vector3> graph = CreateSimpleGraph();
        // Dictionary<Vector3, Vector3> came_from = BestFirstBidirectionalSearch(graph, start_pos, goal_pos);
        // Vector3 current = goal_pos;
        // List<Vector3> path = new List<Vector3>();
        // while (current != start_pos)
        // {
        //     path.Add(current);
        //     // KeyNotFoundException: The given key '(95.00, 0.00, 195.00)' was not present in the dictionary.
        //     // Solution:
        //     if (came_from.ContainsKey(current))
        //     {
        //         current = came_from[current];
        //     }
        //     else
        //     {
        //         break;
        //     }
        // }
        // path.Add(start_pos);
        // path.Reverse();
        // Gizmos.color = Color.red;
        // foreach (var wp in path)
        // {
        //     Gizmos.DrawCube(wp, new Vector3(0.8f, 0.8f, 0.8f));
        //     if (path.IndexOf(wp) != 0)
        //     {
        //         Gizmos.DrawLine(wp, path[path.IndexOf(wp) - 1]);
        //     }
        // }

        // Draw ahead vectors:
        // Gizmos.color = Color.blue;
        // Gizmos.DrawLine(transform.position, ahead);
        // Gizmos.color = Color.yellow;
        // Gizmos.DrawLine(transform.position, ahead2);
        // Gizmos.color = Color.green;
        // Gizmos.DrawLine(transform.position, avoidance_force);

        // // Draw Vector3 ahead and Vector3 ahead2 lines
        // Gizmos.color = Color.blue;
        // // Gizmos.DrawCube(ahead, new Vector3(0.8f, 0.8f, 0.8f));
        // Gizmos.DrawLine(transform.position, ahead);
        // Gizmos.color = Color.yellow;
        // // Gizmos.DrawCube(ahead2, new Vector3(0.8f, 0.8f, 0.8f));
        // Gizmos.DrawLine(transform.position, ahead2);
        // Gizmos.DrawLine(transform.position, ahead3);


        // Draw one point
        // Gizmos.color = Color.green;
        // Gizmos.DrawCube(start_pos, new Vector3(0.8f, 0.8f, 0.8f));


        // Draw Graph2:
        // if (allowedDisplayGraph2)
        // {
        //     Graph2 graph = ConstructGraph();
        //     foreach (Node2 node in graph.GetNodes())
        //     {
        //         Vector3 node_position = new Vector3(node.center.x, 0.0f, node.center.y);
        //         Gizmos.color = Color.red;
        //         Gizmos.DrawCube(node_position, new Vector3(0.8f, 0.8f, 0.8f));
        //         foreach (Node2 neighbour in node.GetNeighbours())
        //         {
        //             Gizmos.color = Color.blue;
        //             Vector3 neighbour_position = new Vector3(neighbour.center.x, 0.0f, neighbour.center.y);
        //             Gizmos.DrawLine(node_position, neighbour_position);
        //         }
        //     }

        //     // Is computationally expensive to draw the graph (very slow):
        //     // Vector2 grid_size = new Vector2(2f, 2f);
        //     // Graph2 graph2 = constructFineGraph(grid_size);
        //     // foreach (Node2 node in graph2.GetNodes())
        //     // {
        //     //     Vector3 node_position = new Vector3(node.center.x, 0.0f, node.center.y);
        //     //     Gizmos.color = Color.green;
        //     //     Gizmos.DrawCube(node_position, new Vector3(0.8f, 0.8f, 0.8f));
        //     //     foreach (Node2 neighbour in node.GetNeighbours())
        //     //     {
        //     //         Gizmos.color = Color.blue;
        //     //         Vector3 neighbour_position = new Vector3(neighbour.center.x, 0.0f, neighbour.center.y);
        //     //         Gizmos.DrawLine(node_position, neighbour_position);
        //     //     }
        //     // }
        // }

        // Draw RRT and RRT* (that uses Vector3 instead of Vector2):
        // if (rrt_path != null)
        // {
        //     // Debug.Log("pathFound = " + rrt.pathFound);
        //     // Debug.Log("Tree = " + rrt.tree);

        //     if (rrt.pathFound)
        //     {
        //         if (rrt.tree != null)
        //         {
        //             DrawTree();
        //             DrawPath(rrt_path, Color.blue);
        //         }
        //     }
        // }

        // Draw RRT*:
        // if (drawrrttree && rrt_path != null)
        // {
        //     foreach (Node<State> node in rrt_path)
        //     {
        //         Vector3 node_position = new Vector3(node.position.x, 0.0f, node.position.y);
        //         Gizmos.color = Color.red;
        //         Gizmos.DrawCube(node_position, new Vector3(0.8f, 0.8f, 0.8f));
        //         foreach (Node<State> neighbour in node.children)
        //         {
        //             Gizmos.color = Color.blue;
        //             Vector3 neighbour_position = new Vector3(neighbour.position.x, 0.0f, neighbour.position.y);
        //             Gizmos.DrawLine(node_position, neighbour_position);
        //         }
        //     }
        // }

        // Draw PRM:
        // if (drawprmgraph && drawmypath && prmgraph != null)
        // {
        //     foreach (PRMNode node in prmgraph.GetNodes())
        //     {
        //         Vector3 node_position = new Vector3(node.position.x, 0.0f, node.position.z);
        //         Gizmos.color = Color.red;
        //         Gizmos.DrawCube(node_position, new Vector3(0.8f, 0.8f, 0.8f));
        //         foreach (PRMNode neighbour in prmgraph.GetNeighbours(node))
        //         {
        //             Gizmos.color = Color.blue;
        //             Vector3 neighbour_position = new Vector3(neighbour.position.x, 0.0f, neighbour.position.z);
        //             Gizmos.DrawLine(node_position, neighbour_position);
        //         }
        //     }

        //     // Draw Lazy Basic PRM path, make the path thicker
        //     Vector3 old_wp = start_pos;
        //     foreach (var wp in myPRMpath)
        //     {
        //         Gizmos.color = Color.red;
        //         Gizmos.DrawCube(wp.position, new Vector3(0.8f, 0.8f, 0.8f));
        //         Gizmos.DrawLine(old_wp, wp.position);
        //         old_wp = wp.position;

        //         // Debug.DrawLine(old_wp, wp, Color.red, 100f);
        //         // old_wp = wp;
        //     }
        // }


        // if (drawprmgraph && drawmypath && prmgraph != null)
        // {
        //     foreach (PRMNode node in prmgraph.GetNodes())
        //     {
        //         Vector3 node_position = new Vector3(node.position.x, 0.0f, node.position.z);
        //         Gizmos.color = Color.red;
        //         Gizmos.DrawCube(node_position, new Vector3(0.8f, 0.8f, 0.8f));
        //         foreach (PRMNode neighbour in prmgraph.GetNeighbours(node))
        //         {
        //             Gizmos.color = Color.blue;
        //             Vector3 neighbour_position = new Vector3(neighbour.position.x, 0.0f, neighbour.position.z);
        //             Gizmos.DrawLine(node_position, neighbour_position);
        //         }
        //     }

        //     if (myPRMpath != null)
        //     {
        //         // Draw Lazy Basic PRM path, make the path thicker
        //         Vector3 old_wp = start_pos;
        //         foreach (var wp in myPRMpath)
        //         {
        //             Gizmos.color = Color.red;
        //             Gizmos.DrawCube(wp.position, new Vector3(0.8f, 0.8f, 0.8f));
        //             Gizmos.DrawLine(old_wp, wp.position);
        //             old_wp = wp.position;

        //             // Debug.DrawLine(old_wp, wp, Color.red, 100f);
        //             // old_wp = wp;
        //         }
        //     }
        // }


        // if (voronoiReady)
        // {
        //     int numberOfPoints = 150;
        //     // Construct Voronoi Diagram:
        //     List<Vector3> randomSites = new List<Vector3>();

        //     //Generate random numbers with a seed
        //     UnityEngine.Random.InitState(seed);

        //     float max = terrain_manager.myInfo.x_high;
        //     float min = terrain_manager.myInfo.x_low;

        //     for (int i = 0; i < numberOfPoints; i++)
        //     {
        //         float randomX = UnityEngine.Random.Range(min, max);
        //         float randomZ = UnityEngine.Random.Range(min, max);

        //         randomSites.Add(new Vector3(randomX, 0f, randomZ));
        //     }

        //     sampler = new PositionSampler(terrain_manager.myInfo);
        //     // randomSites = sampler.SamplePositions(numberOfPoints);


        //     //Points outside of the screen for voronoi which has some cells that are infinite
        //     float bigSize = terrain_manager.myInfo.x_high * 5f;

        //     //Star shape which will give a better result when a cell is infinite large
        //     //When using other shapes, some of the infinite cells misses triangles
        //     randomSites.Add(new Vector3(0f, 0f, bigSize));
        //     randomSites.Add(new Vector3(0f, 0f, -bigSize));
        //     randomSites.Add(new Vector3(bigSize, 0f, 0f));
        //     randomSites.Add(new Vector3(-bigSize, 0f, 0f));


        //     GeneralizedVoronoiDiagram gvd = new GeneralizedVoronoiDiagram(terrain_manager.myInfo, sampler);
        //     List<VoronoiCell> cells = gvd.DelaunayToVoronoi(randomSites);



        //     //Generate the voronoi diagram
        //     // List<VoronoiCell> cells = GenerateVoronoiDiagram(randomSites);

        //     //Debug
        //     //Display the voronoi diagram
        //     // DisplayVoronoiCells(cells);

        //     //Display the sites
        //     Gizmos.color = Color.red;

        //     for (int i = 0; i < randomSites.Count; i++)
        //     {
        //         float radius = 0.2f;

        //         Gizmos.DrawSphere(randomSites[i], radius);
        //     }

        //     // draw voronoi edges:
        //     foreach (VoronoiCell cell in cells)
        //     {
        //         foreach (VoronoiEdge edge in cell.edges)
        //         {
        //             Gizmos.color = Color.blue;
        //             Gizmos.DrawLine(edge.v1, edge.v2);
        //         }
        //     }
        // }
        // }

        //Display the voronoi diagram with mesh
        // private void DisplayVoronoiCells(List<VoronoiCell> cells)
        // {
        //     UnityEngine.Random.InitState(seed);

        //     for (int i = 0; i < cells.Count; i++)
        //     {
        //         VoronoiCell c = cells[i];

        //         Vector3 p1 = c.sitePos;
        //         // p1.y = 10f;

        //         Gizmos.color = new Color(UnityEngine.Random.Range(0f, 1f), UnityEngine.Random.Range(0f, 1f), UnityEngine.Random.Range(0f, 1f), 1f);

        //         List<Vector3> vertices = new List<Vector3>();

        //         List<int> triangles = new List<int>();

        //         vertices.Add(p1);

        //         for (int j = 0; j < c.edges.Count; j++)
        //         {
        //             Vector3 p3 = c.edges[j].v1;
        //             // p3.y = 10f;
        //             Vector3 p2 = c.edges[j].v2;
        //             // p2.y = 10f;

        //             vertices.Add(p2);
        //             vertices.Add(p3);

        //             triangles.Add(0);
        //             triangles.Add(vertices.Count - 2);
        //             triangles.Add(vertices.Count - 1);
        //         }

        //         Mesh triangleMesh = new Mesh();
        //         triangleMesh.vertices = vertices.ToArray();
        //         triangleMesh.triangles = triangles.ToArray();
        //         triangleMesh.RecalculateNormals();
        //         Gizmos.DrawMesh(triangleMesh);
        //         // make this mesh visible in the scene view:
        //         // Gizmos.DrawWireMesh(triangleMesh);
        //     }
        // }



        private void DrawTree()
        {
            DrawTreeRecursive(rrt.tree.root);
        }

        private void DrawTreeRecursive(Node<State> node)
        {
            DrawNodeWithGizmos(node, Color.green);
            foreach (Node<State> child in node.children)
            {
                DrawPathBetweenNodes(node, child, Color.red);
                DrawTreeRecursive(child);
            }
        }

        private void DrawNodeWithGizmos(Node<State> node, Color color)
        {
            Gizmos.color = color;
            Vector3 pos = node.position;
            pos.y = 0.5f;
            Gizmos.DrawSphere(pos, nodeRadius);

            // Draw the node's position
            //GUIStyle style = new GUIStyle();
            //style.normal.textColor = Color.white;
            //UnityEditor.Handles.Label(pos, node.position.ToString(), style);

        }

        private void DrawPathBetweenNodes(Node<State> nodeA, Node<State> nodeB, Color color)
        {
            Gizmos.color = color;
            Vector3 posA = nodeA.position;
            Vector3 posB = nodeB.position;
            posA.y = 0.5f;
            posB.y = 0.5f;
            Gizmos.DrawLine(posA, posB);
        }

        private void DrawPath(List<Node<State>> path, Color color)
        {
            if (path == null)
            {
                return;
            }
            Node<State> prevNode = null;
            foreach (Node<State> node in path)
            {
                if (prevNode != null)
                {
                    DrawPathBetweenNodes(prevNode, node, color);
                }
                DrawNodeWithGizmos(node, color);
                prevNode = node;
            }
        }

        public Graph2 ConstructGraph()
        {
            Graph2 graph = new Graph2();

            // Add all nodes to the graph
            float[,] traversability = terrain.traversability;
            for (int i = 0; i < traversability.GetLength(0); i++)
            {
                for (int j = 0; j < traversability.GetLength(1); j++)
                {
                    if (traversability[i, j] == 0)
                    {
                        Vector2 node_pos = new Vector2(i, j);
                        Vector2 node_center = new Vector2(terrain.get_x_pos(i), terrain.get_z_pos(j));
                        Vector2 node_size = new Vector2(terrain.get_x_step(), terrain.get_z_step());

                        Node2 node = new Node2(node_pos, node_center, node_size);
                        graph.AddNode(node);
                    }
                }
            }

            // Add all neighbours to the nodes
            // Loop through all nodes
            foreach (Node2 node in graph.GetNodes())
            {
                // Loop through all nodes again
                foreach (Node2 otherNode in graph.GetNodes())
                {
                    // Check if nodes are next to each other
                    if (graph.GetDistance(node, otherNode, 1) == 1)
                    {
                        node.AddNeighbour(otherNode);
                    }
                }
            }

            return graph;
        }

        private Graph2 constructFineGraph(Vector2 node_size)
        {
            Graph2 graph = new Graph2();

            // Add all nodes to the graph
            float[,] traversability = terrain.traversability;
            float x_low = terrain.x_low;
            float x_high = terrain.x_high;
            float z_low = terrain.z_low;
            float z_high = terrain.z_high;

            int i = 0;
            for (float x = x_low; x + node_size.x / 2f < x_high; x += node_size.x)
            {
                int j = 0;
                for (float z = z_low; z + node_size.y / 2f < z_high; z += node_size.y)
                {
                    Vector2 node_pos = new Vector2(i, j);
                    Vector2 node_center = new Vector2(x + node_size.x / 2, z + node_size.y / 2);
                    Node2 node = new Node2(node_pos, node_center, node_size);
                    if (terrain.is_area_clear(node.GetVertices()))
                    {
                        graph.AddNode(node);
                    }

                    j += 1;
                }
                i += 1;
            }
            // Add all neighbours to the nodes
            // Loop through all nodes
            foreach (Node2 node in graph.GetNodes())
            {
                // Loop through all nodes again
                foreach (Node2 otherNode in graph.GetNodes())
                {
                    // Check if nodes are next to each other
                    if (graph.GetDistance(node, otherNode, 2) < 2.5f)
                    {
                        node.AddNeighbour(otherNode);
                    }
                }
            }

            return graph;
        }

        // public void obstaclesInformation()
        // {
        //     float height = 10f;
        //     float width = 10f;

        //     // What's the angle since it's a square:
        //     float angle = 90f;
        //     //corner points:

        // }


        // public List<Vector3> GetObstaclePolygons()
        // {
        //     float[,] traversabilityMatrix = terrain_manager.myInfo.GetPaddedTraversability();
        //     int rows = traversabilityMatrix.GetLength(0);
        //     int columns = traversabilityMatrix.GetLength(1);
        //     bool[,] visited = new bool[rows, columns];
        //     float xStep = (terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low) / terrain_manager.myInfo.x_N;
        //     float zStep = (terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low) / terrain_manager.myInfo.z_N;


        //     List<List<Vector3>> polygons = new List<List<Vector3>>();

        //     for (int r = 0; r < rows; r++)
        //     {
        //         for (int c = 0; c < columns; c++)
        //         {
        //             if (!visited[r, c] && !IsTraversable(traversabilityMatrix, r, c))
        //             {
        //                 var component = GetConnectedComponent(r, c, rows, columns, visited, xStep, zStep, terrain_manager, traversabilityMatrix);
        //                 var polygon = ConstructPolygon(component);
        //                 // Display component:
        //                 foreach (var pos in component)
        //                 {
        //                     Debug.DrawLine(pos, pos + Vector3.up * 10, Color.red, 1000f);
        //                 }
        //                 polygons.Add(polygon);
        //             }
        //         }
        //     }

        //     return polygons;
        // }

        // private List<Vector3> GetConnectedComponent(int startRow, int startCol, int rows, int columns, bool[,] visited, float xStep, float zStep, TerrainManager terrain_manager, float[,] traversabilityMatrix)
        // {
        //     List<Vector3> component = new List<Vector3>();
        //     Queue<Vector2Int> queue = new Queue<Vector2Int>();
        //     queue.Enqueue(new Vector2Int(startRow, startCol));
        //     visited[startRow, startCol] = true;

        //     while (queue.Count > 0)
        //     {
        //         var cell = queue.Dequeue();
        //         int r = cell.x;
        //         int c = cell.y;
        //         component.Add(GetCenterOfCell(r, c, xStep, zStep, terrain_manager));

        //         foreach (var dir in new Vector2Int[] { new Vector2Int(0, 1), new Vector2Int(1, 0), new Vector2Int(0, -1), new Vector2Int(-1, 0) })
        //         {
        //             int nr = r + dir.x, nc = c + dir.y;
        //             if (nr >= 0 && nr < rows && nc >= 0 && nc < columns && !visited[nr, nc] && !IsTraversable(traversabilityMatrix, nr, nc))
        //             {
        //                 queue.Enqueue(new Vector2Int(nr, nc));
        //                 visited[nr, nc] = true;
        //             }
        //         }
        //     }

        //     return component;
        // }

        // private Vector3 CalculateCentroid(List<Vector3> component)
        // {
        //     Vector3 sum = Vector3.zero;
        //     foreach (var pos in component)
        //     {
        //         sum += pos;
        //     }
        //     return sum / component.Count;
        // }


        public List<Vector3> GetObstacleCornersOrCentroids(string query)
        {
            float[,] paddedTraversabilityMatrix = terrain_manager.myInfo.GetPaddedTraversability();
            float xStep = (terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low) / terrain_manager.myInfo.x_N;
            float zStep = (terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low) / terrain_manager.myInfo.z_N;
            int rows = paddedTraversabilityMatrix.GetLength(0);
            int columns = paddedTraversabilityMatrix.GetLength(1);
            List<Vector3> obstacleCorners = new List<Vector3>();
            List<Vector3> obstacleCentroids = new List<Vector3>();

            int[] cornerSteps = new int[] { -1, -1, 1, 1, -1 };
            int[] adjacentSteps = new int[] { 0, -1, 0, 1, 0, -1 };
            for (int r = 1; r < rows - 1; r++)
            {
                for (int c = 1; c < columns - 1; c++)
                {
                    if (IsTraversable(paddedTraversabilityMatrix, r, c) == true) continue;
                    for (int i = 0; i < 4; i++)
                    {
                        // For each corner of the cell
                        if (IsTraversable(paddedTraversabilityMatrix, r + cornerSteps[i], c + cornerSteps[i + 1]) == false) continue;
                        Cell c1 = new Cell(r + adjacentSteps[i], c + adjacentSteps[i + 1]);
                        Cell c2 = new Cell(r + adjacentSteps[i + 1], c + adjacentSteps[i + 2]); // cells are adjacent obstacle corner points
                        if (AreAdjacentCellsTraversable(paddedTraversabilityMatrix, c1, c2) == true)
                        {
                            Vector3 center = GetCenterOfCell(r, c, xStep, zStep, terrain_manager);
                            Vector3 corner = GetCornerOfCell(center, cornerSteps, i, xStep, zStep);
                            obstacleCorners.Add(corner);
                            obstacleCentroids.Add(center);
                        }
                    }
                }
            }
            if (query == "corners")
            {
                return obstacleCorners;
            }
            else if (query == "centroids")
            {
                return obstacleCentroids;
            }
            else
            {
                return null;
            }
        }

        private bool IsTraversable(float[,] traversability, int row, int col)
        {
            return traversability[row, col] == 0.0f;
        }

        // private bool AreAdjacentCellsTraversable(float[,] traversability, Cell c1, Cell c2)
        // {
        //     // Add logic to determine if two adjacent cells are traversable
        //     return IsTraversable(traversability, c1.row, c1.col) && IsTraversable(traversability, c2.row, c2.col);
        // }

        // private Vector3 GetCenterOfCell(int row, int col, float xStep, float zStep, TerrainManager terrain_manager)
        // {
        //     // Logic to determine the center of a cell
        //     float x = terrain_manager.myInfo.x_low + (row - 1 + 0.5f) * xStep;
        //     float z = terrain_manager.myInfo.z_low + (col - 1 + 0.5f) * zStep;
        //     return new Vector3(x, 0, z); // Assuming Y is constant 0
        // }

        // private Vector3 GetCornerOfCell(Vector3 center, int[] cornerOffsets, int index, float xStep, float zStep)
        // {
        //     // Logic to determine the corner of a cell
        //     float x = center.x + cornerOffsets[index] * xStep / 2;
        //     float z = center.z + cornerOffsets[index + 1] * zStep / 2;
        //     return new Vector3(x, 0, z);
        // }

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
        private Vector3 GetCornerOfCell(Vector3 center, int[] cornerSteps, int cornerIndex, float xStep, float zStep)
        {
            float margin = 0f;
            float x = center.x + cornerSteps[cornerIndex] * (xStep / 2 + margin / Mathf.Sqrt(2));
            float z = center.z + cornerSteps[cornerIndex + 1] * (zStep / 2 + margin / Mathf.Sqrt(2));
            return new Vector3(x, 0, z);
        }


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

        // Maybe try to implement this?: https://courses.cs.washington.edu/courses/cse326/00wi/projects/voronoi.html
        // public List<VoronoiCell> GenerateVoronoiDiagram(List<Vector3> sites)
        // {
        //     int width = terrain_manager.myInfo.x_N;
        //     float max = terrain_manager.myInfo.x_high;
        //     float min = terrain_manager.myInfo.x_low;
        // }




        private Vector3 startPos;
        private Vector3 goalPos;
        private Node2 startNode;
        private Node2 goalNode;
        public bool allowedDisplayGraph2 = false;
        public bool drawprmgraph = false;
        public bool drawrrttree = false;
        public bool drawmypath = false;
        public bool callAgain = false;
        public bool voronoiReady = false;

        public void OnDrawGizmos()
        {
            if (terrain == null) return;
            if (!terrain.drawObstacleVertices) return;

            foreach (PolygonalObstacle obstacle in terrain.polygonalObstacles)
            {
                foreach (Vector3 vertex in obstacle.vertices)
                {
                    Gizmos.color = Color.blue;
                    Gizmos.DrawSphere(vertex, 0.5f);
                }
            }

            foreach (Vector3 vertex in terrain.boundaryPoints)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawSphere(vertex, 0.5f);
            }
        }

        private void Start()
        {
            // Get the car controller
            m_Car = GetComponent<CarController>();
            myRigidBody = GetComponent<Rigidbody>(); // Get the rigidbody of the car. This is used to get the car's position and orientation.
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            terrain = terrain_manager.myInfo;
            Utils utils = new Utils(terrain_manager.myInfo);
            carFriends = GameObject.FindGameObjectsWithTag("Player");
            // obstacles = GameObject.FindGameObjectsWithTag("Box");

            voronoiReady = true;

            // Plan your path here:            
            // start_pos = new Vector3(terrain.start_pos.x, 0.0f, terrain.start_pos.z);
            // goal_pos = new Vector3(terrain.goal_pos.x, 0.0f, terrain.goal_pos.z);

            // Use these for A* algorithms you defined in SimpleAStarAlgorithms.cs:
            start_pos = new Vector3(terrain.get_x_pos(terrain.get_i_index(terrain.start_pos.x)), 0.0f, terrain.get_z_pos(terrain.get_j_index(terrain.start_pos.z)));
            goal_pos = new Vector3(terrain.get_x_pos(terrain.get_i_index(terrain.goal_pos.x)), 0.0f, terrain.get_z_pos(terrain.get_j_index(terrain.goal_pos.z)));

            // GameObject[] obstacles = GameObject.FindGameObjectsWithTag("Box");
            // foreach (GameObject obstacle in obstacles)
            // {
            //     BoxCollider boxCollider = obstacle.GetComponent<BoxCollider>();
            //     SphereCollider sphereCollider = obstacle.AddComponent<SphereCollider>();
            //     sphereCollider.center = boxCollider.center;
            //     sphereCollider.radius = 0.5f;
            //     sphereCollider.enabled = false;
            // }

            // _______________________________________________________________________________________________________________________________________________________________________________________

            // A* Algorithm on a Simple/Basic constructed Graph:
            // Graph<Vector3> graph = Utils.CreateSimpleGraph();
            // SimpleAStarAlgorithms aStar = new SimpleAStarAlgorithms(start_pos, goal_pos);
            // Dictionary<Vector3, Vector3> came_from = aStar.SearchOnSimpleGraph(graph);
            // Vector3 current = goal_pos;
            // myPath = new List<Vector3>();
            // while (current != start_pos)
            // {
            //     myPath.Add(current);
            //     current = came_from[current];
            // }
            // myPath.Add(start_pos);
            // myPath.Reverse();

            // // Plot your path to see if it makes sense with cubes
            // Vector3 old_wp = start_pos;
            // foreach (var wp in myPath)
            // {
            //     Debug.DrawLine(old_wp, wp, Color.red, 100f);
            //     old_wp = wp;
            // }

            // _______________________________________________________________________________________________________________________________________________________________________________________

            // A* Algorithm on a Traversable constructed Grid Map:
            // SimpleAStarAlgorithms aStar = new SimpleAStarAlgorithms(start_pos, goal_pos);
            // Dictionary<Vector3, Vector3> came_from = aStar.SearchOnGridMap();
            // Vector3 current = goal_pos;
            // myPath = new List<Vector3>();
            // while (current != start_pos)
            // {
            //     myPath.Add(current);
            //     current = came_from[current];
            // }
            // myPath.Add(start_pos);
            // myPath.Reverse();

            // // Plot your path to see if it makes sense with cubes
            // Vector3 old_wp = start_pos;
            // foreach (var wp in myPath)
            // {
            //     Debug.DrawLine(old_wp, wp, Color.red, 100f);
            //     old_wp = wp;
            // }

            // _______________________________________________________________________________________________________________________________________________________________________________________

            // A* Algorithm on an Inflated Obstacle C-Space:
            // SimpleAStarAlgorithms aStar = new SimpleAStarAlgorithms(start_pos, goal_pos);
            // Dictionary<Vector3, Vector3> came_from = aStar.SearchOnInflatedObstacleCSpace();
            // Vector3 current = goal_pos;
            // myPath = new List<Vector3>();
            // while (current != start_pos)
            // {
            //     myPath.Add(current);
            //     current = came_from[current];
            // }
            // myPath.Add(start_pos);
            // myPath.Reverse();

            // // Plot your path to see if it makes sense with cubes
            // Vector3 old_wp = start_pos;
            // foreach (var wp in myPath)
            // {
            //     Debug.DrawLine(old_wp, wp, Color.red, 100f);
            //     old_wp = wp;
            // }

            // _______________________________________________________________________________________________________________________________________________________________________________________

            // A* Algorithm on a constructed Visibility Graph:
            // VisibilityGraph visibilityGraph = terrain_manager.GetComponent<VisibilityGraph>();
            // myPath = visibilityGraph.GetShortestPlannedPathWayPoints(); // Get the shortest path
            // myPath = visibilityGraph.GetAugmentedPath(myPath); // Get the augmented path for better smoothing later

            // // Plot your path to see if it makes sense with cubes
            // Vector3 old_wp = start_pos;
            // foreach (var wp in myPath)
            // {
            //     Debug.DrawLine(old_wp, wp, Color.red, 100f);
            //     old_wp = wp;
            // }

            // myPath = visibilityGraph.getSmoothedPath(myPath);
            // // Draw the smoothed path
            // old_wp = start_pos;
            // foreach (var wp in myPath)
            // {
            //     Debug.DrawLine(old_wp, wp, Color.green, 100f);
            //     old_wp = wp;
            // }

            // _______________________________________________________________________________________________________________________________________________________________________________________

            // Hybrid A* Algorithm:
            // HybridAStar hybrid_astar = new HybridAStar(terrain_manager.myInfo);
            // hybrid_astar.initializer();
            // myPath = hybrid_astar.GetPath();

            // Vector3 old_wp = myPath[0];
            // foreach (var wp in myPath)
            // {
            //     Debug.DrawLine(old_wp, wp, Color.red, 100f);
            //     old_wp = wp;
            // }

            // _______________________________________________________________________________________________________________________________________________________________________________________

            // RRT or RRT* Search Algorithm on a constructed Tree that appends Node States containing (position and orientation) of the car:
            // Vector3 starPos = terrain.start_pos;
            // Vector3 goalPos = terrain.goal_pos;
            // Vector3 startOrientation = transform.TransformDirection(Vector3.forward); // this is the start direction/orientation of the car pointing forward

            // Node<State> rootNode = new Node<State>(starPos, new State(starPos, startOrientation));
            // Node<State> goalNode = new Node<State>(goalPos, new State(goalPos, Vector3.zero));

            // Tree<State> tree = new Tree<State>(rootNode, goalNode);
            // // How to sample points for RRT:
            // sampler = new PositionSampler(terrain);

            // rrt = new RRTSearchAlgorithm(
            //     tree,
            //     rootNode,
            //     goalNode,
            //     sampler
            // );
            // rrt_path = rrt.FindPath();

            // // Plot the found rrt path in blue
            // Vector3 old_wp = starPos;
            // foreach (var wp in rrt_path)
            // {
            //     Debug.DrawLine(old_wp, wp.position, Color.blue, 100f);
            //     old_wp = wp.position;
            // }

            // index = 0; // can be 0 or 1, it don't matter
            // target = rrt_path[index];
            // Debug.Log($"Target pos: {target.position}");

            // drawrrttree = true;

            // _______________________________________________________________________________________________________________________________________________________________________________________

            // Alternative RRT Search Algorithm implementation along with RRT* Search Algorithm:
            // Vector2 startNodePos = new Vector2(terrain.start_pos.x, terrain.start_pos.z);
            // Vector2 goalNodePos = new Vector2(terrain.goal_pos.x, terrain.goal_pos.z);
            // Vector3 direction = transform.TransformDirection(Vector3.forward);

            // TreeNode startNodeRRT = new TreeNode(startNodePos);
            // TreeNode goalNodeRRT = new TreeNode(goalNodePos);

            // tree = new Tree2(startNodeRRT, goalNodeRRT);
            // rrtStar = new RRTStar(tree, direction, startNodeRRT, goalNodeRRT, terrain_manager);

            // // TWO different approaches for obtaining the RRT or RRT* path:
            // // 1. Computes a path and returns it here
            // // 2. Computes a path and stores it in a Tree class object, which is then accessed here through GetPath() method

            // // 1. Computes a path and returns it here:
            // RRTpath = rrtStar.RRTStarIterationReturnPath();
            // rrt_path = GetVector3Path(RRTpath);

            // // 2. Computes a path and stores it in a Tree class object, which is then accessed here through GetPath() method:
            // // while (tree.GetPath() == null)
            // // {
            // //     rrtStar.RRTStarIterationUpdateTree();
            // // }
            // // RRTpath = tree.GetPath();
            // // rrt_path = GetVector3Path(RRTpath);


            // Debug.Log("Calculated path");
            // // Plot your path to see if it makes sense
            // // Note that path can only be seen in "Scene" window, not "Game" window
            // Vector3 old_wp = rrt_path[0];
            // foreach (var wp in rrt_path)
            // {
            //     Debug.DrawLine(old_wp, wp, Color.red, 100f);
            //     old_wp = wp;
            // }

            // index = 0;
            // target = rrt_path[index];
            // Debug.Log($"Target pos: {target}");

            // _______________________________________________________________________________________________________________________________________________________________________________________

            // Constructing a Graph for alternative RRT Search Algorithm implementation (checking):
            // allowedDisplayGraph2 = true;

            // startPos = terrain.start_pos;
            // goalPos = terrain.goal_pos;
            // Graph2 graph = ConstructGraph();
            // // Vector2 grid_size = new Vector2(2f, 2f);

            // foreach (Node2 node in graph.GetNodes())
            // {
            //     node.LogPosition();
            // }

            // graph.LogGraph();

            // startNode = graph.FindNodeIter(new Vector2(startPos.x, startPos.z));
            // goalNode = graph.FindNodeIter(new Vector2(goalPos.x, goalPos.z));
            // Debug.Log(string.Format("startNode: ({0}, {1})", startNode.position.x, startNode.position.y));

            // _______________________________________________________________________________________________________________________________________________________________________________________

            // Probabilistic Roadmap Basic (PRM) Algorithm:
            // Works fine but need some post-processing (smoothing) the path to avoid sharp turns.
            // start_pos = new Vector3(terrain.start_pos.x, 0.0f, terrain.start_pos.z);
            // goal_pos = new Vector3(terrain.goal_pos.x, 0.0f, terrain.goal_pos.z);
            // PRMNode startnode = new PRMNode(start_pos);
            // PRMNode goalnode = new PRMNode(goal_pos);
            // prmgraph = new PRMGraph(startnode, goalnode);
            // sampler = new PositionSampler(terrain);

            // prm = new PRMPathPlanner(terrain, prmgraph, startnode, goalnode, sampler);
            // prm.GenerateBasicPRMalgorithm();

            // drawprmgraph = true;

            // // A* Algorithm on a PRM Graph:
            // SimpleAStarAlgorithms aStar = new SimpleAStarAlgorithms(start_pos, goal_pos);
            // Dictionary<Vector3, Vector3> came_from = aStar.SearchOnPRMGraph(prmgraph);
            // Vector3 current = goal_pos;
            // myPath = new List<Vector3>();
            // while (current != start_pos)
            // {
            //     myPath.Add(current);
            //     current = came_from[current];
            // }
            // myPath.Add(start_pos);
            // myPath.Reverse();

            // drawmypath = true;

            // _______________________________________________________________________________________________________________________________________________________________________________________

            // Improved PRM: Incremental Probabilistic Roadmap (PRM) Algorithm (with connected compoenents):
            // start_pos = new Vector3(terrain.start_pos.x, 0.0f, terrain.start_pos.z);
            // goal_pos = new Vector3(terrain.goal_pos.x, 0.0f, terrain.goal_pos.z);
            // PRMNode startnode = new PRMNode(start_pos);
            // PRMNode goalnode = new PRMNode(goal_pos);
            // prmgraph = new PRMGraph(startnode, goalnode);
            // sampler = new PositionSampler(terrain);

            // prm = new PRMPathPlanner(terrain, prmgraph, startnode, goalnode, sampler);
            // myPRMpath = prm.GenerateIncrementalVariantPRMalgorithm();

            // drawprmgraph = true;
            // drawmypath = true;

            // _______________________________________________________________________________________________________________________________________________________________________________________

            // RRT Connect (Bidirectional RRT, without kinematic constraints, with goal biasing) Search Algorithm:
            // Works like a charm!
            // Vector3 starPos = terrain.start_pos;
            // Vector3 goalPos = terrain.goal_pos;

            // RRTConnectNode rootNode = new RRTConnectNode(starPos);
            // RRTConnectNode goalNode = new RRTConnectNode(goalPos);

            // RRTConnectTree tree_start = new RRTConnectTree(rootNode);
            // RRTConnectTree tree_goal = new RRTConnectTree(goalNode);
            // sampler = new PositionSampler(terrain);

            // rrt_connect = new RRTConnectAlgorithm(
            //     terrain,
            //     tree_start,
            //     tree_goal,
            //     rootNode,
            //     goalNode,
            //     sampler
            // );
            // rrt_connect_path = rrt_connect.FindPathRRTConnect();

            // Vector3 old_wp = goalPos; // or starPos depending on the tree direction of the path.
            // foreach (var wp in rrt_connect_path)
            // {
            //     Debug.DrawLine(old_wp, wp.position, Color.blue, 100f);
            //     old_wp = wp.position;
            // }

            // _______________________________________________________________________________________________________________________________________________________________________________________

            // Lazy Collision Checking in Basic PRM Graph (with improvement; visibility cache, and (optional) optimal heuristic):
            // start_pos = new Vector3(terrain.start_pos.x, 0.0f, terrain.start_pos.z);
            // goal_pos = new Vector3(terrain.goal_pos.x, 0.0f, terrain.goal_pos.z);
            // PRMNode startnode = new PRMNode(start_pos);
            // PRMNode goalnode = new PRMNode(goal_pos);
            // prmgraph = new PRMGraph(startnode, goalnode);
            // sampler = new PositionSampler(terrain);

            // prm = new PRMPathPlanner(terrain, prmgraph, startnode, goalnode, sampler);
            // myPRMpath = prm.CreateLazyBasicPRMvariant();

            // drawprmgraph = true;
            // drawmypath = true;

            // _______________________________________________________________________________________________________________________________________________________________________________________

            // Lazy Incremental Probabilistic Roadmap (PRM) Algorithm (with connected compoenents):
            // Not working correctly, it's just a draft implementation.
            // start_pos = new Vector3(terrain.start_pos.x, 0.0f, terrain.start_pos.z);
            // goal_pos = new Vector3(terrain.goal_pos.x, 0.0f, terrain.goal_pos.z);
            // PRMNode startnode = new PRMNode(start_pos);
            // PRMNode goalnode = new PRMNode(goal_pos);
            // prmgraph = new PRMGraph(startnode, goalnode);
            // sampler = new PositionSampler(terrain);

            // prm = new PRMPathPlanner(terrain, prmgraph, startnode, goalnode, sampler);
            // myPRMpath = prm.GenerateLazyIncrementalVariantPRMalgorithm();

            // drawprmgraph = true;
            // drawmypath = true;

            // _______________________________________________________________________________________________________________________________________________________________________________________

            // Visibility-based Probabilistic Roadmap (Visibility PRM) Algorithm:
            // proposed by T. Simeon, J.P. Laumond, and C. Nissoux. 
            // Improvement: The Visibility PRM drastically reduces the number of nodes in the roadmap, 
            // which allows better performance due to the smaller search space when building edges.

            // start_pos = new Vector3(terrain.start_pos.x, 0.0f, terrain.start_pos.z);
            // goal_pos = new Vector3(terrain.goal_pos.x, 0.0f, terrain.goal_pos.z);
            // PRMNode startnode = new PRMNode(start_pos);
            // PRMNode goalnode = new PRMNode(goal_pos);
            // prmgraph = new PRMGraph(startnode, goalnode);
            // sampler = new PositionSampler(terrain);

            // prm = new PRMPathPlanner(terrain, prmgraph, startnode, goalnode, sampler);
            // int M = 100;

            // // using with List:
            // myPRMpath = prm.VisibilityBasedVariantPRMalgorithmWithList(M); // It can sometimes not find a path when M = 50 due to not enough nodes connecting the start and goal nodes.

            // // using with Stack:
            // // myPRMpath = prm.VisibilityBasedVariantPRMalgorithmWithStack(M);

            // drawprmgraph = true;
            // drawmypath = true;


            // _______________________________________________________________________________________________________________________________________________________________________________________

            // Basic Probabilistic Roadmap (PRM) Algorithm using KD-tree Nearest Neighbour Search:
            // start_pos = new Vector3(terrain.start_pos.x, 0.0f, terrain.start_pos.z);
            // goal_pos = new Vector3(terrain.goal_pos.x, 0.0f, terrain.goal_pos.z);
            // PRMNode startnode = new PRMNode(start_pos);
            // PRMNode goalnode = new PRMNode(goal_pos);
            // prmgraph = new PRMGraph(startnode, goalnode);
            // sampler = new PositionSampler(terrain);

            // prm = new PRMPathPlanner(terrain, prmgraph, startnode, goalnode, sampler);
            // myPRMpath = prm.GenerateBasicPRMalgorithmUsingKDtree();

            // drawprmgraph = true;
            // drawmypath = true;

            // _______________________________________________________________________________________________________________________________________________________________________________________

            sampler = new PositionSampler(terrain);
            DelaunayTriangulation delaunay = new DelaunayTriangulation(terrain, sampler);

            // List<Triangle> incrementalTriangulation = delaunay.IncrementalTriangulation(null);

            // // Draw the triangulation
            // foreach (Triangle triangle in incrementalTriangulation)
            // {
            //     Vector3 p1 = triangle.v1.position;
            //     Vector3 p2 = triangle.v2.position;
            //     Vector3 p3 = triangle.v3.position;

            //     Debug.DrawLine(p1, p2, Color.blue, 100f);
            //     Debug.DrawLine(p2, p3, Color.blue, 100f);
            //     Debug.DrawLine(p3, p1, Color.blue, 100f);
            // }

            // List<Triangle> delaunayTriangulationAlgorithm = delaunay.DelaunayTriangulateByFlippingEdges();

            // foreach (Triangle triangle in delaunayTriangulationAlgorithm)
            // {
            //     Vector3 p1 = triangle.v1.position;
            //     Vector3 p2 = triangle.v2.position;
            //     Vector3 p3 = triangle.v3.position;

            //     Debug.DrawLine(p1, p2, Color.blue, 100f);
            //     Debug.DrawLine(p2, p3, Color.blue, 100f);
            //     Debug.DrawLine(p3, p1, Color.blue, 100f);
            // }

            List<Vector3> obstacleCorners = GetObstacleCornersOrCentroids("corners");
            // foreach (Vector3 corner in obstacleCorners)
            // {
            //     Debug.DrawLine(corner, corner + Vector3.up * 10, Color.magenta, 100f);
            // }


            // List<Vector3> obstacleCentroids = GetObstacleCornersOrCentroids("centroids");
            // foreach (Vector3 centroid in obstacleCentroids)
            // {
            //     Debug.DrawLine(centroid, centroid + Vector3.up * 10, Color.cyan, 100f);
            // }

            // Terrain plane boundary midpoints:
            // float xMidpoint = (terrain_manager.myInfo.x_high + terrain_manager.myInfo.x_low) / 2;
            // float zMidpoint = (terrain_manager.myInfo.z_high + terrain_manager.myInfo.z_low) / 2;
            // Vector3 topMidpoint = new Vector3(xMidpoint, 0.0f, terrain_manager.myInfo.z_high - 5f);
            // Vector3 bottomMidpoint = new Vector3(xMidpoint, 0.0f, terrain_manager.myInfo.z_low + 5f);
            // Vector3 leftMidpoint = new Vector3(terrain_manager.myInfo.x_low + 5f, 0.0f, zMidpoint);
            // Vector3 rightMidpoint = new Vector3(terrain_manager.myInfo.x_high - 5f, 0.0f, zMidpoint);

            // Debug.DrawLine(topMidpoint, topMidpoint + Vector3.up * 10, Color.blue, 100f);
            // Debug.DrawLine(bottomMidpoint, bottomMidpoint + Vector3.up * 10, Color.blue, 100f);
            // Debug.DrawLine(leftMidpoint, leftMidpoint + Vector3.up * 10, Color.blue, 100f);
            // Debug.DrawLine(rightMidpoint, rightMidpoint + Vector3.up * 10, Color.blue, 100f);

            // // merge obstacleCentroids with terrain midpoints:
            // List<Vector3> sites = new List<Vector3>();
            // sites.AddRange(obstacleCentroids);
            // sites.Add(topMidpoint);
            // sites.Add(bottomMidpoint);
            // sites.Add(leftMidpoint);
            // sites.Add(rightMidpoint);

            // UnityEngine.Random.InitState(seed);
            // int numberOfPoints = 150;
            // List<Vector3> sites = sampler.SamplePositions(numberOfPoints);

            // foreach (Vector3 point in obstacleCorners) // sites)
            // {
            //     Debug.DrawLine(point, point + Vector3.up * 10, Color.red, 1000f);
            // }

            // Vector3 topLeftCorner = new Vector3(terrain_manager.myInfo.x_low, 0.0f, terrain_manager.myInfo.z_high);
            // Vector3 topRightCorner = new Vector3(terrain_manager.myInfo.x_high, 0.0f, terrain_manager.myInfo.z_high);
            // Vector3 bottomLeftCorner = new Vector3(terrain_manager.myInfo.x_low, 0.0f, terrain_manager.myInfo.z_low);
            // Vector3 bottomRightCorner = new Vector3(terrain_manager.myInfo.x_high, 0.0f, terrain_manager.myInfo.z_low);

            // Remove the corners that are outside the terrain boundary from the obstacleCorners list:
            // obstacleCorners.Remove(topLeftCorner);
            // obstacleCorners.Remove(topRightCorner);
            // obstacleCorners.Remove(bottomRightCorner);
            // obstacleCorners.Remove(bottomLeftCorner);

            // List<Triangle> triangulateConvexPolygonAlgorithm = delaunay.TriangulateConvexPolygon(obstacleCorners); // sites);
            List<Triangle> delaunayTriangulationAlgorithm = delaunay.DelaunayTriangulateByFlippingEdges(obstacleCorners); // sites);

            // foreach (Triangle triangle in delaunayTriangulationAlgorithm)
            // {
            //     Vector3 p1 = triangle.v1.position;
            //     Vector3 p2 = triangle.v2.position;
            //     Vector3 p3 = triangle.v3.position;

            //     Debug.DrawLine(p1, p2, Color.red, 100f);
            //     Debug.DrawLine(p2, p3, Color.red, 100f);
            //     Debug.DrawLine(p3, p1, Color.red, 100f);
            // }

            Graph<Vector3> centroidGraph = new Graph<Vector3>();

            // Calculate the centroids of triangles:
            List<Vector3> triangleCentroids = new List<Vector3>();
            foreach (Triangle triangle in delaunayTriangulationAlgorithm)
            {
                Vector3 p1 = triangle.v1.position;
                Vector3 p2 = triangle.v2.position;
                Vector3 p3 = triangle.v3.position;

                Vector3 centroid = (p1 + p2 + p3) / 3;
                triangleCentroids.Add(centroid);

                foreach (Triangle triangle2 in delaunayTriangulationAlgorithm)
                {
                    if (triangle == triangle2) continue;
                    if (AreTrianglesAdjacent(triangle, triangle2))
                    {
                        Vector3 p4 = triangle2.v1.position;
                        Vector3 p5 = triangle2.v2.position;
                        Vector3 p6 = triangle2.v3.position;

                        Vector3 centroid2 = (p4 + p5 + p6) / 3;
                        centroidGraph.AddNeighbourSymmetric(centroid, centroid2);

                    }
                }
            }



            // foreach (Triangle triangle in delaunayTriangulationAlgorithm)
            // {
            //     Vector3 p1 = triangle.v1.position;
            //     Vector3 p2 = triangle.v2.position;
            //     Vector3 p3 = triangle.v3.position;

            //     Vector3 centroid = (p1 + p2 + p3) / 3;
            //     triangleCentroids.Add(centroid);

            // }

            // foreach (Vector3 centroid in triangleCentroids)
            // {
            //     foreach (Vector3 neighbour in triangleCentroids)
            //     {
            //         if (centroid == neighbour) continue;
            //         if (Vector3.Distance(centroid, neighbour) < 20f)
            //         {
            //             centroidGraph.AddNeighbourSymmetric(centroid, neighbour);
            //         }
            //     }
            // }



            // Display the centroids of triangles:
            // foreach (Vector3 centroid in triangleCentroids)
            // {
            //     Debug.DrawLine(centroid, centroid + Vector3.up * 10, Color.green, 1000f);
            // }

            // // Display the graph:
            // foreach (Vector3 centroid in centroidGraph.GetNodes())
            // {
            //     foreach (Vector3 neighbour in centroidGraph.GetNeighbors(centroid))
            //     {
            //         Debug.DrawLine(centroid, neighbour, Color.red, 100f);
            //     }
            // }



            // Detect which triangle edges are inside the obstacles:
            // foreach (Triangle triangle in delaunayTriangulationAlgorithm)
            // {
            //     Vector3 p1 = triangle.v1.position;
            //     Vector3 p2 = triangle.v2.position;
            //     Vector3 p3 = triangle.v3.position;

            //     if (IsEdgeInsideObstacle(p1, p2))
            //     {
            //         Debug.DrawLine(p1, p2, Color.red, 100f);
            //     }
            //     if (IsEdgeInsideObstacle(p2, p3))
            //     {
            //         Debug.DrawLine(p2, p3, Color.red, 100f);
            //     }
            //     if (IsEdgeInsideObstacle(p3, p1))
            //     {
            //         Debug.DrawLine(p3, p1, Color.red, 100f);
            //     }
            // }

            // Obstacle barycenters:
            // List<Vector3> obstacleBarycenters = terrain_manager.myInfo.GetObstacleBarycenters();
            // foreach (Vector3 barycenter in obstacleBarycenters)
            // {
            //     Debug.DrawLine(barycenter, barycenter + Vector3.up * 10, Color.red, 1000f);
            // }

            // List<Triangle> delaunayTriangulationAlgorithm = delaunay.DelaunayTriangulateByFlippingEdges(obstacleBarycenters); // sites);

            // foreach (Triangle triangle in delaunayTriangulationAlgorithm)
            // {
            //     Vector3 p1 = triangle.v1.position;
            //     Vector3 p2 = triangle.v2.position;
            //     Vector3 p3 = triangle.v3.position;

            //     Debug.DrawLine(p1, p2, Color.blue, 100f);
            //     Debug.DrawLine(p2, p3, Color.blue, 100f);
            //     Debug.DrawLine(p3, p1, Color.blue, 100f);
            // }


            // _______________________________________________________________________________________________________________________________________________________________________________________

            // Gift Wrapping Jarvis March Convex Hull Algorithm:
            // https://www.habrador.com/tutorials/math/8-convex-hull/
            // JarvisMarchAlgorithm jarvisMarch = new JarvisMarchAlgorithm();

            // Vector3 c1 = new Vector3(60f, 0f, 200f);
            // Vector3 c2 = new Vector3(70f, 0f, 200f);
            // Vector3 c3 = new Vector3(70f, 0f, 220f);
            // Vector3 c4 = new Vector3(130f, 0f, 220f);
            // Vector3 c5 = new Vector3(130f, 0f, 210f);
            // Vector3 c6 = new Vector3(110f, 0f, 210f);
            // Vector3 c7 = new Vector3(110f, 0f, 180f);
            // Vector3 c8 = new Vector3(130f, 0f, 180f);
            // Vector3 c9 = new Vector3(130f, 0f, 130f);
            // Vector3 c10 = new Vector3(120f, 0f, 130f);
            // Vector3 c11 = new Vector3(120f, 0f, 140f);
            // Vector3 c12 = new Vector3(110f, 0f, 140f);
            // Vector3 c13 = new Vector3(110f, 0f, 130f);
            // Vector3 c14 = new Vector3(90f, 0f, 130f);
            // Vector3 c15 = new Vector3(90f, 0f, 100f);
            // Vector3 c16 = new Vector3(140f, 0f, 100f);
            // Vector3 c17 = new Vector3(140f, 0f, 90f);
            // Vector3 c18 = new Vector3(80f, 0f, 90f);
            // Vector3 c19 = new Vector3(80f, 0f, 140f);
            // Vector3 c20 = new Vector3(100f, 0f, 140f);
            // Vector3 c21 = new Vector3(100f, 0f, 210f);
            // Vector3 c22 = new Vector3(80f, 0f, 210f);
            // Vector3 c23 = new Vector3(80f, 0f, 190f);
            // Vector3 c24 = new Vector3(60f, 0f, 190f);

            // List<Vector3> obstaclePolygonCorners = new List<Vector3> { c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11, c12, c13, c14, c15, c16, c17, c18, c19, c20, c21, c22, c23, c24 };
            // List<Vector3> jmConvexHull = jarvisMarch.GetConvexHullAsVector3(obstaclePolygonCorners);

            // // Display the convex hull:
            // Vector3 old_wp = jmConvexHull[0];
            // foreach (var wp in jmConvexHull)
            // {
            //     Debug.DrawLine(old_wp, wp, Color.red, 100f);
            //     old_wp = wp;
            // }

            // _______________________________________________________________________________________________________________________________________________________________________________________

            // Constrained Delaunay Triangulation Algorithm:
            sampler = new PositionSampler(terrain);
            ConstrainedDelaunayTriangulation constrainedDelaunay = new ConstrainedDelaunayTriangulation(terrain, sampler);
            // List<Vector3> obstacleCorners = GetObstacleCornersOrCentroids("corners");

            List<Vector3> constraints = new List<Vector3>();
            constraints.Add(new Vector3(130f, 0f, 180f));
            constraints.Add(new Vector3(130f, 0f, 130f));
            constraints.Add(new Vector3(110f, 0f, 180f));
            constraints.Add(new Vector3(120f, 0f, 130f));


            foreach (Vector3 point in constraints)
            {
                Debug.DrawLine(point, point + Vector3.up * 10, Color.red, 1000f);
            }
            List<Triangle> constrainedDelaunayTriangulationAlgorithm = constrainedDelaunay.GenerateConstrainedDelaunayTriangulation(obstacleCorners, constraints);

            // foreach (Triangle triangle in constrainedDelaunayTriangulationAlgorithm)
            // {
            //     Vector3 p1 = triangle.v1.position;
            //     Vector3 p2 = triangle.v2.position;
            //     Vector3 p3 = triangle.v3.position;

            //     Debug.DrawLine(p1, p2, Color.blue, 100f);
            //     Debug.DrawLine(p2, p3, Color.blue, 100f);
            //     Debug.DrawLine(p3, p1, Color.blue, 100f);
            // }




        }

        private bool SharesEdge(Vector3 a, Vector3 b, Triangle other)
        {
            // Check if edge (a,b) is the same as any edge in 'other'
            bool edge1 = (a.Equals(other.v1.position) && b.Equals(other.v2.position)) || (b.Equals(other.v1.position) && a.Equals(other.v2.position));
            bool edge2 = (a.Equals(other.v2.position) && b.Equals(other.v3.position)) || (b.Equals(other.v2.position) && a.Equals(other.v3.position));
            bool edge3 = (a.Equals(other.v3.position) && b.Equals(other.v1.position)) || (b.Equals(other.v3.position) && a.Equals(other.v1.position));

            return edge1 || edge2 || edge3;
        }

        public bool AreTrianglesAdjacent(Triangle triangle1, Triangle triangle2)
        {
            // Check if this triangle1 shares any edge with 'triangle2'
            return SharesEdge(triangle1.v1.position, triangle1.v2.position, triangle2) ||
                   SharesEdge(triangle1.v2.position, triangle1.v3.position, triangle2) ||
                   SharesEdge(triangle1.v3.position, triangle1.v1.position, triangle2);
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

        public bool Feasible(Vector3 node)
        {
            int i = terrain_manager.myInfo.get_i_index(node.x);
            int j = terrain_manager.myInfo.get_j_index(node.z);
            // 0.0 = air/free, 1.0 = block/obstacle
            if (terrain_manager.myInfo.traversability[i, j] < 0.5f)
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

        // PD Controller parameters for RRT search algorithm:
        public float rrt_k_p = 2.75f; // 1.0f
        public float rrt_k_d = 2f; // 1.0f
        private bool finished = false;
        public Vector3 target_velocity;


        // Simple PD Controller parameters for A* and Hybrid A*:
        public float max_speed = 30f;
        public float max_force = 20;
        public float k_p = 2.75f; // Proportional gain
        public float k_d = 2f; // Derivative gain
        public float max_see_ahead = 10f;
        public float max_avoid_force = 10f;
        public int currentCheckpoint = 1;
        public Vector3 old_target_position;
        public bool stop = false;
        public float MAX_AVOID_FORCE = 10f;


        // Pure Pursuit Controller parameters:
        // public float lookahead_distance = 1;
        // public float desired_speed = 5;
        // private int prev_lookahead_index = 0;
        // public float k_p = 2f;
        // public float k_d = 0.5f;


        // Stanley Controller parameters:
        // public float speed = 10f;
        // public float mu = 1.3f; // Friction coefficient
        // public float minimum_turning_radius = 10f; // Minimum turning radius. Note that this is based on some kind of friction coefficient and the speed of the car.
        // private float distance_between_wheels = 4.47f;
        // public float max_acceleration = 4f;
        // public float max_steering_angle = 25f;
        // public Vector3 velocity = Vector3.zero;
        // public Vector3 pre_position = Vector3.zero;
        // public int max_lookahead = 5; // Maximum number of waypoints to look ahead
        // // Control Parameters
        // public float kp_stanley = 2f; // Proportional gain for stanley controller
        // public float ksoft_stanley = 1f; // Softening gain for stanley controller
        // public float kp_acceleration = 1f; // Proportional gain for acceleration
        // // Private variables
        // private int previous_waypoint = 0;


        // public Vector3 target_velocity_orientation;
        // public Vector3 next_target_position;
        // public float target_velocity_factor = 1.0f;

        // The pathFollowing() method is the one responsible for generating the path following force. 
        // Currently it produces no force, but it does select the targets properly.
        private Vector3 PathFollowingForAStar()
        {
            Vector3 target = Vector3.zero;
            if (myPath != null && myPath.Count > 0)
            {
                target = myPath[currentCheckpoint];
                // next_target_position = myPath[currentCheckpoint + 1];
                // target_velocity_orientation = (next_target_position - target) * target_velocity_factor;
                if (EuclideanDistance(transform.position, target) <= 5) // 10 is margin 
                {
                    currentCheckpoint += 1;
                    if (currentCheckpoint >= myPath.Count)
                    {
                        currentCheckpoint = myPath.Count - 1;
                        stop = true;
                    }
                }
            }
            return target;
        }

        private Vector3 PathFollowingForAStarWithPRM()
        {
            Vector3 target = Vector3.zero;
            if (myPRMpath != null && myPRMpath.Count > 0)
            {
                target = myPRMpath[currentCheckpoint].position;
                // next_target_position = myPRMpath[currentCheckpoint + 1];
                // target_velocity_orientation = (next_target_position - target) * target_velocity_factor;
                if (EuclideanDistance(transform.position, target) <= 5) // 10 is margin 
                {
                    currentCheckpoint += 1;
                    if (currentCheckpoint >= myPRMpath.Count)
                    {
                        currentCheckpoint = myPRMpath.Count - 1;
                        stop = true;
                    }
                }
            }
            return target;
        }

        private Vector3 PathFollowingForRRTStar()
        {
            Vector3 target = Vector3.zero;
            if (rrt_path != null && rrt_path.Count > 0)
            {
                target = rrt_path[index].position;
                if (EuclideanDistance(transform.position, target) <= 5) // 10 is margin 
                {
                    index += 1;
                    if (index >= rrt_path.Count)
                    {
                        index = rrt_path.Count - 1;
                        finished = true;
                    }
                }
            }
            return target;
        }

        private float EuclideanDistance(Vector3 a, Vector3 b)
        {
            return Mathf.Sqrt(Mathf.Pow(a.x - b.x, 2) + Mathf.Pow(a.z - b.z, 2));
        }

        private bool LineIntersectsCircle(Vector3 ahead, Vector3 ahead2, GameObject obstacle)
        {
            // SphereCollider sphereCollider = obstacle.GetComponent<SphereCollider>();
            Vector3 obstacleCenter = obstacle.transform.position;
            float radius = 10.0f; // obstacle.transform.localScale.x / 2;
            return EuclideanDistance(obstacleCenter, ahead) <= radius || EuclideanDistance(obstacleCenter, ahead2) <= radius;
        }

        // private GameObject FindMostThreateningObstacle()
        // {
        //     GameObject mostThreatening = null;
        //     foreach (GameObject obstacle in obstacles)
        //     {
        //         bool collision = LineIntersectsCircle(ahead, ahead2, obstacle);
        //         if (collision && (mostThreatening == null || EuclideanDistance(transform.position, obstacle.transform.position) < EuclideanDistance(transform.position, mostThreatening.transform.position)))
        //         {
        //             mostThreatening = obstacle;
        //         }
        //     }
        //     return mostThreatening;
        // }

        Vector3 targetForCar;
        Vector3 targetForCar2;

        public float maxSpeed = 30.0f;
        public float maxForce = 10.0f;
        public float mass = 1000.0f;

        private void FixedUpdate()
        {
            // ahead = transform.position + myRigidBody.velocity.normalized * max_see_ahead; // The ahead vector is the character's line of sight.
            // ahead2 = transform.position + myRigidBody.velocity.normalized * max_see_ahead * 0.5f;

            // avoidance_force = (ahead - new Vector3(205f, 0.0f, 215f)).normalized * MAX_AVOID_FORCE;
            // Debug.DrawLine(transform.position, avoidance_force, Color.green, 0.1f);

            // ahead2 = ahead;
            // ahead3 = ahead;
            // ahead2.x = ahead.x + 3f;
            // ahead3.x = ahead.x - 3f;

            // GameObject[] obstacles = GameObject.FindGameObjectsWithTag("Box");
            // foreach (GameObject obstacle in obstacles)
            // {
            //     bool collision = LineIntersectsCircle(ahead, ahead2, obstacle);
            //     if (collision)
            //     {
            //         Debug.Log("Collision detected");

            //         avoidance_force = (ahead - obstacle.transform.position).normalized * MAX_AVOID_FORCE;
            //         Debug.Log("Avoidance force: " + avoidance_force);

            //         steering += avoidance_force;
            //         steering = Vector3.ClampMagnitude(steering, maxForce);
            //         steering /= mass;
            //         velocity = Vector3.ClampMagnitude(velocity + steering, maxSpeed);

            //         // Add the avoidance force to the desired_acceleration
            //         // desired_acceleration += avoidance_force;

            //         // Recalculate steering and acceleration based on the updated desired_acceleration
            //         float steeringValue = Vector3.Dot(velocity.normalized, transform.right);
            //         float accelerationValue = Vector3.Dot(velocity.normalized, transform.forward);

            //         // Apply the updated steering and acceleration to the car's movement
            //         m_Car.Move(steeringValue, accelerationValue, accelerationValue, 0f);
            //     } 
            //     else
            //     {
            //         Vector3 my_position = transform.position;
            //         Vector3 target_position = new Vector3(190f, 0.0f, 195f);
            //         Vector3 position_error = target_position - my_position;
            //         Vector3 velocity_error = myRigidBody.velocity * (max_speed - myRigidBody.velocity.magnitude);
            //         desired_acceleration = position_error * k_p + velocity_error * k_d;
            //         float steeringValue = Vector3.Dot(desired_acceleration, transform.right);
            //         float accelerationValue = Vector3.Dot(desired_acceleration, transform.forward);
            //         m_Car.Move(steeringValue, accelerationValue, accelerationValue, 0f);
            //     }
            // }

            // _______________________________________________________________________________________________________________________________________________________________________________________

            // // Simple PD-controller, works for A*, Hybrid A* and PRM Graph:
            Vector3 my_position = transform.position;
            // Moving From Node to Node:
            // Vector3 target_position = PathFollowingForAStar();
            Vector3 target_position = PathFollowingForAStarWithPRM();
            // Calculating and Adding Forces: Seek Behavior
            Vector3 position_error = target_position - my_position; // aka target_position

            // Vector3 target_velocity = new Vector3(0f, 0f, 0f);
            // Vector3 current_velocity = myRigidBody.velocity;
            // Vector3 velocity_error = target_velocity - current_velocity; // aka target_velocity

            Vector3 velocity_error = myRigidBody.velocity * (max_speed - myRigidBody.velocity.magnitude);

            Vector3 desired_acceleration = position_error * k_p + velocity_error * k_d;

            float steering = Vector3.Dot(desired_acceleration, transform.right);
            float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

            // Debug.Log("Forward Amount : " + acceleration.ToString() + " Turn Amount: " + steering.ToString());

            // if the path is behind the car turn around:
            if (Vector3.Dot(myRigidBody.velocity, position_error) < 0)
            {
                // myRigidBody.velocity is used here because it is the direction the car is currently moving in
                steering = -steering;
                // acceleration = -acceleration;
            }

            // This is how you control the car
            if (!stop)
                m_Car.Move(steering, acceleration, acceleration, 0f);
            else
                m_Car.Move(0f, 0f, 0f, 0f);

            // _______________________________________________________________________________________________________________________________________________________________________________________

            // Simple PD-controller, works for RRT:
            // if (target == null)
            //     return;

            // // Track next node position and vel
            // // Vector3 target_pos = target;
            // // target_velocity = transform.TransformDirection(Vector3.forward);

            // Vector3 target_pos = PathFollowingForRRTStar();
            // target_velocity = target.GetModelState().Orientation; // drive robot using control inputs stored along edges in the tree

            // // a PD controller to get desired velocity
            // // OBS! myRigidBody.velocity can be viewed as the current direction the car is moving in
            // Vector3 position_error = target_pos - transform.position;
            // Vector3 velocity_error = (target_velocity - myRigidBody.velocity).normalized;
            // Vector3 desired_acceleration = rrt_k_p * position_error + rrt_k_d * velocity_error;

            // float steering = Vector3.Dot(desired_acceleration, transform.right);
            // float acceleration = Vector3.Dot(desired_acceleration, transform.forward);

            // Debug.DrawLine(transform.position, transform.position + myRigidBody.velocity, Color.yellow);

            // // why is myRigidBody.velocity facing the opposite direction?
            // Debug.DrawLine(transform.position, transform.position + myRigidBody.velocity, Color.red);
            // // I want it to face in front of the car:
            // Debug.DrawLine(transform.position, transform.position + transform.TransformDirection(Vector3.forward) * myRigidBody.velocity.magnitude, Color.green);

            // // It WORKS! The car moves in the right direction now. It's all thanks to understanding the direction of the velocity vector!
            // Vector3 heading_front = transform.TransformDirection(Vector3.forward) * myRigidBody.velocity.magnitude;
            // if (Vector3.Dot(heading_front, myRigidBody.velocity) < 0)
            // {
            //     steering = -steering;
            // }

            // if (myRigidBody.velocity.magnitude < 13)
            //     m_Car.Move(steering, acceleration, acceleration, 0f);
            // else
            //     m_Car.Move(steering, 0, 0, 1.0f);

            // if (finished)
            // {
            //     m_Car.Move(0, 0, 0, 0);
            // }




            // Check if we reached the target.
            // PathFollowingForRRTStar() procedure for RRT:
            // if (Vector3.Distance(transform.position, target_pos) < 8.0f)
            // {
            //     index += 1;
            //     if (index < rrt_path.Count - 1)
            //     {
            //         target = rrt_path[index];
            //         // Debug.Log($"New target: #{index}. Pos: {target.position}, Vel: {target.GetModelState().Orientation}");
            //         // Debug.Log($"New target: #{index}. Pos: {target}, Vel: {target_velocity}");
            //     }
            //     else
            //     {
            //         finished = true;
            //         Debug.Log("Finish!");
            //     }
            // }

            // _______________________________________________________________________________________________________________________________________________________________________________________

            // // Pure Pursuit PD-controller:
            // // Find the path reference point closes to the vehicle
            // Vector3 LookaheadPoint = GetLookaheadPoint(myPath, my_position);

            // // Debug the x,y,z values of the lookahead point
            // // Debug.LogFormat("Lookahead: x: {0}, y: {1}, z: {2}", LookaheadPoint.x, LookaheadPoint.y, LookaheadPoint.z);

            // // Calculate the steering angle and throttle using PurePursuitPDController
            // List<float> m = PurePursuitPDController(my_position, LookaheadPoint, desired_speed);
            // m_Car.Move(m[0], m[1], m[2], m[3]);

            // _______________________________________________________________________________________________________________________________________________________________________________________

            // RaycastHit hit;
            // float maxRange = 50f;
            // if (Physics.Raycast(transform.position + transform.up, transform.TransformDirection(Vector3.forward), out hit, maxRange))
            // {
            //     Vector3 closestObstacleInFront = transform.TransformDirection(Vector3.forward) * hit.distance;
            //     Debug.DrawRay(transform.position, closestObstacleInFront, Color.yellow);
            //     //Debug.Log("Did Hit");
            // }

            // foreach (GameObject car in carFriends)
            // {
            //     CarController carController = car.GetComponent<CarController>();
            //     if (car.name == "Car")
            //     {
            //         // FLEES FROM CAR (2):
            //         // Debug.Log("You're fleeing from Car (2)");
            //         // targetForCar2 = car.transform.position;
            //         Debug.Log("1 You're " + car.name);
            //     }
            //     else if (car.name == "Car (2)")
            //     {
            //         // CHASES CAR:
            //         // Debug.Log("You're chasing Car ");
            //         // targetForCar = car.transform.position;
            //         Debug.Log("2 You're " + car.name);
            //     }
            // }

            // TODO: Collection of steering behaviors: https://www.youtube.com/watch?v=O_10_1WDj6Y&ab_channel=luis
            // https://github.com/luis-l/UnityAssets/blob/master/Assets/Plugins/AutonomousAgents/Scripts/Steering/FollowPath2D.cs
            // 1. Seek
            // 2. Flee
            // 3. Arrive
            // 4. Pursuit
            // 5. Offset Pursuit
            // 6. Interpose
            // 7. Evade
            // 8. Wander
            // 9. Follow Path
            // 10. Obstacle/Wall Avoidance
            // 11. Hide
            // 12. Flock (300 Agents)
            // 13. Flock + Path
        }

    }
}