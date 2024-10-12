using System;
using System.Collections.Generic;
using System.Data;
using UnityEngine;
public class PRMPathPlanner
{
    private readonly int MAX_ITERATIONS = 200; // 20000; //400000; // Change this value if you want to run for longer time to find a path (reach goal)

    private PRMNode startNode;
    private PRMNode goalNode;

    private readonly PositionSampler sampler;

    public PRMGraph graph;
    public float radius = 50f;
    private float carHeight = 3f;
    private float carWidth = 3f;

    public bool pathFound = false;

    private readonly TerrainInfo info;

    public int k = 2; // number of dimensions
    // public KDtree kdTree;

    private void Log(string msg) { Debug.Log($"[PRM]: {msg}"); }

    /*
        So far the tuning parameters that computes a good Basic PRM graph are:
        - MAX_ITERATIONS = 300
        - radius = 50f (the nearness criterion; either number of neighbors 𝑘 or radius 𝑅)
        - distEpsilon = 0.001f
        - int n = 100; // number of nodes to generate
    */

    /*
        So far the tuning parameters that computes a good Incremental PRM graph with CC (computes faster) are:
        - MAX_ITERATIONS = 1000
        - radius = 20f (the nearness criterion; either number of neighbors 𝑘 or radius 𝑅). The smaller R is the zig-zaggier (jerkier) the path is (jerkiness of the produced path is more pronounced), which it tends to be more pronounced when Incremental PRM is used, or when fewer neighbors are connected.
        - distEpsilon = 0.001f
    */

    /*
        So far the tuning parameters that computes a pretty good Lazy Basic PRM graph are:
        - n = 150+ with edge visibility cache (speed improvement) & optimal heuristic A* search
        - radius = 50f (the nearness criterion; either number of neighbors 𝑘 or radius 𝑅)
        - distEpsilon = 0.001f
    */

    /*
        So far the tuning parameters that computes a pretty good Visibility-based PRM graph are:
        - M = 100+ (can sometimes not find a path when M = 50 or 100)
        - radius = 50f (the nearness criterion; either number of neighbors 𝑘 or radius 𝑅)
        - distEpsilon = 0.001f
    */

    public PRMPathPlanner(TerrainInfo info, PRMGraph graph, PRMNode startNode, PRMNode goalNode, PositionSampler sampler)
    {
        this.info = info;
        this.graph = graph;
        this.startNode = startNode;
        this.goalNode = goalNode;
        this.sampler = sampler;
    }

    // A different alternative to Basic PRM algorithm from a research paper:
    // https://www.cs.columbia.edu/~allen/F15/NOTES/Probabilisticpath.pdf
    public void GenerateBasicPRMalgorithm()
    {
        List<PRMNode> V = new List<PRMNode> { startNode, goalNode };
        int n = 150; // number of nodes to generate
        Vector3 randomPosition;
        while (V.Count < n)
        {
            // Sample node (sampling a random position within the drivable space. Avoiding the obstacles.) (Sample() method)
            randomPosition = sampler.SamplePosition();
            if (UnityEngine.Random.value > 0.9) // bias distribution towards the goal, pick the goal point with a 0.9 probability
                randomPosition = goalNode.position;

            PRMNode randomNode = new PRMNode(randomPosition);
            if (!V.Contains(randomNode))
            {
                V.Add(randomNode);
            }
        }

        foreach (PRMNode node in V)
        {
            List<PRMNode> nearestNodes = ClosestNeighborsOfNodeChosenFromVsetAccordingToDist(node.position, V, radius);
            foreach (PRMNode nearestNode in nearestNodes)
            {
                if (!node.hasNeighbour(nearestNode) && VisibleRecurse(node, nearestNode))
                {
                    node.AddNeighbourSymmetric(nearestNode);
                    if (!graph.ContainsNode(node))
                    {
                        graph.AddNode(node);
                    }
                }
            }
        }

        // Instead of adding the start and goal nodes to the graph, we add them to the V set and connect them to the nodes in the V set.
        // List<PRMNode> nearestNodesToStartNode = ClosestNeighborsOfNodeChosenFromVsetAccordingToDist(startNode.position, V, radius);
        // List<PRMNode> nearestNodesToGoalNode = ClosestNeighborsOfNodeChosenFromVsetAccordingToDist(goalNode.position, V, radius);

        // foreach (PRMNode nearestNode in nearestNodesToStartNode)
        // {
        //     if (!startNode.hasNeighbour(nearestNode) && VisibleRecurse(startNode, nearestNode))
        //     {
        //         startNode.AddNeighbourSymmetric(nearestNode);
        //         if (!graph.ContainsNode(startNode))
        //         {
        //             graph.AddNode(startNode);
        //         }
        //     }
        // }

        // foreach (PRMNode nearestNode in nearestNodesToGoalNode)
        // {
        //     if (!goalNode.hasNeighbour(nearestNode) && VisibleRecurse(goalNode, nearestNode))
        //     {
        //         goalNode.AddNeighbourSymmetric(nearestNode);
        //         if (!graph.ContainsNode(goalNode))
        //         {
        //             graph.AddNode(goalNode);
        //         }
        //     }
        // }
    }

    public void GenerateBasicPRMalgorithm2()
    {
        Log("Generating a basic PRM graph roadmap ...");

        int iterations = 0;

        while (iterations < MAX_ITERATIONS)
        {
            if (iterations >= MAX_ITERATIONS)
            {
                Log($"Iteration timeout at {MAX_ITERATIONS}");
                break;
            }

            // Radius modification: Rad = yprm*(log(itr)/(itr))^(1/d)
            // yprm = 1.1; 
            // d = 2;
            // Rad = 1.1*(log(1000)/(1000))^(1/2) = 0.1

            iterations++;

            // Sample node (sampling a random position within the drivable space. Avoiding the obstacles.) (Sample() method)
            Vector3 randomPosition = sampler.SamplePosition();

            if (UnityEngine.Random.value > 0.9) // bias distribution towards the goal, pick the goal point with a 0.9 probability
                randomPosition = goalNode.position;

            PRMNode randomNode = new PRMNode(randomPosition);

            if (graph.ContainsNode(randomNode))
            {
                continue;
            }
            graph.AddNode(randomNode);

            List<PRMNode> nearestNodes = ListOfNodesInGeneratedNodeProximity(randomNode.position, radius);

            // nearestNodes.Sort((nodeA, nodeB) => Vector3.Distance(nodeA.position, randomNode.position).CompareTo(Vector3.Distance(nodeB.position, randomNode.position)));

            foreach (PRMNode nearestNode in nearestNodes)
            {
                if (!randomNode.hasNeighbour(nearestNode) && VisibleRecurse(randomNode, nearestNode))
                {
                    randomNode.AddNeighbourSymmetric(nearestNode);
                }
            }
        }

        List<PRMNode> nearestNodesToStartNode = ListOfNodesInGeneratedNodeProximity(startNode.position, radius);
        List<PRMNode> nearestNodesToGoalNode = ListOfNodesInGeneratedNodeProximity(goalNode.position, radius);

        foreach (PRMNode nearestNode in nearestNodesToStartNode)
        {
            if (!startNode.hasNeighbour(nearestNode) && VisibleRecurse(startNode, nearestNode))
            {
                startNode.AddNeighbourSymmetric(nearestNode);
                if (!graph.ContainsNode(startNode))
                {
                    graph.AddNode(startNode);
                }
            }
        }

        foreach (PRMNode nearestNode in nearestNodesToGoalNode)
        {
            if (!goalNode.hasNeighbour(nearestNode) && VisibleRecurse(goalNode, nearestNode))
            {
                goalNode.AddNeighbourSymmetric(nearestNode);
                if (!graph.ContainsNode(goalNode))
                {
                    graph.AddNode(goalNode);
                }
            }
        }
    }

    // Incremental PRM is an Improvement for Basic PRM: Provides more efficient collision queries (faster)
    public List<PRMNode> GenerateIncrementalVariantPRMalgorithm()
    {
        Log("Generating an Incremental variant of PRM graph roadmap ...");

        int iterations = 0;
        List<PRMNode> pathPRM = new List<PRMNode>();
        List<PRMNode> V = new List<PRMNode> { startNode, goalNode };

        Dictionary<PRMNode, HashSet<PRMNode>> CC = new Dictionary<PRMNode, HashSet<PRMNode>>();
        CC[startNode] = new HashSet<PRMNode> { startNode };
        CC[goalNode] = new HashSet<PRMNode> { goalNode };

        while (iterations < MAX_ITERATIONS)
        {
            if (iterations >= MAX_ITERATIONS)
            {
                Log($"Iteration timeout at {MAX_ITERATIONS}");
                break;
            }

            iterations++;

            // Sample node (sampling a random position within the drivable space. Avoiding the obstacles.) (Sample() method)
            Vector3 randomPosition = sampler.SamplePosition();

            if (UnityEngine.Random.value > 0.9) // bias distribution towards the goal, pick the goal point with a 0.9 probability
                randomPosition = goalNode.position;

            PRMNode randomNode = new PRMNode(randomPosition);
            if (!V.Contains(randomNode))
            {
                V.Add(randomNode);
            }
            if (!graph.ContainsNode(randomNode))
            {
                graph.AddNode(randomNode);
            }
            CC[randomNode] = new HashSet<PRMNode> { randomNode }; // for now, random node gets its own connected component

            List<PRMNode> nearestNodes = ClosestNeighborsOfNodeChosenFromVsetAccordingToDist(randomNode.position, V, radius);

            foreach (PRMNode nearestNode in nearestNodes)
            {
                if (randomNode == nearestNode) continue;
                if (!randomNode.hasNeighbour(nearestNode) && VisibleRecurse(randomNode, nearestNode))
                {
                    randomNode.AddNeighbourSymmetric(nearestNode);

                    CC = MergeCC(nearestNode, randomNode, CC);
                    // if (CC[startNode].SetEquals(CC[goalNode]))
                    if (CC[startNode].Contains(goalNode))
                    {
                        Debug.Log("Start and goal nodes are now in the same connected component");
                        pathPRM = FindPathFromStartToGoalUsingAstar();
                        return pathPRM;
                    }
                }
            }
        }
        return pathPRM;
    }

    // Doesn't work. Not implemented correctly. It's just a draft. Would be nice to make it work. In progress...
    public List<PRMNode> GenerateLazyIncrementalVariantPRMalgorithm()
    {
        Log("Generating a Lazy Incremental variant of PRM graph roadmap ...");

        int iterations = 0;
        List<PRMNode> pathPRM = new List<PRMNode>();
        List<PRMNode> V = new List<PRMNode> { startNode, goalNode };

        Dictionary<PRMNode, HashSet<PRMNode>> CC = new Dictionary<PRMNode, HashSet<PRMNode>>();
        CC[startNode] = new HashSet<PRMNode> { startNode };
        CC[goalNode] = new HashSet<PRMNode> { goalNode };

        while (iterations < MAX_ITERATIONS)
        {
            if (iterations >= MAX_ITERATIONS)
            {
                Log($"Iteration timeout at {MAX_ITERATIONS}");
                break;
            }

            iterations++;

            // Sample node (sampling a random position within the drivable space. Avoiding the obstacles.) (Sample() method)
            Vector3 randomPosition = sampler.SamplePosition();

            if (UnityEngine.Random.value > 0.9) // bias distribution towards the goal, pick the goal point with a 0.9 probability
                randomPosition = goalNode.position;

            PRMNode randomNode = new PRMNode(randomPosition);
            if (!V.Contains(randomNode))
            {
                V.Add(randomNode);
            }
            if (!graph.ContainsNode(randomNode))
            {
                graph.AddNode(randomNode);
            }
            CC[randomNode] = new HashSet<PRMNode> { randomNode }; // for now, random node gets its own connected component

            List<PRMNode> nearestNodes = ClosestNeighborsOfNodeChosenFromVsetAccordingToDist(randomNode.position, V, radius);

            foreach (PRMNode nearestNode in nearestNodes)
            {
                if (randomNode == nearestNode) continue;
                if (!randomNode.hasNeighbour(nearestNode)) // && VisibleRecurse(randomNode, nearestNode))
                {
                    randomNode.AddNeighbourSymmetric(nearestNode);

                    CC = MergeCC(nearestNode, randomNode, CC);
                    // if (CC[startNode].SetEquals(CC[goalNode]))
                    if (CC[startNode].Contains(goalNode))
                    {

                        Debug.Log("Contains goalNode in CC[startNode]");
                        pathPRM = FindPathFromStartToGoalUsingAstar();

                        // Draw pathPRM:
                        for (int i = 0; i < pathPRM.Count - 1; i++)
                        {
                            PRMNode vi = pathPRM[i];
                            PRMNode viPlus1 = pathPRM[i + 1];
                            Debug.DrawLine(vi.position, viPlus1.position, Color.green, 5f);

                            if (!VisibleRecurse(vi, viPlus1))
                            {
                                Debug.Log($"Edge ({vi.position}, {viPlus1.position}) is not feasible. Removing...");
                                Debug.DrawLine(vi.position, viPlus1.position, Color.red, 5f);
                                vi.RemoveNeighbourSymmetric(viPlus1);
                                CC = DismergeCC(vi, viPlus1, CC);
                                break;
                            }
                        }

                        return pathPRM;

                        // return FindPathInLazyIncrementalPRMusingAstarSearch(CC);

                        // while (true)
                        // {
                        //     pathPRM = FindPathFromStartToGoalUsingAstar();

                        //     bool pathFeasible = CheckPathForCollision(pathPRM);
                        //     Debug.Log("Path is feasible: " + pathFeasible);

                        //     if (pathFeasible)
                        //     {
                        //         Debug.Log("Start and goal nodes are now in the same connected component");
                        //         return pathPRM;
                        //     }
                        //     else
                        //     {
                        //         Debug.Log("Edge is not feasible. Deleting it from E and restarting pathfinding.");
                        //         randomNode.RemoveNeighbourSymmetric(nearestNode);
                        //         // CC = DismergeCC(randomNode, nearestNode, CC);
                        //         // break;
                        //     }
                        // }
                    }
                    else
                    {
                        Debug.Log("NO goalNode in CC[startNode]");
                    }

                }
            }
        }
        return pathPRM;
    }



    // There is also an issue of jerkiness of the produced path, which tends to be more pronounced when Incremental PRM is used, 
    // or fewer neighbors are connected. We shall see strategies to address the jerkiness of PRM paths when discussing 
    // "optimizing roadmaps" and "shortcutting".
    // TO BE CONTINUED ...



    // This VisibilityBasedVariantPRMalgorithm() method iteratively processes two sets of nodes: Guard and Connection. The nodes of Guard belonging to a same connected component
    // (i.e. connected by nodes of Connection) are gathered in subsets G_i.
    // At each elementary iteration, the algorithm randomly selects a collision-free configuration q. The main loop processes all the current components
    // G_i of Guard. The algorithm loops over the nodes g in G_i, until it fins a node visible from q. 
    // The first time the algorithm succeeds in finding such a visible node g, it memorizes both g and its component G_i and 
    // switches to the next component G_i+1. When q "sees" another guard g' in another component G_j, the algorithm adds q to the 
    // Connection set and the component G_j is merged with the memorized G_i. If q is not visible from any component, it is added
    // to the Guard set. The main loop fails to create a new node when q is visible from only one component; in that case q is rejected.
    // https://msl.cs.uiuc.edu/~lavalle/cs576/projects/kevrotti/index.html (Code Implementation)
    // https://laas.hal.science/hal-01993321/document (Paper)
    // https://stefanosnikolaidis.net/course-files/CS545/Lecture_CSCI545_VisPRM.pdf (Powerpoint)
    // https://sgvr.kaist.ac.kr/~sungeui/mp/prm.pdf (Article)
    public List<PRMNode> VisibilityBasedVariantPRMalgorithmWithList(int M)
    {
        Vector3 qvector = Vector3.zero;
        PRMNode qnode = null;
        List<PRMNode> Guards = new List<PRMNode> { startNode, goalNode };
        List<PRMNode> Connectors = new List<PRMNode>();

        int rejects = 0;
        while (rejects < M)
        {
            qvector = sampler.SamplePosition();

            List<PRMNode> visibleGuards = new List<PRMNode>();
            List<PRMNode> connectedComponentNodes = new List<PRMNode>();

            foreach (PRMNode guardCurrent in Guards)
            {
                if (VisibleRecurse(qvector, guardCurrent.position))
                {
                    visibleGuards.Add(guardCurrent);
                }
            }

            // now visibleGuards holds a list of all the guards visible by qvector

            if (visibleGuards.Count == 0)
            {
                // qvector will become a guard
                qnode = new PRMNode(qvector);
                Guards.Add(qnode);
                if (!graph.ContainsNode(qnode))
                {
                    graph.AddNode(qnode);
                }
                rejects = 0;
            }
            else // if !visibleGuards.empty()
            {
                // now we need to prune down visibleGuards to a single
                // representative from each connected component 
                // (aka merge connected components)
                while (visibleGuards.Count != 0)
                {
                    PRMNode lastElement = visibleGuards[visibleGuards.Count - 1];
                    connectedComponentNodes.Add(lastElement);
                    visibleGuards.RemoveAt(visibleGuards.Count - 1);

                    // Initialize all nodes to false in Gnode list:
                    Dictionary<PRMNode, bool> Gnode = new Dictionary<PRMNode, bool>();
                    List<PRMNode> reachable = new List<PRMNode>();
                    reachable = DFS(reachable, connectedComponentNodes[connectedComponentNodes.Count - 1], Gnode);

                    foreach (PRMNode tempnode in reachable)
                    {
                        visibleGuards.Remove(tempnode);
                    }
                }
                if (connectedComponentNodes.Count > 1)
                {
                    // qvector will become connector
                    qnode = new PRMNode(qvector);
                    Connectors.Add(qnode);
                    if (!graph.ContainsNode(qnode))
                    {
                        graph.AddNode(qnode);
                    }
                    rejects = 0;
                    foreach (PRMNode tempnode in connectedComponentNodes)
                    {
                        qnode.AddNeighbourSymmetric(tempnode);
                    }
                }
                else
                {
                    // discard node, it is only connected to one connected component
                    rejects++;
                }

            }
        }

        return FindPathFromStartToGoalUsingAstar();
    }

    public List<PRMNode> VisibilityBasedVariantPRMalgorithmWithStack(int M)
    {
        Vector3 qvector = Vector3.zero;
        PRMNode qnode = null;
        List<PRMNode> Guards = new List<PRMNode> { startNode, goalNode };
        List<PRMNode> Connectors = new List<PRMNode>();

        int rejects = 0;
        while (rejects < M)
        {
            qvector = sampler.SamplePosition();

            // utilizes a stack-based approach to keep track of the connected components
            Stack<PRMNode> visibleGuardsStack = new Stack<PRMNode>();
            Stack<PRMNode> connectedComponentNodesStack = new Stack<PRMNode>();

            foreach (PRMNode guardCurrent in Guards)
            {
                if (VisibleRecurse(qvector, guardCurrent.position))
                {
                    visibleGuardsStack.Push(guardCurrent);
                }
            }

            // now visibleGuards holds a list of all the guards visible by qvector

            if (visibleGuardsStack.IsEmpty())
            {
                // qvector will become a guard
                qnode = new PRMNode(qvector);
                Guards.Add(qnode);
                if (!graph.ContainsNode(qnode))
                {
                    graph.AddNode(qnode);
                }
                rejects = 0;
            }
            else // if !visibleGuards.empty()
            {
                // now we need to prune down visibleGuards to a single
                // representative from each connected component 
                // (aka merge connected components)
                while (!visibleGuardsStack.IsEmpty())
                {
                    connectedComponentNodesStack.Push(visibleGuardsStack.Pop());

                    // Initialize all nodes to false in Gnode list:
                    Dictionary<PRMNode, bool> Gnode = new Dictionary<PRMNode, bool>();
                    List<PRMNode> reachable = new List<PRMNode>();
                    reachable = DFS(reachable, connectedComponentNodesStack.Front(), Gnode);

                    foreach (PRMNode tempnode in reachable)
                    {
                        visibleGuardsStack.Remove(tempnode);
                    }

                }
                if (connectedComponentNodesStack.length() > 1)
                {
                    // qvector will become connector
                    qnode = new PRMNode(qvector);
                    Connectors.Add(qnode);
                    if (!graph.ContainsNode(qnode))
                    {
                        graph.AddNode(qnode);
                    }
                    rejects = 0;
                    foreach (PRMNode tempnode in connectedComponentNodesStack.GetElements())
                    {
                        qnode.AddNeighbourSymmetric(tempnode);
                    }
                }
                else
                {
                    // discard node, it is only connected to one connected component
                    rejects++;
                }

            }
        }

        return FindPathFromStartToGoalUsingAstar();
    }

    // Stack data structure for simulating the recursive DFS algorithm:
    public class Stack<T>
    {
        private List<T> elements = new List<T>();

        // get list of elements
        public List<T> GetElements()
        {
            return elements;
        }

        public void Push(T element)
        {
            elements.Add(element);
        }

        public T Pop()
        {
            T element = elements[elements.Count - 1];
            elements.RemoveAt(elements.Count - 1);
            return element;
        }

        public T Front()
        {
            return elements[elements.Count - 1];
        }

        public bool IsEmpty()
        {
            return elements.Count == 0;
        }

        public int length()
        {
            return elements.Count;
        }

        public void Remove(T element)
        {
            elements.Remove(element);
        }

        // Without using <T>:
        // private List<PRMNode> stack = new List<PRMNode>();

        // public void Push(PRMNode node)
        // {
        //     stack.Add(node);
        // }

        // public PRMNode Pop()
        // {
        //     if (stack.Count == 0)
        //     {
        //         return null;
        //     }
        //     PRMNode node = stack[stack.Count - 1];
        //     stack.RemoveAt(stack.Count - 1);
        //     return node;
        // }

        // // Front of the stack:
        // public PRMNode Front()
        // {
        //     if (stack.Count == 0)
        //     {
        //         return null;
        //     }
        //     return stack[stack.Count - 1];
        // }

        // public void Remove(PRMNode node)
        // {
        //     stack.Remove(node);
        // }

        // public int length()
        // {
        //     return stack.Count;
        // }

        // public bool IsEmpty()
        // {
        //     return stack.Count == 0;
        // }

    }




    // The Lazy PRM algorithm can be implemented in both basic and incremental forms. A lazy Basic PRM variant is as follows:
    // 1. Create a PRM(𝑉,𝐸), assuming IsVisible always returns true.
    public List<PRMNode> CreateLazyBasicPRMvariant()
    {
        List<PRMNode> V = new List<PRMNode> { startNode, goalNode };
        int n = 150; // number of nodes to generate
        Vector3 randomPosition;
        while (V.Count < n)
        {
            // Sample node (sampling a random position within the drivable space. Avoiding the obstacles.) (Sample() method)
            randomPosition = sampler.SamplePosition();
            if (UnityEngine.Random.value > 0.9) // bias distribution towards the goal, pick the goal point with a 0.9 probability
                randomPosition = goalNode.position;

            PRMNode randomNode = new PRMNode(randomPosition);
            if (!V.Contains(randomNode))
            {
                V.Add(randomNode);
            }
        }

        foreach (PRMNode node in V)
        {
            List<PRMNode> nearestNodes = ClosestNeighborsOfNodeChosenFromVsetAccordingToDist(node.position, V, radius);
            foreach (PRMNode nearestNode in nearestNodes)
            {
                if (node == nearestNode) continue;
                if (!node.hasNeighbour(nearestNode))
                {
                    node.AddNeighbourSymmetric(nearestNode);
                    if (!graph.ContainsNode(node))
                    {
                        graph.AddNode(node);
                    }
                }
            }
        }
        return FindPathInLazyBasicPRMusingAstarSearch();
    }

    // 2. Find a path from 𝑠 to 𝑔, 𝑣1=𝑠,𝑣2,…,𝑣𝑚−1,𝑣𝑚=𝑔 using search. If no path is found, return failure.
    // 3. Check each edge VisibleRecurse (𝑣𝑖,𝑣𝑖+1), 𝑖=1,...,𝑚−1 for collision.
    // 4. If any edge (𝑣𝑖,𝑣𝑖+1) is not feasible, delete it from  𝐸 and return to 2.
    // 5. If all edges are feasible, return 𝑣1→𝑣2→⋯→𝑣𝑚 as the path.
    public List<PRMNode> FindPathInLazyBasicPRMusingAstarSearch()
    {
        while (true) // Loop to restart the pathfinding process if needed
        {
            List<PRMNode> myPath = FindPathFromStartToGoalUsingAstar();

            // Check each edge for visibility and collision
            bool pathFeasible = true; // Assume path is feasible until proven otherwise
            for (int i = 0; i < myPath.Count - 1; i++)
            {
                PRMNode nodeA = myPath[i];
                PRMNode nodeB = myPath[i + 1];

                // Slow! Takes more time to compute:
                // if (!VisibleRecurse(nodeA, nodeB))
                // {
                //     Debug.Log("Edge is not feasible. Deleting it from E and restarting pathfinding.");
                //     // Remove edge from graph (E)
                //     nodeA.RemoveNeighbourSymmetric(nodeB);
                //     pathFeasible = false; // Set flag to indicate that the path is not feasible
                //     break; // Exit the loop to restart the pathfinding process
                // }

                // Speed Improvement!: Takes less time to compute (aka is faster) because it uses the visibility cache:
                // Can use more nodes to sample when constructing the roadmap
                if (!IsEdgeVisible(nodeA, nodeB))
                {
                    Debug.Log("Edge is not feasible. Deleting it from E and restarting pathfinding.");
                    // Remove edge from graph and visibility cache
                    RemoveEdgeFromGraphAndCache(nodeA, nodeB);
                    pathFeasible = false; // Set flag to indicate that the path is not feasible
                    break; // Exit the loop to restart the pathfinding process
                }
            }
            if (pathFeasible)
            {
                return myPath; // Return the path if it is feasible
            }
        }
    }




    // Not working correctly. It's just a draft. Would be nice to make it work. In progress...
    public List<PRMNode> FindPathInLazyIncrementalPRMusingAstarSearch(Dictionary<PRMNode, HashSet<PRMNode>> CC)
    {
        while (true) // Loop to restart the pathfinding process if needed
        {
            SimpleAStarAlgorithms aStar = new SimpleAStarAlgorithms(startNode.position, goalNode.position);
            Dictionary<PRMNode, PRMNode> came_from = aStar.SearchOnPRMGraph(graph, startNode, goalNode);

            if (came_from == null || came_from.Count == 0)
            {
                Debug.Log("No path found. Failure");
                return null;
            }

            PRMNode current = goalNode;
            List<PRMNode> myPath = new List<PRMNode>();
            while (current != startNode)
            {
                myPath.Add(current);
                if (!came_from.ContainsKey(current))
                {
                    Debug.Log("current: " + current.position);
                }
                current = came_from[current];
            }
            myPath.Add(startNode);
            myPath.Reverse();

            // Check each edge for visibility and collision
            bool pathFeasible = true; // Assume path is feasible until proven otherwise
            for (int i = 0; i < myPath.Count - 1; i++)
            {
                PRMNode nodeA = myPath[i];
                PRMNode nodeB = myPath[i + 1];

                // Slow! Takes more time to compute:
                // if (!VisibleRecurse(nodeA, nodeB))
                // {
                //     Debug.Log("Edge is not feasible. Deleting it from E and restarting pathfinding.");
                //     // Remove edge from graph (E)
                //     nodeA.RemoveNeighbourSymmetric(nodeB);
                //     pathFeasible = false; // Set flag to indicate that the path is not feasible
                //     break; // Exit the loop to restart the pathfinding process
                // }

                // Speed Improvement!: Takes less time to compute (aka is faster) because it uses the visibility cache:
                // Can use more nodes to sample when constructing the roadmap
                Debug.DrawLine(nodeA.position, nodeB.position, Color.green, 5f);
                if (!IsEdgeVisible(nodeA, nodeB))
                {
                    Debug.Log($"Edge ({nodeA.position}, {nodeB.position}) is not feasible. Removing...");
                    Debug.DrawLine(nodeA.position, nodeB.position, Color.red, 5f);
                    // Remove edge from graph and visibility cache
                    RemoveEdgeFromGraphAndCache(nodeA, nodeB);
                    pathFeasible = false; // Set flag to indicate that the path is not feasible
                    break; // Exit the loop to restart the pathfinding process

                    // We can also try to recompute the connected components here by calling MergeCC method again
                    // CC = MergeCC(nodeA, nodeB, CC);
                    // pathFeasible = false; // Set flag to indicate that the path is not feasible
                    // break; // Exit the loop to restart the pathfinding process
                }
            }
            if (pathFeasible)
            {
                return myPath; // Return the path if it is feasible
            }
        }
    }



    public List<PRMNode> GenerateBasicPRMalgorithmUsingKDtree()
    {
        List<PRMNode> V = new List<PRMNode> { startNode, goalNode };
        KDtree kdTree = CreateKDTreeFromPRMGraph();
        int n = 250; // number of nodes to generate
        Vector3 randomPosition;
        while (V.Count < n)
        {
            // Sample node (sampling a random position within the drivable space. Avoiding the obstacles.) (Sample() method)
            randomPosition = sampler.SamplePosition();
            if (UnityEngine.Random.value > 0.9) // bias distribution towards the goal, pick the goal point with a 0.9 probability
                randomPosition = goalNode.position;

            PRMNode randomNode = new PRMNode(randomPosition);
            if (!V.Contains(randomNode))
            {
                V.Add(randomNode);
                kdTree.Insert(randomNode.position);
            }
        }

        // use the KD tree to find the nearest neighbors of each node:
        foreach (PRMNode node in V)
        {
            List<PRMNode> nearestNodes = KDTreeNearestNeighbors(node.position, kdTree);
            foreach (PRMNode nearestNode in nearestNodes)
            {
                if (node == nearestNode) continue;
                if (!node.hasNeighbour(nearestNode) && VisibleRecurse(node, nearestNode))
                {
                    node.AddNeighbourSymmetric(nearestNode);
                    if (!graph.ContainsNode(node))
                    {
                        graph.AddNode(node);
                    }
                }
            }

            // KDtreeNode nearestNodeKD = kdTree.Search(node.position);
            // PRMNode nearestNode = new PRMNode(nearestNodeKD.position);

            // Alternative:
            // PRMNode nearestNode = KDTreeNearestNeighbor(node.position, kdTree);

            // if (node == nearestNode) continue;
            // if (!node.hasNeighbour(nearestNode) && VisibleRecurse(node, nearestNode))
            // {
            //     node.AddNeighbourSymmetric(nearestNode);
            //     if (!graph.ContainsNode(node))
            //     {
            //         graph.AddNode(node);
            //     }
            // }
        }

        // Find a path from start to goal using A* search:
        List<PRMNode> pathPRM = FindPathFromStartToGoalUsingAstar();
        return pathPRM;
    }



    // UTILITY TOOLS:

    // create KD tree from the PRM graph:
    public KDtree CreateKDTreeFromPRMGraph()
    {
        KDtree kdTree = new KDtree();
        foreach (PRMNode node in graph.GetNodes())
        {
            kdTree.Insert(node.position);
        }
        return kdTree;
    }

    public List<PRMNode> KDTreeNearestNeighbors(Vector3 position, KDtree kdTree)
    {
        Vector2 position2D = new Vector2(position.x, position.z);
        List<PRMNode> nearestNodes = new List<PRMNode>();
        return KDTreeNearestNeighborsRecurse(kdTree.getRoot(), position2D, 0, nearestNodes, float.MaxValue);
    }

    public List<PRMNode> KDTreeNearestNeighborsRecurse(KDtreeNode node, Vector2 position, int depth, List<PRMNode> nearestNodes, float bestDistance)
    {
        if (node == null)
        {
            return nearestNodes;
        }

        Vector3 position3D = new Vector3(position.x, 0, position.y);
        float distance = Vector3.Distance(node.position, position3D);
        if (distance < bestDistance)
        {
            nearestNodes.Add(new PRMNode(node.position));
            bestDistance = distance;
        }

        int currentDepth = depth % k; // k = 2

        if (position[currentDepth] < node.position[currentDepth])
        {
            nearestNodes = KDTreeNearestNeighborsRecurse(node.left, position, depth + 1, nearestNodes, bestDistance);
            if (position[currentDepth] + bestDistance >= node.position[currentDepth])
            {
                nearestNodes = KDTreeNearestNeighborsRecurse(node.right, position, depth + 1, nearestNodes, bestDistance);
            }
        }
        else
        {
            nearestNodes = KDTreeNearestNeighborsRecurse(node.right, position, depth + 1, nearestNodes, bestDistance);
            if (position[currentDepth] - bestDistance <= node.position[currentDepth])
            {
                nearestNodes = KDTreeNearestNeighborsRecurse(node.left, position, depth + 1, nearestNodes, bestDistance);
            }
        }

        return nearestNodes;
    }

    public PRMNode KDTreeNearestNeighbor(Vector3 position, KDtree kdTree)
    {
        Vector2 position2D = new Vector2(position.x, position.z);
        PRMNode nearestNode = KDTreeNearestNeighborRecurse(kdTree.getRoot(), position2D, 0, null, float.MaxValue);
        return nearestNode;
    }

    public PRMNode KDTreeNearestNeighborRecurse(KDtreeNode node, Vector2 position, int depth, PRMNode bestNode, float bestDistance)
    {
        if (node == null)
        {
            return bestNode;
        }

        Vector3 position3D = new Vector3(position.x, 0, position.y);
        float distance = Vector3.Distance(node.position, position3D);
        if (distance < bestDistance)
        {
            bestNode = new PRMNode(node.position);
            bestDistance = distance;
        }

        int currentDepth = depth % k; // k = 2

        if (position[currentDepth] < node.position[currentDepth])
        {
            bestNode = KDTreeNearestNeighborRecurse(node.left, position, depth + 1, bestNode, bestDistance);
            if (position[currentDepth] + bestDistance >= node.position[currentDepth])
            {
                bestNode = KDTreeNearestNeighborRecurse(node.right, position, depth + 1, bestNode, bestDistance);
            }
        }
        else
        {
            bestNode = KDTreeNearestNeighborRecurse(node.right, position, depth + 1, bestNode, bestDistance);
            if (position[currentDepth] - bestDistance <= node.position[currentDepth])
            {
                bestNode = KDTreeNearestNeighborRecurse(node.left, position, depth + 1, bestNode, bestDistance);
            }
        }

        return bestNode;
    }

    // a structure to represent a node in the KD tree:
    public class KDtreeNode
    {
        public Vector3 position; // to store k-dimensional point
        public KDtreeNode left;
        public KDtreeNode right;

        public KDtreeNode(Vector3 position)
        {
            this.position = position;
            this.left = null;
            this.right = null;
        }
    }

    public class KDtree
    {
        private int k = 2; // k-dimensional space
        KDtreeNode root;

        public KDtree()
        {
            root = null;
        }

        public KDtreeNode getRoot()
        {
            return root;
        }

        // a method to create a node of KD tree:
        public KDtreeNode newNode(Vector3 position)
        {
            KDtreeNode temp = new KDtreeNode(position);
            return temp;
        }

        // a utility method to determine if two points are same in K-dimensional space:
        public bool SamePoint(Vector3 position1, Vector3 position2)
        {
            return (position1 == position2);
        }

        public void Insert(Vector3 position)
        {
            Vector2 position2D = new Vector2(position.x, position.z);
            root = InsertRecurse(root, position2D, 0);
        }

        // Inserts a new node and returns root of modified tree
        // The parameter depth is used to decide axis of comparison
        private KDtreeNode InsertRecurse(KDtreeNode node, Vector2 position, int depth)
        {
            // Tree is empty?
            if (node == null)
            {
                Vector3 position3D = new Vector3(position.x, 0, position.y);
                return newNode(position3D); // new KDtreeNode(position);
            }

            // Calculate current dimension (cd) of comparison
            int currentDepth = depth % k;

            // Compare the new point with root on current dimension 'cd'
            // and decide the left or right subtree
            if (position[currentDepth] < node.position[currentDepth])
            {
                node.left = InsertRecurse(node.left, position, depth + 1);
            }
            else
            {
                node.right = InsertRecurse(node.right, position, depth + 1);
            }

            return node;
        }

        public KDtreeNode Search(Vector3 position)
        {
            Vector2 position2D = new Vector2(position.x, position.z);
            return SearchRecurse(root, position2D, 0);
        }

        private KDtreeNode SearchRecurse(KDtreeNode node, Vector2 position, int depth)
        {
            if (node == null)
            {
                return null;
            }

            Vector3 position3D = new Vector3(position.x, 0, position.y);
            if (node.position == position3D)
            {
                return node;
            }

            int currentDepth = depth % k;

            if (position[currentDepth] < node.position[currentDepth])
            {
                return SearchRecurse(node.left, position, depth + 1);
            }
            else
            {
                return SearchRecurse(node.right, position, depth + 1);
            }
        }

        // public void Delete(Vector3 position)
        // {
        //     root = DeleteRecurse(root, position, 0);
        // }

        // private KDtreeNode DeleteRecurse(KDtreeNode node, Vector3 position, int depth)
        // {
        //     if (node == null)
        //     {
        //         return null;
        //     }

        //     int currentDepth = depth % k;

        //     if (node.position == position)
        //     {
        //         if (node.left == null)
        //         {
        //             return node.right;
        //         }
        //         else if (node.right == null)
        //         {
        //             return node.left;
        //         }

        //         KDtreeNode minNode = FindMinNode(node.right, currentDepth);
        //         node.position = minNode.position;
        //         node.right = DeleteRecurse(node.right, minNode.position, depth + 1);
        //     }
        //     else if (position[currentDepth] < node.position[currentDepth])
        //     {
        //         node.left = Delete
        //     }

        //     return node;
        // }
    }

    // Depth First Search (DFS) algorithm to find all reachable nodes from a given node:
    public List<PRMNode> DFS(List<PRMNode> reachable, PRMNode node, Dictionary<PRMNode, bool> Gnode)
    {
        reachable.Add(node); // add node to reachable set
        Gnode[node] = true; // mark node as visited
        foreach (PRMNode neighbour in node.neighbours)
        {
            if (!Gnode.ContainsKey(neighbour)) // if neighbour is not visited
            {
                DFS(reachable, neighbour, Gnode);
            }
        }

        return reachable;
    }

    public List<PRMNode> FindPathFromStartToGoalUsingAstar()
    {
        // Use a pathfinding algorithm to find a path from start to goal
        // For example, you can use A* search
        SimpleAStarAlgorithms aStar = new SimpleAStarAlgorithms(startNode.position, goalNode.position);
        Dictionary<PRMNode, PRMNode> came_from = aStar.SearchOnPRMGraph(graph, startNode, goalNode);

        // With precomputed optimal heuristic: (The path seems to be more improved or the same, idk)
        // Dictionary<PRMNode, PRMNode> came_from = aStar.SearchOnLazyBasicPRMWithAstarOptimalHeuristic(graph, startNode, goalNode);

        if (came_from == null || came_from.Count == 0)
        {
            Debug.Log("No path found. Failure");
            return null;
        }

        PRMNode current = goalNode;
        List<PRMNode> myPath = new List<PRMNode>();
        while (current != startNode)
        {
            myPath.Add(current);
            // if (!came_from.ContainsKey(current))
            // {
            //     Debug.Log("current: " + current.position);
            // }
            current = came_from[current];
        }
        myPath.Add(startNode);
        myPath.Reverse();
        return myPath;
    }

    public bool CheckPathForCollision(List<PRMNode> path)
    {
        // Check each edge of the path for collision
        for (int i = 0; i < path.Count - 1; i++)
        {
            PRMNode nodeA = path[i];
            PRMNode nodeB = path[i + 1];

            if (!VisibleRecurse(nodeA, nodeB))
            {
                return false; // Path is not feasible due to collision
            }
        }
        return true; // Path is feasible
    }

    public Dictionary<PRMNode, HashSet<PRMNode>> MergeCC(PRMNode p, PRMNode q, Dictionary<PRMNode, HashSet<PRMNode>> CC)
    {
        if (CC[p].SetEquals(CC[q]))
        {
            return CC;
        }

        HashSet<PRMNode> mergedCC = new HashSet<PRMNode>();
        mergedCC.UnionWith(CC[p]);
        mergedCC.UnionWith(CC[q]);

        foreach (PRMNode node in mergedCC)
        {
            CC[node] = mergedCC;
        }

        return CC;
    }

    // disconnect the connected components (dismerge the connected components):
    public Dictionary<PRMNode, HashSet<PRMNode>> DismergeCC(PRMNode p, PRMNode q, Dictionary<PRMNode, HashSet<PRMNode>> CC)
    {
        if (!CC[p].SetEquals(CC[q]))
        {
            return CC;
        }

        // CC[p] should contains all its nodes except q:
        HashSet<PRMNode> dismergedCC = new HashSet<PRMNode>();
        dismergedCC.UnionWith(CC[p]);
        dismergedCC.Remove(q);

        // CC[q] should contains all its nodes except p:
        HashSet<PRMNode> dismergedCC2 = new HashSet<PRMNode>();
        dismergedCC2.UnionWith(CC[q]);
        dismergedCC2.Remove(p);

        foreach (PRMNode node in dismergedCC)
        {
            CC[node] = dismergedCC;
        }

        foreach (PRMNode node in dismergedCC2)
        {
            CC[node] = dismergedCC2;
        }

        return CC;
    }

    // private float Clearance(PRMNode node)
    // {
    //     int i = info.get_i_index(node.position.x);
    //     int j = info.get_j_index(node.position.z);
    //     // Assuming traversability[i, j] represents a lower bound on clearance
    //     return info.traversability[i, j];
    // }

    // // Exact dynamic collision checking method: (the project closes when this method is used)
    // private bool VisibleExact1(PRMNode nodeA, PRMNode nodeB)
    // {
    //     if (graph.GetDistance(nodeA.position, nodeB.position, 2) <= Clearance(nodeA) + Clearance(nodeB))
    //     {
    //         return true;
    //     }
    //     PRMNode m = new PRMNode((nodeA.position + nodeB.position) / 2);
    //     if (!Feasible(m))
    //     {
    //         return false;
    //     }
    //     return VisibleExact1(nodeA, m) && VisibleExact1(m, nodeB);
    // }

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
        if (graph.GetDistance(nodeA, nodeB, 2) < distEpsilon)
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

    public bool Feasible(PRMNode node)
    {
        int i = info.get_i_index(node.position.x);
        int j = info.get_j_index(node.position.z);
        // 0.0 = air/free, 1.0 = block/obstacle
        if (info.traversability[i, j] < 0.5f)
            return true; // free space
        return false; // obstacle
    }

    // Approximate dynamic collision checking method: (PRMNode version)
    private bool VisibleRecurse(PRMNode nodeA, PRMNode nodeB, float distEpsilon = 0.001f)
    {
        if (graph.GetDistance(nodeA.position, nodeB.position, 2) < distEpsilon)
        {
            return true;
        }
        PRMNode m = new PRMNode((nodeA.position + nodeB.position) / 2);
        if (!Feasible(m))
        {
            return false;
        }
        return VisibleRecurse(nodeA, m) && VisibleRecurse(m, nodeB);
    }

    // In this design (ie Lazy Basic PRM), it is helpful to cache which edges 
    // have been found to be visible to avoid re-checking edges in step 3.
    public class Edge
    {
        public PRMNode nodeA;
        public PRMNode nodeB;

        public Edge(PRMNode a, PRMNode b)
        {
            nodeA = a;
            nodeB = b;
        }

        // Override Equals and GetHashCode to allow using Edge as a key in a dictionary
        public override bool Equals(object obj)
        {
            Edge other = obj as Edge;
            return other != null && ((other.nodeA == nodeA && other.nodeB == nodeB) || (other.nodeA == nodeB && other.nodeB == nodeA));
        }

        public override int GetHashCode()
        {
            unchecked
            {
                int hash = 17;
                hash = hash * 23 + nodeA.GetHashCode();
                hash = hash * 23 + nodeB.GetHashCode();
                return hash;
            }
        }
    }

    // Dictionary to cache visibility of edges
    private Dictionary<Edge, bool> edgeVisibilityCache = new Dictionary<Edge, bool>();

    // Method to check if an edge is visible, and update the visibility cache
    private bool IsEdgeVisible(PRMNode nodeA, PRMNode nodeB)
    {
        // Check if visibility of the edge is already cached
        Edge edge = new Edge(nodeA, nodeB);
        if (edgeVisibilityCache.ContainsKey(edge))
        {
            return edgeVisibilityCache[edge];
        }

        // Calculate visibility and update cache
        bool isVisible = VisibleRecurse(nodeA, nodeB); // Your visibility check method
        edgeVisibilityCache[edge] = isVisible;
        return isVisible;
    }

    // Method to remove an edge from the graph and visibility cache
    private void RemoveEdgeFromGraphAndCache(PRMNode nodeA, PRMNode nodeB)
    {
        // Remove edge from graph
        nodeA.RemoveNeighbourSymmetric(nodeB);

        // Remove edge from visibility cache
        Edge edge = new Edge(nodeA, nodeB);
        if (edgeVisibilityCache.ContainsKey(edge))
        {
            edgeVisibilityCache.Remove(edge);
        }
    }

    private List<PRMNode> ClosestNeighborsOfNodeChosenFromVsetAccordingToDist(Vector3 position, List<PRMNode> V, float radius)
    {
        List<PRMNode> closestNeighbors = new List<PRMNode>();
        foreach (PRMNode node in V)
        {
            if (graph.GetDistance(node.position, position, 1) <= radius)
            {
                closestNeighbors.Add(node);
            }
        }
        return closestNeighbors;
    }

    private List<PRMNode> ListOfNodesInGeneratedNodeProximity(Vector3 position, float radius)
    {
        return graph.PRMGetNodesWithinRadius(position, radius);
    }

    private bool PRMObstacleFree(PRMNode nodeA, PRMNode nodeB)
    {
        RaycastHit hit;
        return !Physics.SphereCast(
            nodeA.position,
            carHeight,
            nodeB.position - nodeA.position,
            out hit,
            graph.GetDistance(nodeA.position, nodeB.position),
            layerMask: LayerMask.GetMask("Obstacles")
        );
    }

    private void SpawnDebugSphere(Vector3 pos, Color color)
    {
        GameObject nodeMarker = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        nodeMarker.transform.position = pos;
        nodeMarker.GetComponent<SphereCollider>().enabled = false;
        nodeMarker.GetComponent<Renderer>().material.color = color;
    }


}