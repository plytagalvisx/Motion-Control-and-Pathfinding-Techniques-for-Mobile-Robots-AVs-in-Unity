# Motion-Control-and-Pathfinding-Techniques-for-Mobile-Robots-AVs-in-Unity

This repository contains various classes of search algorithms, e.g.
Uninformed and Informed (Heuristic: using Euclidean distance) Search
Strategies and control strategies for autonomous vehicle AI navigation
within a Unity environment. The brief variety of search strategies the
repository includes are Breadth-First Search, Dijkstra's Algorithm,
Greedy Best-First Search, and A\* Algorithm, along with the ability to
visualize graphs, nodes, paths, and other elements like Voronoi diagrams
and Rapidly Exploring Random Trees (RRTs).

# Path Planning Algorithms:

## A-Star (A\*) Algorithm Variants:

- A\* uses a list of waypoints (a path) to guide the autonomous vehicle (carAI).
- A heuristic-based search algorithm that finds the shortest path from a start to a goal node by evaluating possible paths based on cost and estimated distance.
- ### A\* on Simple/Basic Constructed (Discrete) Graph:
  - This implementation of A\* operates on a simple graph structure, using the SimpleAStarAlgorithms class.
- ### A\* on Traversable (Discrete) Grid Map:
  - This uses A\* on a grid representation of the environment to navigate between start and goal positions.
- ### A\* on Inﬂated Obstacle C-Space:
  - A\* is applied to a space where obstacles have been inflated to ensure a collision-free path.
- ### A\* on Constructed Visibility Graph:
  - Utilizes a visibility graph to find the shortest path and smooth it for better navigation.

## Hybrid A\*:

- Combines features of A\* and continuous steering methods for navigating environments (in continuous spaces, suitable for vehicle dynamics), enhancing the vehicle's ability to follow paths smoothly.

## RRT (Rapidly-Exploring Random Tree):

- A sampling-based algorithm that incrementally builds a tree of feasible paths, exploring the state space efficiently to connect the start and goal configurations.
- The RRT algorithm is utilized for path planning, enabling the AI to efficiently explore and navigate through complex (continuous, dynamic space) environments, focusing on finding paths while considering the orientation of the vehicle.
- Parameters include RRTSearchAlgorithm, PositionSampler, and pathfinding through a tree structure (Node\<State>).

## RRT-Star (RRT\*):

- Efficiently explores the search space to find feasible paths in dynamic environments (refines paths to minimize cost, ensuring optimality in the resulting path through iterative improvement).
- An enhanced version of the RRT algorithm, RRT\* optimizes the path by exploring potential routes more thoroughly and improving upon the initial path found.

## RRT Connect Algorithm:

- Bidirectional search to connect start and goal using RRT (Rapidly-exploring Random Tree) methodology.
- This algorithm improves efficiency by connecting two trees simultaneously, facilitating faster pathfinding through the (continuous) environment.

## PRM (Probabilistic Roadmap) Algorithms:

- A method for pathﬁnding that samples points in the environment to build
  a roadmap for navigation.
- The PRM method creates a roadmap of potential paths through the environment, represented as PRMGraph and managed by the PRMPathPlanner.
- The algorithm leverages traversability array data to determine valid movement paths.
- ### Basic PRM:
  - Generates a roadmap and ﬁnds a path using A\* on the PRM.
    graph.
- ### Basic PRM with KD-tree:
  - Utilizes KD-tree nearest neighbor search for efficient pathfinding in high-dimensional spaces.
- ### Lazy Incremental PRM (Improved PRM):
  - An incremental version of PRM focusing on connected components for better performance. Uses an incremental approach and connected components for better performance.
- ### Visibility PRM:
  - Reduces node numbers for improved search space efficiency.
- ### Lazy Collision Checking:
  - Implements collision checking with visibility caching and optional heuristics.

## Visibility Graph:

- Constructs a visibility graph for finding paths based on the line of sight between nodes in the environment.

# Triangulation Techniques:

## Delaunay Triangulation:

- Incremental triangulation and edge-flipping methods to construct a Delaunay triangulation from a set of points in a 2D plane.
- Triangles are visualized and used for further pathfinding.

## Constrained Delaunay Triangulation:

- Constructs a triangulated mesh for effective pathfinding and obstacle representation given as constraint points.
- Extends Delaunay triangulation to accommodate constraints, ensuring that specific edges remain within the triangulation.

## Voronoi Diagram:

- Facilitates efficient (equidistant polygonal region partitioning) space partitioning to improve navigation and obstacle avoidance.
- A partitioning of a plane into regions based on the distance to a specific set of points, enabling efficient obstacle avoidance and pathfinding by identifying free spaces.

## Convex Hull Algorithm:

- Uses the Gift Wrapping (Jarvis March) algorithm to compute convex hulls around obstacle (constraint) points or random points in space.
- A method for finding the convex hull of a set of points, offering insights into shape analysis and boundary detection.

# Motion Planning Controllers:

## Pure Pursuit Controller:

- Implements a pure pursuit strategy to track a reference path, adjusting steering angle based on the position of the robot relative to the path.

## Stanley Controller:

- Enables precise path following for autonomous vehicles by calculating steering angles based on target trajectory.

## Proportional-Derivative (PD) Controller:

- Utilizes proportional and derivative gains to control robot motion towards a target position.
- A proportional-derivative controller is used to manage the vehicle's acceleration and steering based on the position and velocity errors relative to the target position.

# Terrain and Obstacle Management, and Post-processing:

## Terrain Management:

- The implementation integrates terrain analysis and polygonal obstacle generation along with (graph) boundary tracing via the TerrainManager and TerrainInfo classes, assessing traversability array to identify valid movement areas (collision-free areas).

## Path Smoothing:

- After generating paths, smoothing algorithms such as Moving Average Algorithm, Bézier Curves Interpolation, and Spline Interpolation are implemented to reduce sharp turns and improve the overall trajectory.

## Obstacle Detection/Avoidance Behavior:

- Incorporates obstacle detection and processing through centroids and corners.
- Triangulation around obstacle boundaries is addressed (specially for Voronoi Diagram).
- The AI checks for obstacles and adjusts its movement to avoid collisions.
- A raycast is also employed to check for nearby obstacles in front of the vehicle, allowing for real-time avoidance strategies.

This implementation combines several advanced algorithms to enhance AI navigation capabilities, demonstrating proficiency in artificial intelligence techniques for game development, computational geometry (pathfinding) and robotics (robot navigation).

# Suggested Enhancements/Possible Improvements

## 1. Dynamic Obstacle Handling:

- Introduce functionality to handle moving obstacles in the environment. This could involve re-planning paths in real time or during a continuous update loop.

## 2. Performance Optimization:

- Consider optimizing the search algorithms, especially if the number of nodes is large. Implementing spatial partitioning techniques (like quad-trees) could reduce search time. What about R-trees or K-D trees? To be researched.

## 3. User Interactivity:

- Allow users to adjust parameters of the algorithms (like the number of samples in PRM or the size of the RRT) through a UI for better experimentation.

## 4. Steering Behaviors Implementation:

- The steering behaviors mentioned could be implemented incrementally. Start with the simplest behaviors (e.g., Seek and Flee) and gradually build up to more complex behaviors.

## 5. Dynamic Obstacle Avoidance:

- Enhance obstacle avoidance by using dynamic raycasting or sensors that can detect moving objects, enabling the vehicle to react to changes in the environment more effectively.

## 6. Tuning Parameters:

- The various control parameters (like k_p, k_d, max_speed, etc.) should be tunable via the Unity Inspector, allowing for easier experimentation and fine-tuning of the vehicle's performance.

## 7. Integrate Unity NavMesh:

- If your environment is suitable, consider leveraging Unity's NavMesh system for more sophisticated pathfinding and navigation capabilities.

## 8. State Management:

- Introduce a state management system to handle different behaviors (e.g., chasing, avoiding obstacles, path following) cleanly. This could enhance readability and make it easier to add new behaviors.

## 9. Deep Neural Networks/Reinforcement Learning

## 10. Artiﬁcial Potential Field

## 11. Behaviour Tree

## 12. Cell Decomposition Methods

## 13. Dynamic Window Method

## 14. Optimization Algorithms/Techniques:

### - Genetic Algorithm

### - Ant Colony

### - Particle Swarm Optimization:

- it's a search algorithm based on stochastic
  and population-based adaptive optimization

More on path planning techniques:
https://www.sciencedirect.com/science/article/pii/S095741742300756X

This repository introduces a solid foundation for various autonomous
vehicle driving behaviors and pathﬁnding, as well as, motion control
strategies.
