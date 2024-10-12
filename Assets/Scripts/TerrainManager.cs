using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Newtonsoft.Json; // Import JSON.NET from Unity Asset store (not needed in 2022 version)


public class TerrainManager : MonoBehaviour
{


    //public TestScriptNoObject testNoObject = new TestScriptNoObject();

    public string terrain_filename = "Text/terrain";
    public TerrainInfo myInfo;

    public GameObject flag;


    // Use this for initialization
    void Start()
    {

    }

    // Use this for initialization
    void Awake()
    {

        var jsonTextFile = Resources.Load<TextAsset>(terrain_filename);

        // set to false for hard coding new terrain using TerrainInfo2 below
        bool loadFromFile = true;

        if (loadFromFile)
        {
            myInfo = TerrainInfo.CreateFromJSON(jsonTextFile.text);
        }
        else
        {
            myInfo.TerrainInfo2();
            //myInfo.file_name = "test88";
            string myString = myInfo.SaveToString();
            myInfo.WriteDataToFile(myString);
        }

        myInfo.CreateCubes();

        Instantiate(flag, myInfo.start_pos, Quaternion.identity);
        Instantiate(flag, myInfo.goal_pos, Quaternion.identity);



    }



    // Update is called once per frame
    void Update()
    {

    }
}



[System.Serializable]
public class TerrainInfo
{
    public string file_name;
    public float x_low;
    public float x_high;
    public int x_N;
    public float z_low;
    public float z_high;
    public int z_N;
    public float[,] traversability;

    public Vector3 start_pos;
    public Vector3 goal_pos;

    public float[,] GetTraversability()
    {
        return traversability;
    }

    /*
        This code is used to add an additional "padding" layer around the input traversability array 
        which is used to add an extra buffer around the obstacles in the traversability map.
        Returns the padded traversability array.
    */
    public float[,] GetPaddedTraversability()
    {
        int rows = traversability.GetLength(0);
        int cols = traversability.GetLength(1);
        float[,] paddedTraversability = new float[rows + 2, cols + 2];
        for (int i = 0; i < rows; i++)
        {
            for (int j = 0; j < cols; j++)
            {
                paddedTraversability[i + 1, j + 1] = traversability[i, j];
            }
        }
        return paddedTraversability;
    }

    public void TerrainInfo2()
    {
        file_name = "my_terrain";
        x_low = 50f;
        x_high = 250f;
        x_N = 45; // this is the number of grid cells in the x direction. 
        z_low = 50f;
        z_high = 250f;
        z_N = 7;

        start_pos = new Vector3(100f, 1f, 100f);
        goal_pos = new Vector3(110f, 1f, 110f);


        Debug.Log("Using hard coded info...");
        //traversability = new float[,] { { 1.1f, 2f }, { 3.3f, 4.4f } };
        traversability = new float[x_N, z_N]; // hardcoded now, needs to change
        for (int i = 0; i < x_N; i++)
        {
            for (int j = 0; j < z_N; j++)
            {
                if ((i == 0 || i == x_N - 1) || (j == 0 || j == z_N - 1))
                {
                    traversability[i, j] = 1.0f;
                }
                else
                {
                    traversability[i, j] = 0.0f;
                }
            }
        }
    }

    public float calculateGridCellSizeX()
    {
        return (x_high - x_low) / x_N; // this is the size of each grid cell in the x direction
    }

    public float calculateGridCellSizeZ()
    {
        return (z_high - z_low) / z_N;
    }

    public float calculateGridCellSize()
    {
        float x_step = calculateGridCellSizeX();
        float z_step = calculateGridCellSizeZ();
        return (x_step + z_step) / 2;
    }

    public int get_i_index(float x) // returns the discrete coordinate of the given continuous/world coordinate
    {
        int index = (int)Mathf.Floor(x_N * (x - x_low) / (x_high - x_low));
        if (index < 0)
        {
            index = 0;
        }
        else if (index > x_N - 1)
        {
            index = x_N - 1;
        }
        return index;

    }
    public int get_j_index(float z) // get index of given coordinate
    {
        int index = (int)Mathf.Floor(z_N * (z - z_low) / (z_high - z_low));
        if (index < 0)
        {
            index = 0;
        }
        else if (index > z_N - 1)
        {
            index = z_N - 1;
        }
        return index;
    }

    // private Vector3 getCenterOfCell(int i, int j, float xStep, float zStep) {
    //     return new Vector3(x_low + (i - 1 + 0.5f) * xStep, start_pos.y, z_low + (j - 1 + 0.5f) * zStep);
    // }

    public bool is_pos_clear(float x, float z)
    {
        int i = get_i_index(x);
        int j = get_j_index(z);
        return traversability[i, j] == 0;
    }

    public bool is_area_clear(List<Vector2> vertices)
    {
        foreach (Vector2 vertex in vertices)
        {
            if (!is_pos_clear(vertex.x, vertex.y))
            {
                return false;
            }
        }
        return true;
    }

    public float get_x_pos(int i) // returns the continuous/world coordinate of the given discrete coordinate
    {
        float step = (x_high - x_low) / x_N;
        return x_low + step / 2 + step * i;
    }

    public float get_z_pos(int j) // get position of given index
    {
        float step = (z_high - z_low) / z_N;
        return z_low + step / 2 + step * j;
    }

    public float get_x_step()
    {
        float step = (x_high - x_low) / x_N;
        return step;
    }

    public float get_z_step()
    {
        float step = (z_high - z_low) / z_N;
        return step;
    }

    // public void CreateCubes()
    // {
    //     float x_step = (x_high - x_low) / x_N;
    //     float z_step = (z_high - z_low) / z_N;
    //     for (int i = 0; i < x_N; i++)
    //     {
    //         for (int j = 0; j < z_N; j++)
    //         {
    //             if (traversability[i, j] > 0.5f) // obstacle
    //             {
    //                 GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
    //                 // GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Sphere);
    //                 cube.transform.position = new Vector3(get_x_pos(i), 0.0f, get_z_pos(j));
    //                 cube.transform.localScale = new Vector3(x_step, 15.0f, z_step);
    //                 cube.tag = "Box";
    //             }

    //         }
    //     }
    // }

    public List<Vector3> obstacleBarycenters { get; private set; } = new List<Vector3>();

    public List<PolygonalObstacle> polygonalObstacles = new List<PolygonalObstacle>();

    public List<Vector3> boundaryPoints = new List<Vector3>();
    public bool drawObstacleVertices = false;

    public void CreateCubes()
    {
        float x_step = (x_high - x_low) / x_N;
        float z_step = (z_high - z_low) / z_N;
        for (int i = 0; i < x_N; i++)
        {
            for (int j = 0; j < z_N; j++)
            {
                if (traversability[i, j] > 0.5f)
                {
                    GameObject cube = GameObject.CreatePrimitive(PrimitiveType.Cube);
                    cube.layer = 6;
                    // cube.tag = "Obstacle";
                    cube.GetComponent<Renderer>().material = Resources.Load("Obstacles", typeof(Material)) as Material;
                    cube.transform.parent = GameObject.Find("Obstacles").transform;
                    cube.transform.position = new Vector3(get_x_pos(i), 0.0f, get_z_pos(j));
                    cube.transform.localScale = new Vector3(x_step, 15.0f, z_step);

                    obstacleBarycenters.Add(GetBarycenter(cube));
                }

            }
        }

        float[,] tempTraversability = traversability.Clone() as float[,]; // Copy of the traversability map
        CreatePolygonalObstacles(tempTraversability);

        // Detect and remove duplicate vertices from the polygonal obstacle vertex list:
        for (int i = 0; i < polygonalObstacles.Count; i++)
        {
            PolygonalObstacle obstacle = polygonalObstacles[i];
            List<Vector3> uniqueVertices = new List<Vector3>();
            foreach (Vector3 vertex in obstacle.vertices)
            {
                if (!uniqueVertices.Contains(vertex))
                {
                    uniqueVertices.Add(vertex);
                }
            }
            polygonalObstacles[i].vertices = uniqueVertices;
        }

        float[,] tempTraversability2 = traversability.Clone() as float[,]; // Copy of the traversability map 
        ComputeEnvironmentBoundaryPoints(tempTraversability2);

        RemoveNonBoundaryVerticesOfObstacle(polygonalObstacles);

        drawObstacleVertices = true;

        // Shall we take care of the obstacle boundary points in the environment boundary area here?
    }

    // Find corner points of the polygonal obstacle where its edges intersect (critical points):
    // public List<Vector3> criticalPoints = new List<Vector3>();
    // private void FindCriticalPointsOfPolygonalObstacles()
    // {
    //     foreach (PolygonalObstacle obstacle in polygonalObstacles)
    //     {
    //         // List<Vector3> criticalPoints = new List<Vector3>();
    //         for (int i = 0; i < obstacle.vertices.Count; i++)
    //         {
    //             Vector3 v1 = obstacle.vertices[i];
    //             Vector3 v2 = obstacle.vertices[(i + 1) % obstacle.vertices.Count];
    //             for (int j = 0; j < obstacle.vertices.Count; j++)
    //             {
    //                 if (i == j) continue;
    //                 Vector3 v3 = obstacle.vertices[j];
    //                 Vector3 v4 = obstacle.vertices[(j + 1) % obstacle.vertices.Count];
    //                 Vector3 intersection = GetIntersectionPoint(v1, v2, v3, v4);
    //                 if (intersection != Vector3.zero)
    //                 {
    //                     criticalPoints.Add(intersection);
    //                 }
    //             }
    //         }
    //     }
    // }

    // // Get intersection point of two line segments:
    // private Vector3 GetIntersectionPoint(Vector3 p1, Vector3 p2, Vector3 p3, Vector3 p4)
    // {
    //     // Line segment 1:
    //     float x1 = p1.x;
    //     float y1 = p1.z;
    //     float x2 = p2.x;
    //     float y2 = p2.z;

    //     // Line segment 2:
    //     float x3 = p3.x;
    //     float y3 = p3.z;
    //     float x4 = p4.x;
    //     float y4 = p4.z;

    //     // Calculate the intersection point:
    //     float x = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) /
    //               ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));
    //     float z = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) /
    //               ((x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4));

    //     return new Vector3(x, 0, z);
    // }

    private void RemoveNonBoundaryVerticesOfObstacle(List<PolygonalObstacle> polygonalObstacles)
    {
        List<Vector3> verticesToRemove = new List<Vector3>();
        foreach (PolygonalObstacle obstacle in polygonalObstacles)
        {
            foreach (Vector3 vertex in obstacle.vertices)
            {
                // If a vertex's eight neighbours are all blocked, then it's not a boundary vertex
                // because it's surrounded by obstacles from all sides
                Vector3 vertexToRemove = CheckEightNeighboursTraversability(obstacle, vertex);
                if (vertexToRemove != Vector3.zero)
                {
                    verticesToRemove.Add(vertexToRemove);
                }
            }
        }

        // Remove the vertices that are not obstacle boundary vertices:
        foreach (PolygonalObstacle obstacle in polygonalObstacles)
        {
            foreach (Vector3 vertex in verticesToRemove)
            {
                if (obstacle.vertices.Contains(vertex))
                {
                    obstacle.vertices.Remove(vertex);
                }
            }
        }
    }

    private Vector3 CheckEightNeighboursTraversability(PolygonalObstacle obstacle, Vector3 vertex)
    {
        float halfXStep = (x_high - x_low) / x_N / 2.0f;
        float halfZStep = (z_high - z_low) / z_N / 2.0f;

        Vector3[] directions = {
            new Vector3(-halfXStep, 0, 0), // left
            new Vector3(halfXStep, 0, 0),  // right
            new Vector3(0, 0, -halfZStep), // down
            new Vector3(0, 0, halfZStep),  // up
            new Vector3(-halfXStep, 0, halfZStep),  // up-left
            new Vector3(halfXStep, 0, halfZStep),   // up-right
            new Vector3(-halfXStep, 0, -halfZStep), // down-left
            new Vector3(halfXStep, 0, -halfZStep)   // down-right
        };

        // Color[] colors = {
        //     Color.red,     // left
        //     Color.blue,    // right
        //     Color.green,   // down
        //     Color.yellow,  // up
        //     Color.magenta, // up-left
        //     Color.cyan,    // up-right
        //     Color.black,   // down-left
        //     Color.white    // down-right
        // };

        bool allBlocked = true;  // Assume all directions are blocked initially

        for (int i = 0; i < directions.Length; i++)
        {
            Vector3 adjacent = vertex + directions[i];
            int i_index = get_i_index(adjacent.x);
            int j_index = get_j_index(adjacent.z);

            // Check if traversable (less than or equal to 0.5f means traversable)
            if (traversability[i_index, j_index] < 0.5f) // Adjusted to check if traversable
            {
                // Debug.DrawLine(vertex, adjacent, colors[i], 100f);
                allBlocked = false;  // At least one direction is traversable, so it's okay
            }
        }

        // If all directions are blocked (not traversable), return the vertex to remove it
        if (allBlocked)
        {
            return vertex;
        }

        return Vector3.zero;
    }

    private bool IsAdjacent(int i1, int j1, int i2, int j2)
    {
        // Check if cubes are adjacent by their indices (horizontal or vertical)
        return (i1 == i2 && Mathf.Abs(j1 - j2) == 1) || (j1 == j2 && Mathf.Abs(i1 - i2) == 1);
    }

    public void CreatePolygonalObstacles(float[,] tempTraversability)
    {
        bool[,] visitedCubes = new bool[x_N, z_N]; // To keep track of visited cubes

        // Turn the boundary values from 1.0 to 0.0 in the traversability map:
        for (int i = 0; i < x_N; i++)
        {
            tempTraversability[0, i] = 0.0f; // Top row boundary
            tempTraversability[x_N - 1, i] = 0.0f; // Bottom row boundary
        }

        for (int j = 0; j < z_N; j++)
        {
            tempTraversability[j, 0] = 0.0f; // Left column boundary
            tempTraversability[j, z_N - 1] = 0.0f; // Right column boundary
        }

        for (int i = 1; i < x_N - 1; i++)
        {
            for (int j = 1; j < z_N - 1; j++)
            {
                if (!visitedCubes[i, j] && tempTraversability[i, j] > 0.5f) // if cube is an obstacle and not visited
                {
                    // Start a new polygon from this cube
                    PolygonalObstacle obstacle = new PolygonalObstacle();
                    ExploreObstacle(i, j, visitedCubes, obstacle, tempTraversability);
                    polygonalObstacles.Add(obstacle);
                }
            }
        }
    }

    public void ComputeEnvironmentBoundaryPoints(float[,] tempTraversability)
    {
        // Turn everything else except the boundary values to 0.0 in the traversability map:
        for (int i = 1; i < x_N - 1; i++)
        {
            for (int j = 1; j < z_N - 1; j++)
            {
                tempTraversability[i, j] = 0.0f; // Set all inner values to 0.0f
            }
        }

        // List<Vector3> boundaryPoints = new List<Vector3>();
        for (int i = 0; i < x_N; i++)
        {
            for (int j = 0; j < z_N; j++)
            {
                if (tempTraversability[i, j] > 0.5f)
                {
                    boundaryPoints.Add(new Vector3(get_x_pos(i), 0.0f, get_z_pos(j)));
                }
            }
        }
    }

    private void ExploreObstacle(int i, int j, bool[,] visitedCubes, PolygonalObstacle obstacle, float[,] tempTraversability)
    {
        // Stack-based or queue-based flood fill to explore all adjacent cubes
        Queue<Vector2Int> cubeToVisit = new Queue<Vector2Int>();
        cubeToVisit.Enqueue(new Vector2Int(i, j));

        while (cubeToVisit.Count > 0)
        {
            Vector2Int current = cubeToVisit.Dequeue();
            int x = current.x;
            int z = current.y;

            if (x < 0 || z < 0 || x >= x_N || z >= z_N || visitedCubes[x, z] || tempTraversability[x, z] <= 0.5f)
            {
                continue;
            }

            // Mark this cube as visited
            visitedCubes[x, z] = true;

            // Add this cube's vertices to the polygon obstacle
            AddCubeVerticesToPolygon(x, z, obstacle);

            // if (x == 0 || z == 0 || x == x_N - 1 || z == z_N - 1)
            // {
            //     continue; // Skip boundary cubes
            // }

            // Check for adjacent cubes (up, down, left, right)
            if (x > 0 && !visitedCubes[x - 1, z]) cubeToVisit.Enqueue(new Vector2Int(x - 1, z)); // Left
            if (x < x_N - 1 && !visitedCubes[x + 1, z]) cubeToVisit.Enqueue(new Vector2Int(x + 1, z)); // Right
            if (z > 0 && !visitedCubes[x, z - 1]) cubeToVisit.Enqueue(new Vector2Int(x, z - 1)); // Down
            if (z < z_N - 1 && !visitedCubes[x, z + 1]) cubeToVisit.Enqueue(new Vector2Int(x, z + 1)); // Up
        }
    }

    private void AddCubeVerticesToPolygon(int i, int j, PolygonalObstacle obstacle)
    {
        float x_pos = get_x_pos(i);
        float z_pos = get_z_pos(j);

        // Calculate the corner vertices of the cube
        float halfXStep = (x_high - x_low) / x_N / 2.0f;
        float halfZStep = (z_high - z_low) / z_N / 2.0f;

        // Cube vertices
        Vector3 v1 = new Vector3(x_pos - halfXStep, 0, z_pos - halfZStep); // Bottom-left
        Vector3 v2 = new Vector3(x_pos + halfXStep, 0, z_pos - halfZStep); // Bottom-right
        Vector3 v3 = new Vector3(x_pos + halfXStep, 0, z_pos + halfZStep); // Top-right
        Vector3 v4 = new Vector3(x_pos - halfXStep, 0, z_pos + halfZStep); // Top-left

        // Add vertices to the polygon obstacle
        obstacle.AddVertex(v1);
        obstacle.AddVertex(v2);
        obstacle.AddVertex(v3);
        obstacle.AddVertex(v4);
    }

    // Barycenter calculation function
    private Vector3 GetBarycenter(GameObject cube)
    {
        return cube.transform.position;
    }

    public List<Vector3> GetObstacleBarycenters()
    {
        return obstacleBarycenters;
    }



    public static TerrainInfo CreateFromJSON(string jsonString)
    {
        //Debug.Log("Reading json");
        return JsonConvert.DeserializeObject<TerrainInfo>(jsonString);
        //return JsonUtility.FromJson<TerrainInfo>(jsonString);
    }

    public string SaveToString()
    {
        JsonSerializerSettings JSS = new JsonSerializerSettings()
        {
            ReferenceLoopHandling = Newtonsoft.Json.ReferenceLoopHandling.Ignore
        };

        return JsonConvert.SerializeObject(this, Formatting.None, JSS);

        //return JsonConvert.SerializeObject(this); // throws ref loop error
        //return JsonUtility.ToJson(this); // cannot handle multi-dim arrays
    }

    public void WriteDataToFile(string jsonString)
    {
        string path = Application.dataPath + "/Resources/Text/" + file_name + ".json";
        Debug.Log("AssetPath:" + path);
        System.IO.File.WriteAllText(path, jsonString);
#if UNITY_EDITOR
        UnityEditor.AssetDatabase.Refresh();
#endif
    }


}

public class CubeObstacle
{
    public List<Vector3> eight_neighbours;
    public List<Vector3> corners;
    public Vector3 centroid;

    public CubeObstacle()
    {
        eight_neighbours = new List<Vector3>();
        corners = new List<Vector3>();

        // eight_neighbours.Add(new Vector3(1, 0, 0));
        // eight_neighbours.Add(new Vector3(-1, 0, 0));
        // eight_neighbours.Add(new Vector3(0, 0, 1));
        // eight_neighbours.Add(new Vector3(0, 0, -1));
        // eight_neighbours.Add(new Vector3(1, 0, 1));
        // eight_neighbours.Add(new Vector3(1, 0, -1));
        // eight_neighbours.Add(new Vector3(-1, 0, 1));
        // eight_neighbours.Add(new Vector3(-1, 0, -1));

        // corners.Add(new Vector3(1, 0, 1));
        // corners.Add(new Vector3(1, 0, -1));
        // corners.Add(new Vector3(-1, 0, 1));
        // corners.Add(new Vector3(-1, 0, -1));
    }

}

public class PolygonalObstacle
{
    public List<Vector3> vertices;

    public PolygonalObstacle()
    {
        vertices = new List<Vector3>();
    }

    public void AddVertex(Vector3 vertex)
    {
        vertices.Add(vertex);
    }
}
