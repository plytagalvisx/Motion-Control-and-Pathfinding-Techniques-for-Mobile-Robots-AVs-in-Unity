using UnityEngine.Assertions;
using Unity.Collections;
using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using System.Linq;
using static StateData;
using UnityEngine.Profiling;

public class PositionSampler
{
    private readonly TerrainInfo info;

    public PositionSampler(TerrainInfo info)
    {
        this.info = info;
    }

    // Samples a random position within the drivable space. Avoiding the blocks/obstacles.
    // Returns a random position within the x/z plane
    public Vector3 SamplePosition()
    {
        Vector3 randomlySampledPosition = new(Random.Range(info.x_low, info.x_high), 0, Random.Range(info.x_low, info.x_high));

        while (true)
        {
            // Make sure it's empty space
            if (!IsInsideObstacle(randomlySampledPosition))
                break;

            randomlySampledPosition = new Vector3(Random.Range(info.x_low, info.x_high), 0, Random.Range(info.x_low, info.x_high));
        }

        return randomlySampledPosition;
    }

    public List<Vector3> SamplePositions(int num_samples)
    {
        List<Vector3> positions = new List<Vector3>();

        for (int i = 0; i < num_samples; i++)
        {
            positions.Add(SamplePosition());
        }

        return positions;
    }

    // public Vector3 BiasedSamplePosition(float bias = 0.2f)
    // {
    //     if (Random.Range(0, 1) > bias)
    //     {
    //         return SamplePosition();
    //     }
    //     else
    //     {
    //         return info.goal_pos;
    //     }
    // }

    // Check whether the position is inside a block/obstacle or not.
    // Returns True if it is inside
    public bool IsInsideObstacle(Vector3 position)
    {
        int i = info.get_i_index(position.x);
        int j = info.get_j_index(position.z);

        // 0.0 = air/free, 1.0 = block/obstacle
        if (info.traversability[i, j] < 0.5f)
            return false; // not inside an obstacle

        return true; // inside an obstacle
    }
}
