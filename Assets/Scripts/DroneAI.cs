using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

[RequireComponent(typeof(DroneController))]
public class DroneAI : MonoBehaviour
{

    private DroneController m_Drone; // the car controller we want to use

    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;
    private List<Vector3> my_path = new List<Vector3>();


    public Rigidbody rigidBody;

    public bool isFinished = false;

    private float goalDistance;



    private float t;

    public Vector3 start_pos;
    public Vector3 goal_pos;

    private int pathpoint_index = 1;


    private void Start()
    {
        // get the drone controller
        m_Drone = GetComponent<DroneController>();
        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
        rigidBody = GetComponent<Rigidbody>();

        // Plan your path here
        // ...

        start_pos = terrain_manager.myInfo.start_pos;
        goal_pos = terrain_manager.myInfo.goal_pos;


        VisibilityGraph visibilityGraph = terrain_manager.GetComponent<VisibilityGraph>();
        my_path = visibilityGraph.GetShortestPlannedPathWayPoints();
        my_path = visibilityGraph.GetAugmentedPath(my_path);


        // Plot your path to see if it makes sense
        Vector3 old_wp = start_pos;
        foreach (var wp in my_path)
        {
            Debug.DrawLine(old_wp, wp, Color.red, 100f);
            old_wp = wp;
        }

        my_path = visibilityGraph.getSmoothedPath(my_path);

        // Draw the smoothed path
        old_wp = start_pos;
        foreach (var wp in my_path)
        {
            Debug.DrawLine(old_wp, wp, Color.green, 100f);
            old_wp = wp;
        }



    }

    private void FixedUpdate()
    {


        // if the car is within a certain distance of the next node, increase the next index by one
        // the distance is squared (e.g. 25 = 5m distance from car to point)
        /*if (Mathf.Pow(drone_pos[0] - my_path[next][0], 2) + Mathf.Pow(drone_pos[2] - my_path[next][2], 2) < 1)
        {
            if (next < my_path.Count - 1)
            {
                next++;
                nextPoint=true;
            }
        }
    
        transform.position = terrain_manager.myInfo.start_pos + new Vector3(0f, 0f, 0f);
        this.transform.rotation = Quaternion.LookRotation(my_path[1] - my_path[0]);  // TODO: Should we fix rotation to 0? Could be pointing wrong direction... should it be random?
        m_Drone.max_acceleration = 15;
            */


        t = Time.fixedDeltaTime;
        Vector3 drone_pos = transform.position;
        float min_dis = float.MaxValue;
        int min_idx = pathpoint_index;
        Vector3 next_pos = my_path[pathpoint_index];
        Vector3 acceleration = next_pos - drone_pos + rigidBody.velocity * (5f - rigidBody.velocity.magnitude) / t;


        if (goalDistance < 10)
        {
            isFinished = true;
            m_Drone.Move(0f, 0f);
        }
        else
        {
            // check the distance from the drone to the path point. if this distance is less than the min distance from a point, then we save this as our current pathpoint index.

            for (int idx = pathpoint_index; idx < my_path.Count; idx++)
            {
                float drone_pathpoint_dis = Vector3.Distance(my_path[idx], drone_pos);
                if (drone_pathpoint_dis < min_dis && idx - pathpoint_index <= 2)
                {
                    min_dis = drone_pathpoint_dis;
                    min_idx = idx;
                }
            }

            // update current index and next path point, draw a line from these 
            pathpoint_index = min_idx;
            next_pos = my_path[pathpoint_index];
            Debug.DrawLine(transform.position, my_path[pathpoint_index], Color.yellow);


            float dist_to_next = Vector3.Distance(drone_pos, next_pos);


            //while we still have points and pass the current one, move on to the next point
            if (dist_to_next < 5f && pathpoint_index + 1 < my_path.Count)
            {
                pathpoint_index += 1;
            }

            // look ahead and prepare for any sharp turns
            int sharp_turn = 0;
            if (pathpoint_index < my_path.Count - 6)
            {
                for (int lookahead_idx = 0; lookahead_idx < 7; lookahead_idx++)
                {
                    Vector3 start_vector = my_path[pathpoint_index + lookahead_idx] - my_path[pathpoint_index + lookahead_idx - 1];
                    Vector3 upcoming_vector = my_path[pathpoint_index + lookahead_idx + 1] - my_path[pathpoint_index + lookahead_idx];
                    Debug.Log("angle: " + Vector3.Angle(start_vector, upcoming_vector));
                    if (Vector3.Angle(start_vector, upcoming_vector) > 8f)
                    {
                        sharp_turn = 1;
                        break;
                    }
/*
                    if (Vector3.Angle(start_vector, upcoming_vector) > 8f)
                    {
                        sharp_turn = 2;
                        break;
                    }
                    */

                }
            }

            // drive slower in case of sharp turn

            if (sharp_turn == 1)
            {
                m_Drone.Move(0.1f * acceleration.x, 0.1f * acceleration.z);
            }
            else if (sharp_turn == 2)
            {
                m_Drone.Move(0f, 0f);
            }

            else
            {
                m_Drone.Move(acceleration.x, acceleration.z);
            }

        }
    }



    // Update is called once per frame
    void Update()
    {

        goalDistance = Vector3.Distance(transform.position, goal_pos);
        Debug.Log("goal distance: " + goalDistance);
        if (goalDistance < 10)
        {
            isFinished = true;
        }

    }
}