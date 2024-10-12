using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GameManager : MonoBehaviour {


    public GameObject terrain_manager_game_object;
    TerrainManager terrain_manager;

    public GameObject race_car;

    float start_time;
    public float completion_time;
    public float goal_tolerance = 10.0f;
    public bool finished = false;

    // Use this for initialization
    void Start () {

        terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
       
        start_time = Time.time;

        race_car.transform.position = terrain_manager.myInfo.start_pos + 3f* Vector3.up;
        race_car.transform.rotation = Quaternion.identity;


    }

    // Update is called once per frame
    void Update () {
        if(!finished)
        {
            if ((race_car.transform.position - terrain_manager.myInfo.goal_pos).magnitude < goal_tolerance)
            {
                completion_time = Time.time - start_time;
                finished = true;
            }
        }

	}
}
