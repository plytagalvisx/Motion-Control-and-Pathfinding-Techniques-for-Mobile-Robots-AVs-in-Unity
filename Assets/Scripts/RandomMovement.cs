using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RandomMovement : MonoBehaviour {

    public float speed = 1.0f;
    public float range = 20.0f;
    private float time_of_last_near_collision;
    private float turning_time;

    //Random rnd = new Random();


    // Use this for initialization
    void Start () {
        //speed = 1.0f;
        time_of_last_near_collision = -Mathf.Infinity;
        turning_time = 1.0f;
		
	}
	
	// Update is called once per frame
	void Update () {
        RaycastHit hit;

        if (Time.time - time_of_last_near_collision < turning_time)
        {
            transform.eulerAngles += transform.up * Time.deltaTime * 100.0f;
        } else if (Physics.Raycast(transform.position, transform.TransformDirection(Vector3.forward), out hit, range))
        {
            Debug.DrawRay(transform.position, transform.TransformDirection(Vector3.forward) * hit.distance, Color.yellow);
            time_of_last_near_collision = Time.time;
            turning_time = Random.Range(0.5f, 2.0f);

        } else
        {
            transform.position += transform.forward * speed * Time.deltaTime;
            if (Random.Range(0.0f,1.0f) > 0.995f){
                time_of_last_near_collision = Time.time; // fake collision to induce turn
            }
        }
    }
}
