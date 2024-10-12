using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class RotateObject : MonoBehaviour {

    public float angular_velocity = 4000f;
	// Use this for initialization
	void Start () {
		
	}
	
	// Update is called once per frame
	void Update () {
        //transform.Rotate(new Vector3(0, angular_velocity * Time.deltaTime, 0));
        transform.localEulerAngles += transform.up * angular_velocity * Time.deltaTime;
	}
}
