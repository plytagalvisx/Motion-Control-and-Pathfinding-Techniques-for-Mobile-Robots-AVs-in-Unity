using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityStandardAssets.CrossPlatformInput;


public class DroneController : MonoBehaviour
{

    public Vector3 velocity;
    public Vector3 acceleration;

    public float max_speed = 15f;
    public float max_acceleration = 15f;

    private float v = 0f; //desired acceleration first component
    private float h = 0f; //desired acceleration second component

    public void Move(float h_in, float v_in)
    { 
        h = h_in;
        v = v_in;
    }

    // Start is called before the first frame update
    void Start()
    {
        velocity = Vector3.zero;
        acceleration = Vector3.zero;
    }

    // Update is called once per frame
    void Update()
    {

    }

    private void FixedUpdate()
    {
        // get input 
        //float h = CrossPlatformInputManager.GetAxis("Horizontal");
        //float v = CrossPlatformInputManager.GetAxis("Vertical");
        // UPDATE: will be set externally using Move(v,h) method

        //acceleration = transform.forward * v + transform.right * h * 1f;
        acceleration = (Vector3.right * h + Vector3.forward * v) * max_acceleration;
        if (acceleration.magnitude > max_acceleration)
        {
            acceleration = acceleration.normalized * max_acceleration;
        }

        velocity = velocity + acceleration * Time.fixedDeltaTime;
        if (velocity.magnitude > max_speed)
        {
            velocity = velocity.normalized * max_speed;
        }

        transform.position = transform.position + velocity * Time.fixedDeltaTime;

        var targetRotation = Quaternion.LookRotation(Vector3.up * 9.82f + acceleration, Vector3.forward);


        // Smoothly rotate towards the target direction
        transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, 10.0f * Time.fixedDeltaTime);
    }

    private void OnTriggerEnter(Collider other)
    {
        max_acceleration = max_acceleration / 2f;
        transform.position = transform.position - velocity * Time.fixedDeltaTime * 2f; // move back out of collision
        velocity = Vector3.zero;
        Debug.Log("Collision detected!");
    }
}
