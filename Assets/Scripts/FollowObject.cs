using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FollowObject : MonoBehaviour
{

    public Transform target_object;

    public float behind = 6f;
    public float above = 2f;

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        transform.position = target_object.position - behind * target_object.forward + above * target_object.up;
        transform.rotation = target_object.rotation;
    }
}
