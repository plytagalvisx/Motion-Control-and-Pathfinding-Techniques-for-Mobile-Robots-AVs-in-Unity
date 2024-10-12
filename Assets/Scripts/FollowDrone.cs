using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FollowDrone : MonoBehaviour
{
    public Transform target_object;

    public float above = 2f;

    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void Update()
    {
        transform.position = target_object.position + above * Vector3.up;

        var targetRotation = Quaternion.LookRotation(target_object.transform.position - transform.position);

        transform.rotation = targetRotation;
    }
}
