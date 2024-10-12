using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityStandardAssets.CrossPlatformInput;


[RequireComponent(typeof(DroneController))]
public class DroneUserControl : MonoBehaviour
{
    private DroneController m_Drone;

    private void Awake()
    {
        m_Drone = GetComponent<DroneController>();
    }


    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        
    }

    private void FixedUpdate()
    {
        float h = CrossPlatformInputManager.GetAxis("Horizontal");
        float v = CrossPlatformInputManager.GetAxis("Vertical");

        m_Drone.Move(h, v);
    }
}
