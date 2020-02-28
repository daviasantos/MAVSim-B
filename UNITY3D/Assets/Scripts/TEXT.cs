using System;
using UnityEngine;
using UnityEngine.UI;

public class TEXT : MonoBehaviour
{
    public Transform dr;
    public Text t;
    public GameObject Drone;
    public DroneControl mode;
    public string Tag;

    public void Start()
    { 
        mode = Drone.GetComponentInParent<DroneControl>();
    }

    // Update is called once per frame
    void Update()
    {

        // Verify the Flight Mode of the Drone Controller
        if(mode.Mode == 1f)
        {
            Tag = "OFF";
        }
        else if (mode.Mode == 2f)
        {
            Tag = "INIT";
        }
        else if (mode.Mode == 3f)
        {
            Tag = "READY";
        }
        else if (mode.Mode == 4f)
        {
            Tag = "ARMED";
        }
        else if (mode.Mode == 5f)
        {
            Tag = "TAKEOFF";
        }
        else if (mode.Mode == 6f)
        {
            Tag = "MANUAL";
        }
        else if (mode.Mode == 7f)
        {
            Tag = "WAYPOINT";
        }
        else if (mode.Mode == 8f)
        {
            Tag = "LANDING";
        }
        else if (mode.Mode > 8f || mode.Mode < 1f)
        {
            Tag = "...";
        }

        // Show in the Simulator screen: The Flight Mode selected and The current position of the Drone
        t.text = "Flight Mode: " + Tag + "\nPosition: " + dr.transform.position.ToString("0.00") + "m";
    }
}