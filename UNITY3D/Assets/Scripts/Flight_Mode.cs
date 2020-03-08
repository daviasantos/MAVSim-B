using System;
using UnityEngine;
using UnityEngine.UI;

public class Flight_Mode : MonoBehaviour
{
    //public Transform drone;     
    public Text Logs;
    public GameObject Drone;    
    public DroneControl mode;   // Script 'DroneControl'
    public string Tag; 

    public void Start()
    {
        // Take the variable Mode contained in the Script 'DroneControl'
        mode = Drone.GetComponentInParent<DroneControl>();
    }

    // Update is called once per frame
    void Update()
    {

        // Verify and print the Flight Mode of the Drone Controller
        if(mode.Mode == 1f)
            Tag = "OFF";
        else if (mode.Mode == 2f)
            Tag = "INIT";
        else if (mode.Mode == 3f)
            Tag = "READY";
        else if (mode.Mode == 4f)
            Tag = "ARMED";
        else if (mode.Mode == 5f)
            Tag = "TAKEOFF";
        else if (mode.Mode == 6f)
            Tag = "MANUAL";
        else if (mode.Mode == 7f)
            Tag = "WAYPOINT";
        else if (mode.Mode == 8f)
            Tag = "LANDING";
        else if (mode.Mode > 8f || mode.Mode < 1f)
            Tag = "...";

        // Show in the Simulator screen: The Flight Mode selected and The current position of the Drone
        Logs.text = "MAVSim 1.0.0\nFlight Mode: " + Tag + "\nPosition: " + Drone.transform.position.ToString("0.00") + "m" + "\n\nPress 'Esc' or the button [?] to show or hide the Help Screen.\nPress the button [X] to stop the TCP connection.";
    }
}