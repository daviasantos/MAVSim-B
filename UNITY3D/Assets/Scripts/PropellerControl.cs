using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PropellerControl : MonoBehaviour
{
    float omegaMax = 10000.0f;
    float omega = 0;
    float omegaRate = 50.0f;

    public GameObject Drone;

    public DroneControl power;

    public void Start()
    {
        power = Drone.GetComponentInParent<DroneControl>();
    }

    // Update is called once per frame
    void Update()
    {

        if (omega < omegaMax)
        {
            omega += omegaRate;
            if (omega > omegaMax)
                omega = omegaMax;
        }

        if (power.Power == 1f)
            transform.Rotate(new Vector3(0.0f, 0.0f, omega) * Time.deltaTime);
        else
            transform.Rotate(new Vector3(0.0f, 0.0f, 0.0f) * Time.deltaTime);
    }
}
