using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CamfollowDrone : MonoBehaviour
{
    public GameObject toFollow;     // Drone Position
    public GameObject camPos;       // Camera Position
    private float velocidade = 1.0f;

    // Update is called once per frame
    void LateUpdate()
    {
        // Change the rotation of the camera to follow the Drone 
        transform.position = Vector3.Lerp(transform.position, camPos.transform.position, velocidade * Time.deltaTime);
        transform.LookAt(toFollow.transform);
    }
}
