using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Cam2Drone : MonoBehaviour
{
    public GameObject toFollow;     // Drone Position
    public GameObject camPos;
    private float velocidade = 1.0f;


    // Start is called before the first frame update
    void Start()
    {

    }

    // Update is called once per frame
    void LateUpdate()
    {
        transform.position = Vector3.Lerp(transform.position, camPos.transform.position, velocidade * Time.deltaTime);
        transform.LookAt(toFollow.transform);
    }
}
