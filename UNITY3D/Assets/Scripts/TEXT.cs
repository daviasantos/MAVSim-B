using System;
using UnityEngine;
using UnityEngine.UI;

public class TEXT : MonoBehaviour
{
    public Transform dr;
    public Text t;
    public DroneControl power;
    // Update is called once per frame
    void Update() => t.text = "Position: " + dr.transform.position.ToString("0.00") + "\nRotation: " + dr.transform.rotation.eulerAngles.ToString("0.00");
}
