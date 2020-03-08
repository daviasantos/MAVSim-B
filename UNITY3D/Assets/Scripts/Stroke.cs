using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Stroke : MonoBehaviour {

    private void OnCollisionEnter(Collision on){
        if (on.gameObject.name == "Cube")
        {   
            // Show the collision moment 
            Debug.Log("Stroke DETECTED");
        }
    }
}
