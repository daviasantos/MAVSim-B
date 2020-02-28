using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Hit : MonoBehaviour {

    private void OnCollisionEnter(Collision on){
        if (on.gameObject.name == "Cube")
        {
            Debug.Log("HIT DETECTED");
        }
    }
}
