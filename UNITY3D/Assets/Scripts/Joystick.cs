using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Joystick : MonoBehaviour
{
    public GameObject text;
    public GameObject image;
    public GameObject image2;
    public GameObject image3;
    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (Input.GetKeyDown(KeyCode.Escape))
        {
            text.gameObject.SetActive(true);
            image.gameObject.SetActive(true);
            image2.gameObject.SetActive(true);
            image3.gameObject.SetActive(true);
        }
        else if (Input.GetKeyDown(KeyCode.Q))
        {
            text.gameObject.SetActive(false);
            image.gameObject.SetActive(false);
            image2.gameObject.SetActive(false);
            image3.gameObject.SetActive(false);
        }
    }
}
