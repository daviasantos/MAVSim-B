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
            // Show and Hide the Help Screen
            text.gameObject.SetActive(!text.gameObject.activeSelf);
            image.gameObject.SetActive(!image.gameObject.activeSelf);
            image2.gameObject.SetActive(!image2.gameObject.activeSelf);
            image3.gameObject.SetActive(!image3.gameObject.activeSelf);
        }
    }
}
