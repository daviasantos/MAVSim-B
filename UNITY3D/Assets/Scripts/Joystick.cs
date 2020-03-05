using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Joystick : MonoBehaviour
{
    public GameObject text;
    public GameObject image;
    public GameObject image2;
    public GameObject image3;
    public GameObject painel;
    public GameObject text2;
    public GameObject text3;
    public GameObject exit;
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
            text2.gameObject.SetActive(!text2.gameObject.activeSelf);
            text3.gameObject.SetActive(!text3.gameObject.activeSelf);
            exit.gameObject.SetActive(!exit.gameObject.activeSelf);
            painel.gameObject.SetActive(!painel.gameObject.activeSelf);
            image.gameObject.SetActive(!image.gameObject.activeSelf);
            image2.gameObject.SetActive(!image2.gameObject.activeSelf);
            image3.gameObject.SetActive(!image3.gameObject.activeSelf);
        }
    }
}
