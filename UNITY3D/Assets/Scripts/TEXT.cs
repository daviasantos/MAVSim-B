using UnityEngine;
using UnityEngine.UI;

public class TEXT : MonoBehaviour
{
    public Transform dr;
    public Text t;
     
    // Update is called once per frame
    void Update()
    {
        t.text = dr.transform.position.ToString("0.00");
        
    }
}
