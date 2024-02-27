using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ObjectRotate : MonoBehaviour
{
    public float SpinSpeed = 10;
    public SpinDir dir;
    public enum SpinDir
    {
        up,
        forward,
        right
    }

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        if (dir == SpinDir.up)
        {
            gameObject.transform.Rotate(Vector3.up * SpinSpeed * Time.fixedDeltaTime);
        }
        if (dir == SpinDir.forward)
        {
            gameObject.transform.Rotate(Vector3.forward * SpinSpeed * Time.fixedDeltaTime);
        }
        if (dir == SpinDir.right)
        {
            gameObject.transform.Rotate(Vector3.right * SpinSpeed * Time.fixedDeltaTime);
        }


    }
}
