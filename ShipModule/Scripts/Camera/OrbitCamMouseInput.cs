using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;

public class OrbitCamMouseInput : MonoBehaviour
{
    [SerializeField]
    private OrbitCam _cam;

    public float sensitivity = 30f;

    void FixedUpdate()
    {

        if (Input.GetMouseButton(1))
        {

            Vector3 mouseDelta = Vector3.zero;

            mouseDelta.x = Input.GetAxis("Mouse X") * sensitivity;
            mouseDelta.y = Input.GetAxis("Mouse Y") * sensitivity;

            //mouseDelta = commandModule.transform.TransformDirection(mouseDelta);

            // adjust to screen size
            Vector3 moveDelta = mouseDelta * (360f / Screen.height);

            _cam.Move(moveDelta.x, -moveDelta.y);
        } 



    }

}
