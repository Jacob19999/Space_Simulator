using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

public class TestSetVeloicty : MonoBehaviour
{
    [Header("Objects")]
    public SpaceShip shipController;

    [Header("Object Stats")]
    public Vector3 worldVelocity = Vector3.zero;
    
    [Header("Set Test Parameters")]
    public Transform targetPoint;
    public float RCS_X_Translate = 0;
    public float RCS_Y_Translate = 0;
    public float RCS_Z_Translate = 0;

    // Update is called once per frame
    void FixedUpdate()
    {
        if (!shipController)
        {
            shipController = gameObject.GetComponent<SpaceShip>();
            shipController._controlMode = SpaceShip.controlMode.RATE;
            shipController._thrustingMode = SpaceShip.thrustingMode.RCS;
        }

        shipController.rcs_x_rate = RCS_X_Translate;
        shipController.rcs_y_rate = RCS_Y_Translate;
        shipController.rcs_z_rate = RCS_Z_Translate;

       
        //if (_inertiaTensor == null)
        //{
        //    _inertiaTensor = transform.GetComponent<InertiaTensor>();
        //    _inertiaTensor.worldVelocity = _inertiaTensor.worldVelocity + worldVelocity;
        //}

    }

    public void goToWP(){

        shipController._controlMode = SpaceShip.controlMode.WAYPOINT;

        if (targetPoint != null)
        {
            Debug.Log("Waypoint Go");
            shipController.goToPos(targetPoint, 0f);
        }


    }


}


[CustomEditor(typeof(TestSetVeloicty))]
public class TestSetVeloictyEditor : Editor
{
    public override void OnInspectorGUI()
    {

        DrawDefaultInspector();

        TestSetVeloicty _input = (TestSetVeloicty)target;

        if (GUILayout.Button("Go To Waypoint"))
        {
            _input.goToWP();

        }
    }
}

