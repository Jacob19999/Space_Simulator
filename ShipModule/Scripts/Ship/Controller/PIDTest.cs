using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PIDTest : MonoBehaviour
{

    PIDController pitchController;
    PIDController yawController;
    PIDController rollController;

    public float rcs_Pitch_Rate = 0f;
    public float rcs_Yaw_Rate = 0f;
    public float rcs_Roll_Rate = 0f;

    public float angularVelocityX = 0f;

    public float PitchOutput = 0f;

    public float minVal = -1f;
    public float maxVal = 1f;


    // Update is called once per frame
    void FixedUpdate()
    {
        pitchController.OutputMin = minVal;
        pitchController.OutputMax = maxVal;

        pitchController.SetPoint = rcs_Pitch_Rate;
        pitchController.ProcessVariable = angularVelocityX;

        PitchOutput = pitchController.ControlVariable(Time.fixedDeltaTime,  0.0001f);


    }
}
