using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MissileController : MonoBehaviour
{
    public GameObject leadIndicator;
    public GameObject velocityVectorIndicator;

    public InertiaTensorGeneric _inertiaTensor;
    public float speed = 0;

    [Header("Current States")]
    public Vector3 inertiaTensor;
    public Vector3 _angularVelocity;
    public Vector3 localAngularVelocity;
    public Vector3 _angularVelocityDeg;

    public Vector3 localVelocity;
    public Vector3 worldVelocity;

    public Vector3 desiredDirection = Vector3.zero;

    [Header("Desired Pitch Rates")]
    public float rcs_Pitch_Rate = 0f;
    public float rcs_Yaw_Rate = 0f;
    public float rcs_Roll_Rate = 0f;

    [Header("Pitch Contorller")]
    [Range(0, 20)]
    public float pitchProportional = 10f;
    [Range(0, 20)]
    public float pitchIntegral = 7f;
    [Range(0, 10)]
    public float pitchDerivative = 0.1f;

    [Header("Yaw Contorller")]
    [Range(0, 20)]
    public float yawProportional = 10f;
    [Range(0, 20)]
    public float yawIntegral = 7f;
    [Range(0, 10)]
    public float yawDerivative = .1f;

    [Header("Roll Contorller")]
    [Range(0, 20)]
    public float rollProportional = 10f;
    [Range(0, 20)]
    public float rollIntegral = 7f;
    [Range(0, 10)]
    public float rollDerivative = 0.1f;

    [Header("Throttle Outputs")]
    public float pitchThrottle = 0f;
    public float rollThrottle = 0f;
    public float yawThrottle = 0f;
    [Range(0, 1)]
    public float engineThrottle = 0f;

    //PID Controller class
    private PIDController pitchController;
    private PIDController yawController;
    private PIDController rollController;

    public bool pitchEnable = true;
    public bool yawEnable = true;
    public bool rollEnable = false;

    public float Controller_Max_Rate = 1f;

    public bool globalEnable = true;
    void Awake()
    {

        initPID();
    }


    private void initPID()
    {
        if (_inertiaTensor == null)
        {
            _inertiaTensor = transform.GetComponent<InertiaTensorGeneric>();
        }
        

        if (pitchEnable)
        {
            pitchController = new PIDController(1, -1);
            pitchController.setGains(pitchProportional, pitchIntegral, pitchDerivative);
        }

        if (rollEnable)
        {
            rollController = new PIDController(1, -1);
            rollController.setGains(rollProportional, rollIntegral, rollDerivative);
        }

        if (yawEnable)
        {
            yawController = new PIDController(1, -1);
            yawController.setGains(yawProportional, yawIntegral, yawDerivative);
        }



    }

 

    private static Vector3 quaternionPitchYawRoll(Quaternion q1, Quaternion q2)
    {

        Quaternion qd = Conjugate(q1) * q2;

        float _phi = Mathf.Atan2(2 * (qd.w * qd.x + qd.y * qd.z), 1 - 2 * (Mathf.Pow(qd.x, 2) + Mathf.Pow(qd.y, 2))) ;
        float _theta = Mathf.Asin(2 * (qd.w * qd.y - qd.z * qd.x));
        float _psi = Mathf.Atan2(2 * (qd.w * qd.z + qd.x * qd.y), 1 - 2 * (Mathf.Pow(qd.y, 2) + Mathf.Pow(qd.z, 2))) ;

        return new Vector3(_phi, _theta, _psi);

    }
    private static Quaternion Conjugate(Quaternion value)
    {
        Quaternion ans;

        ans.x = -value.x;
        ans.y = -value.y;
        ans.z = -value.z;
        ans.w = value.w;

        return ans;
    }

    public void pointToTargetQuaternion(Quaternion pointingDirection, float RATE, float sensitivity)
    {
        float maxOffbore = 45f;

        Quaternion pointingDirectionLocal = Quaternion.LookRotation(pointingDirection * Vector3.forward, transform.up);

        Vector3 result = quaternionPitchYawRoll(pointingDirectionLocal, transform.rotation);
        Vector3 offboresight = quaternionPitchYawRoll(pointingDirectionLocal, transform.rotation);

        if (Mathf.Abs(result.x * Mathf.Rad2Deg) < 5)
        {
            result.x = result.x * 5f;
        }
        if (Mathf.Abs(result.y * Mathf.Rad2Deg) < 5)
        {
            result.x = result.y * 5f;
        }

        //Debug.Log("deg = " + result * Mathf.Rad2Deg + " rad = " + result + " velocity vec off = " + offboresight * Mathf.Rad2Deg);

        if (Mathf.Abs(offboresight.x * Mathf.Rad2Deg) > maxOffbore)
        {
            Debug.Log("Clamp Pitch");
            rcs_Pitch_Rate = 0;
        }
        else
        {
            rcs_Pitch_Rate = Mathf.Clamp(result.x * sensitivity, -1, 1) * RATE;
        }

        if (Mathf.Abs(offboresight.y * Mathf.Rad2Deg) > maxOffbore)
        {
            rcs_Yaw_Rate = 0;
        }
        else
        {
            rcs_Yaw_Rate = Mathf.Clamp(result.y * sensitivity, -1, 1) * RATE;
        }

        if (Mathf.Abs(rcs_Pitch_Rate) < 0.001f)
        {
            rcs_Pitch_Rate = 0;
        }

        if (Mathf.Abs(rcs_Yaw_Rate) < 0.001f)
        {
            rcs_Yaw_Rate = 0;
        }

        //velocityVectorIndicator.transform.rotation = Quaternion.LookRotation(_inertiaTensor.worldVelocity, transform.up);
        //leadIndicator.transform.rotation = pointingDirectionLocal;

    }

    public void pointToTargetQuaternionVelocityVec(Quaternion pointingDirection, float RATE, float sensitivity)
    {
        float maxOffbore = 45f;

        Quaternion pointingDirectionLocal = Quaternion.LookRotation(pointingDirection * Vector3.forward, transform.up);
        Quaternion velocityDir =  Quaternion.LookRotation(_inertiaTensor.worldVelocity, transform.up);

        Vector3 result = quaternionPitchYawRoll(pointingDirectionLocal, velocityDir);
        Vector3 offboresight = quaternionPitchYawRoll(velocityDir, transform.rotation);

        if (Mathf.Abs(result.x * Mathf.Rad2Deg) < 5 )
        {
            result.x = result.x * 5f;
        }
        if (Mathf.Abs(result.y * Mathf.Rad2Deg) < 5)
        {
            result.x = result.y * 5f;
        }

        Debug.Log("deg = " + result * Mathf.Rad2Deg + " rad = " + result + " velocity vec off = " + offboresight * Mathf.Rad2Deg);

        if (Mathf.Abs(offboresight.x * Mathf.Rad2Deg) > maxOffbore)
        {
            Debug.Log("Clamp Pitch");
            rcs_Pitch_Rate = 0;
        } else
        {
            rcs_Pitch_Rate = Mathf.Clamp(result.x * sensitivity, -1, 1) * RATE;
        }

        if (Mathf.Abs(offboresight.y * Mathf.Rad2Deg) > maxOffbore)
        {
            rcs_Yaw_Rate = 0;
        } else
        {
            rcs_Yaw_Rate = Mathf.Clamp(result.y * sensitivity, -1, 1) * RATE;
        }

        if (Mathf.Abs(rcs_Pitch_Rate) < 0.001f)
        {
            rcs_Pitch_Rate = 0;
        }

        if (Mathf.Abs(rcs_Yaw_Rate) < 0.001f)
        {
            rcs_Yaw_Rate = 0;
        }

        //velocityVectorIndicator.transform.rotation = velocityDir;
        //leadIndicator.transform.rotation = pointingDirectionLocal;

    }
    float AngleSigned(float angle, float max)
    {
        return (angle > max / 2f) ? angle - max : angle;
    }
    public void pointToTarget(Vector3 pointingDirection, float RATE)
    {
        Quaternion pointingDirectionLocal = Quaternion.LookRotation(pointingDirection, transform.up);
        Quaternion velocityDir = Quaternion.LookRotation(_inertiaTensor.worldVelocity, transform.up);

        Vector3 rotationDirection = Vector3.RotateTowards(transform.forward, pointingDirection.normalized, 360, 0.00f);
        Quaternion targetRotation = Quaternion.LookRotation(rotationDirection);

        //Figure out the error for each asix
        float xAngleError = Mathf.DeltaAngle(transform.rotation.eulerAngles.x, targetRotation.eulerAngles.x);

        float yAngleError = Mathf.DeltaAngle(transform.rotation.eulerAngles.y, targetRotation.eulerAngles.y);

        //float zAngleError = Mathf.DeltaAngle(transform.rotation.eulerAngles.z, targetRotation.eulerAngles.z);

        float sensitivity = 5f;

        rcs_Yaw_Rate = Mathf.Clamp(-yAngleError * Mathf.Deg2Rad * sensitivity, -1, 1) * RATE;
        rcs_Pitch_Rate = Mathf.Clamp(-xAngleError * Mathf.Deg2Rad * sensitivity, -1, 1) * RATE;

        if (Mathf.Abs(rcs_Pitch_Rate) < 0.001f)
        {
            rcs_Pitch_Rate = 0;
        }

        if (Mathf.Abs(rcs_Yaw_Rate) < 0.001f)
        {
            rcs_Yaw_Rate = 0;
        }

        //velocityVectorIndicator.transform.rotation = velocityDir;
        //leadIndicator.transform.rotation = pointingDirectionLocal;

    }

    public void pointToTargetVelocityVector(Vector3 pointingDirection, float RATE)
    {

        Vector3 gotoPos = transform.position - (pointingDirection.normalized * 500f);
        Vector3 goToPosVector = (gotoPos - transform.position).normalized;

        Vector3 rotationDirection = Vector3.RotateTowards( _inertiaTensor.worldVelocity.normalized, goToPosVector, 360, 0.00f);

        rotationDirection = transform.InverseTransformDirection(rotationDirection);

        float sensitivity = 40f;
        rcs_Yaw_Rate = Mathf.Clamp(rotationDirection.x * sensitivity, -1, 1) * RATE;
        rcs_Pitch_Rate = -Mathf.Clamp(rotationDirection.y * sensitivity, -1, 1) * RATE;


        if (Mathf.Abs(rcs_Pitch_Rate) < 0.001f)
        {
            rcs_Pitch_Rate = 0;
        }

        if (Mathf.Abs(rcs_Yaw_Rate) < 0.001f)
        {
            rcs_Yaw_Rate = 0;
        }

        //velocityVectorIndicator.transform.forward = _inertiaTensor.worldVelocity.normalized;
        //leadIndicator.transform.forward = goToPosVector;


    }

    float AngleBetween(Vector3 reference, Vector3 target)
    {
        float angle = Vector3.Angle(reference, target);
        float sign = Mathf.Sign(Vector3.Dot(Vector3.Cross(reference, target), Vector3.up));
        return angle * sign;
    }

    void FixedUpdate()
    {

        
        if (globalEnable)
        {
            // Set up PID system.
            if (pitchController == null || yawController == null || rollController == null)
            {
                initPID();
            }

            // Update Angular Velocity from Inertia Tensor
            inertiaTensor = _inertiaTensor.VehicleInertiaTensorVector;
            localAngularVelocity = transform.InverseTransformDirection(_inertiaTensor.AngularVelocity);
            _angularVelocityDeg = _angularVelocity;

            yawController.setGains(yawProportional, yawIntegral, yawDerivative);
            pitchController.setGains(pitchProportional, pitchIntegral, pitchDerivative);

            // Rotational PID
            if (pitchEnable)
            {
                PitchPID();
            }

            if (rollEnable)
            {
                RowPID();
            }

            if (yawEnable)
            {
                YawPID();
            }

            mainEngines();
            Rotational();

            foreach (MainEngine thisEngine in _inertiaTensor.ProgradeEngines)
            {
                thisEngine.throttle = Mathf.Abs(engineThrottle);
            }

        } else
        {


        }

    }


    public void setThrottleZero()
    {

        foreach (MainEngine thisEngine in _inertiaTensor.ProgradeEngines)
        {
            thisEngine.throttle = 0;
        }
        foreach (Thruster thisThruster in _inertiaTensor.Thrusters)
        {
            thisThruster.throttlePersistant = 0;
        }

    }


    private void PitchPID()
    {
        float throttle = 0;

        if (localAngularVelocity.x == 0)
        {
            localAngularVelocity.x = 0.0000001f;
        }

        pitchController.SetPoint = rcs_Pitch_Rate;
        pitchController.ProcessVariable = localAngularVelocity.x;
        throttle = pitchController.ControlVariable(Time.fixedDeltaTime, 0.0001f);
        pitchThrottle = throttle;


        if (throttle > 0f)
        {
            foreach (Thruster thisThruster in _inertiaTensor.Pitch_Up)
            {
                thisThruster.throttlePersistant = Mathf.Abs(throttle);
            }
        }

        if (throttle < 0f)
        {
            foreach (Thruster thisThruster in _inertiaTensor.Pitch_Dn)
            {
                thisThruster.throttlePersistant = Mathf.Abs(throttle);
            }
        }


    }

    private void mainEngines()
    {
        // Addforce for main engines
        if (engineThrottle > 0f)
        {
            foreach (MainEngine thisEngine in _inertiaTensor.ProgradeEngines)
            {
               // Debug.Log(thisEngine.name + " " + engineThrottle);
                _inertiaTensor.addForceAtPosMainEngine(thisEngine, thisEngine.thrustN * Mathf.Abs(engineThrottle), Time.fixedDeltaTime);
                thisEngine.throttle = Mathf.Abs(engineThrottle);
            }
        }

        if (engineThrottle < 0f)
        {
            foreach (MainEngine thisEngine in _inertiaTensor.RetroEngines)
            {
                _inertiaTensor.addForceAtPosMainEngine(thisEngine, thisEngine.thrustN * Mathf.Abs(engineThrottle), Time.fixedDeltaTime);
                thisEngine.throttle = Mathf.Abs(engineThrottle);
            }
        }
    }

    public Vector3 getCurrentAcceleration()
    {
        return _inertiaTensor.worldAccelVec * engineThrottle;
    }

    public Vector3 getCurrentVelocity()
    {
        return _inertiaTensor.worldVelocity;
    }


    private void YawPID()
    {
        float throttle = 0;

        if (localAngularVelocity.y == 0)
        {
            localAngularVelocity.y = 0.0000001f;
        }

        yawController.SetPoint = rcs_Yaw_Rate;
        yawController.ProcessVariable = localAngularVelocity.y;
        throttle = yawController.ControlVariable(Time.fixedDeltaTime, 0.0001f);

        yawThrottle = throttle;

        if (throttle > 0f)
        {
            foreach (Thruster thisThruster in _inertiaTensor.Yaw_Left)
            {
                thisThruster.throttlePersistant = Mathf.Abs(throttle);

            }
        }

        if (throttle < 0f)
        {
            foreach (Thruster thisThruster in _inertiaTensor.Yaw_Right)
            {
                thisThruster.throttlePersistant = Mathf.Abs(throttle);

            }
        }


    }

    private void RowPID()
    {
        float throttle = 0;

        if (localAngularVelocity.z == 0)
        {
            localAngularVelocity.z = 0.0000001f;
        }

        rollController.SetPoint = rcs_Roll_Rate;
        rollController.ProcessVariable = localAngularVelocity.z;
        throttle = rollController.ControlVariable(Time.fixedDeltaTime, 0.0001f);
        rollThrottle = throttle;

        if (throttle < 0f)
        {
            foreach (Thruster thisThruster in _inertiaTensor.Roll_Left)
            {
                thisThruster.throttlePersistant = Mathf.Abs(throttle);

            }
        }

        if (throttle > 0f)
        {
            foreach (Thruster thisThruster in _inertiaTensor.Roll_Right)
            {
                thisThruster.throttlePersistant = Mathf.Abs(throttle);

            }
        }

    }


    private void Rotational()
    {

        // Overall addforce for RCS thruters
        foreach (Thruster thisThruster in _inertiaTensor.Thrusters)
        {

            float throttle = thisThruster.getFireThrottle();

            if (throttle > 0)
            {
                _inertiaTensor.addForceAtPos(thisThruster, thisThruster.thrustN * Mathf.Abs(throttle), Time.fixedDeltaTime);
            }
            thisThruster.throttlePersistant = 0;

            // For drawing particle system.
            thisThruster.throttlePersistant = throttle;
        }

    }

}
