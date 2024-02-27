using System.Collections;
using System.Collections.Generic;
using System.IO.Compression;
using System.Runtime.InteropServices;
using Unity.VisualScripting;
using UnityEngine;
using static UnityEditor.ShaderGraph.Internal.KeywordDependentCollection;

public class SpaceShip : MonoBehaviour
{
    [Header("State")]
    public bool globalEnable = true;

    [Header("Objects")]
    [SerializeField]
    public GameObject parentGameObject;
    private InertiaTensor _inertiaTensor;
    private ShipInput _shipInput;

    [Header("Setting")]
    public float maxRotationRate = 360f;
    public controlMode _controlMode;
    public thrustingMode _thrustingMode;

    [Header("Current States")]
    public Vector3 inertiaTensor;
    public Vector3 _angularVelocity;
    public Vector3 localAngularVelocity;
    public Vector3 _angularVelocityDeg;
    public float engineThrottle = 0f;

    public Vector3 localVelocity;
    public Vector3 worldVelocity;

    [Header("Desired Pitch Rates")]
    public float rcs_Pitch_Rate = 0f;
    public float rcs_Yaw_Rate = 0f;
    public float rcs_Roll_Rate = 0f;

    [Header("Desired Translational Rates")]
    public float rcs_z_rate = 0f;
    public float rcs_x_rate = 0f;
    public float rcs_y_rate = 0f;

    [Header("PID Contorllers")]
    public bool changeGains = false;

    [Header("Pitch Contorller")]
    [Range(0, 20)]
    public float pitchProportional = 10f;
    [Range(0, 20)]
    public float pitchIntegral = 7f;
    [Range(0, 10)]
    public float pitchDerivative = 0.1f;

    [Header("Roll Contorller")]
    [Range(0, 20)]
    public float rollProportional = 10f;
    [Range(0, 20)]
    public float rollIntegral = 7f;
    [Range(0, 10)]
    public float rollDerivative = 0.1f;

    [Header("Yaw Contorller")]
    [Range(0, 20)]
    public float yawProportional = 10f;
    [Range(0, 20)]
    public float yawIntegral = 7f;
    [Range(0, 10)]
    public float yawDerivative = .1f;

    [Header("Translate X Contorller")]
    [Range(0, 20)]
    public float x_Proportional = 10f;
    [Range(0, 20)]
    public float x_Integral = 7f;
    [Range(0, 10)]
    public float x_Derivative = 0.1f;

    [Header("Translate Y Contorller")]
    [Range(0, 20)]
    public float y_Proportional = 10f;
    [Range(0, 20)]
    public float y_Integral = 7f;
    [Range(0, 10)]
    public float y_Derivative = 0.1f;

    [Header("Translate Z Contorller")]
    [Range(0, 20)]
    public float z_Proportional = 10f;
    [Range(0, 20)]
    public float z_Integral = 7f;
    [Range(0, 10)]
    public float z_Derivative = .1f;

    [Header("Throttle Outputs")]
    public float pitchThrottle = 0f;
    public float rollThrottle = 0f;
    public float yawThrottle = 0f;

    [Header("Throttle Outputs")]
    public float x_Throttle = 0f;
    public float y_Throttle = 0f;
    public float z_Throttle = 0f;

    [Header("Error Term")]
    public float pitchError = 0f;
    public float rollError = 0f;
    public float yawError = 0f;

    [Header("Rate Averaged Differece")]
    public bool dampeningEnable = true;
    public Vector3 averageSetting = Vector3.zero;
    [Range(0.001f, 1.000f)]
    public float treashold = 0.01f;


    //PID Controller class
    private PIDController pitchController;
    private PIDController yawController;
    private PIDController rollController;

    private PIDController x_RateController;
    private PIDController y_RateController;
    private PIDController z_RateController;

    // For dampening (Not used)
    private List<Vector3> angularVelocityHistory = new List<Vector3>();
    private List<Vector3> thrustSettingHistory = new List<Vector3>();

    private int historyTracker = 0;
    private int historyLen = 50;

    // Waypoint Navigation
    [Header("Waypoint Navigation Settings")]
    public bool moveToNewWp = false;

    private bool newWaypoint = false;
    private bool arrivedWp = false;
    private Transform goToWaypoint;
    private float navGoToVelocity = 0;
    private float initialDistance = 0f;
    private float decelDistance = 0;


    public enum thrustingMode
    {
        mainEngines,
        RCS
    }

    public enum controlMode
    {
        NORMAL,
        RATE,
        OFF,
        MANUAL,
        WAYPOINT
    }


    public void initPID()
    {
        parentGameObject = gameObject;
        _inertiaTensor = gameObject.GetComponent<InertiaTensor>();
        _shipInput = gameObject.GetComponent<ShipInput>();

        pitchController = new PIDController(1, -1);
        yawController = new PIDController(1, -1);
        rollController = new PIDController(1, -1);

        x_RateController = new PIDController(1, -1);
        y_RateController = new PIDController(1, -1);
        z_RateController = new PIDController(1, -1);

        pitchController.setGains(pitchProportional, pitchIntegral, pitchDerivative);
        yawController.setGains(yawProportional, yawIntegral, yawDerivative);
        rollController.setGains(rollProportional, rollIntegral, rollDerivative);

        x_RateController.setGains(x_Proportional, x_Integral, x_Derivative);
        y_RateController.setGains(y_Proportional, y_Integral, y_Derivative);
        z_RateController.setGains(z_Proportional, z_Integral, z_Derivative);

    }


    void Awake()
    {
        if(pitchController== null || yawController== null || rollController== null)
        {
            initPID();
        }
        

    }

    void FixedUpdate()
    {

        Vector3 initialVelocity = Vector3.zero;

        if (_inertiaTensor.integrateOrbit)
        {
            initialVelocity = _inertiaTensor.worldVelocity;

        }


        // Set up PID system.
        if (pitchController == null || yawController == null || rollController == null)
        {
            initPID();
        }

        if (changeGains)
        {
            pitchController.setGains(pitchProportional, pitchIntegral, pitchDerivative);
            yawController.setGains(yawProportional, yawIntegral, yawDerivative);
            rollController.setGains(rollProportional, rollIntegral, rollDerivative);

            x_RateController.setGains(x_Proportional, x_Integral, x_Derivative);
            y_RateController.setGains(y_Proportional, y_Integral, y_Derivative);
            z_RateController.setGains(z_Proportional, z_Integral, z_Derivative);
        }
        


        // Editor Settings
        if (dampeningEnable)
        {
            dampOsc();
            historyTracker += 1;
        }

        if (globalEnable == false)
        {
            return;
        }

        ////////////////////////////////////////////////////////////////////////////////////////////////

        // Update Angular Velocity from Inertia Tensor
        inertiaTensor = _inertiaTensor.VehicleInertiaTensorVector;
        localAngularVelocity = parentGameObject.transform.InverseTransformDirection(_inertiaTensor.AngularVelocity);
        _angularVelocityDeg = _angularVelocity;

        // Update Velocity from Inertia Tensor
        localVelocity = _inertiaTensor.localVelocity;
        worldVelocity = _inertiaTensor.worldVelocity;

        // Rotational PID
        RowPID();
        PitchPID();
        YawPID();

        if (_thrustingMode == thrustingMode.RCS)
        {
            if (_controlMode != controlMode.OFF)
            {

                // Translational PID

                if (_controlMode != controlMode.MANUAL)
                {
                    translate_Z_PID();
                    translate_Y_PID();
                    translate_X_PID();
                }

                translationalThrottle();
            }
            
        } 


        if (_thrustingMode == thrustingMode.mainEngines)
        {
            mainEngines();
        }

        if (_controlMode == controlMode.WAYPOINT)
        {
            if (moveToNewWp)
            {
                navGoToPosition();
            }
        }

        if (_controlMode != controlMode.WAYPOINT)
        {
            moveToNewWp = false;
        }

        addForcesAll();


        // Calculate the change in delta v
        if (_inertiaTensor.integrateOrbit)
        {
            _inertiaTensor.inertiaTensorUpdate(Time.fixedDeltaTime);
            Vector3 deltaV = _inertiaTensor.worldVelocity - initialVelocity;

            _inertiaTensor.orbitAncherOrbit.helioCentricVelocity += Vector3d.toVector3d( deltaV);

            //Debug.Log(deltaV);

        }

    }

    public void goToPos(Transform gotoPosTransform, float goToVelocity)
    {

        float thrustTranslateAft = 0f;
        float thrustTranslateFwd = 0f;
        float AccelDistance = 0f;
        goToWaypoint = gotoPosTransform;

        // Initial Distance
        initialDistance = Vector3.Distance(transform.position, goToWaypoint.position);

        foreach (Thruster thisThruster in _inertiaTensor.translate_Back)
        {
            thrustTranslateAft = thrustTranslateAft + thisThruster.thrustN;
        }

        // Get deceleration distance
        decelDistance = (0 - Mathf.Pow(goToVelocity, 2)) / (thrustTranslateAft / _inertiaTensor.compositeMass);

        // If not defined any velocity
        if (goToVelocity == 0)
        {
            foreach (Thruster thisThruster in _inertiaTensor.translate_Fwd)
            {
                thrustTranslateFwd = thrustTranslateFwd + thisThruster.thrustN;
            }

            decelDistance = (initialDistance / (thrustTranslateFwd + thrustTranslateAft)) * thrustTranslateAft;
            AccelDistance = (initialDistance / (thrustTranslateFwd + thrustTranslateAft)) * thrustTranslateFwd;

            goToVelocity = Mathf.Sqrt(2 * (thrustTranslateFwd / _inertiaTensor.compositeMass) * AccelDistance);
        }

        Debug.Log("Init Accel Dist = " + AccelDistance+  " Accel Dist = " +  decelDistance + " Vel = " + goToVelocity);
        moveToNewWp = true;
        navGoToVelocity = goToVelocity;


    }

    public void navGoToPosition()
    {

        float currentDistance = Vector3.Distance(transform.position, goToWaypoint.transform.position);

        if (navGoToVelocity < 1000)
        {
            if (currentDistance > Mathf.Abs(decelDistance))
            {
                rcs_z_rate = navGoToVelocity;

            } else if (currentDistance < Mathf.Abs(decelDistance))
            {
                Debug.Log("Decel");
                rcs_z_rate = navGoToVelocity * (currentDistance / Mathf.Abs(decelDistance));

            }

            if (currentDistance < 3f)
            {
                rcs_z_rate = 0;
                moveToNewWp = false;
                Debug.Log("Arrived");
            } else
            {
                pointToTarget((goToWaypoint.transform.position - transform.position).normalized, .75f);
            }

            // Calculate deceleration time needed.
            Debug.Log("Distance = " + currentDistance + " Decel Dist = " + decelDistance + " velocity = " + navGoToVelocity);
            
        }

        if (_shipInput)
        {
            _shipInput.rcs_x_rate = rcs_x_rate;
            _shipInput.rcs_y_rate = rcs_y_rate;
            _shipInput.rcs_z_rate = rcs_z_rate;

            _shipInput.Pitch_rate = rcs_Pitch_Rate;
            _shipInput.Roll_rate = rcs_Roll_Rate;
            _shipInput.Yaw_rate = rcs_Yaw_Rate;

        }

    }

    private static Vector3 quaternionPitchYawRoll(Quaternion q1, Quaternion q2)
    {

        Quaternion qd = Conjugate(q1) * q2;

        float _phi = Mathf.Atan2(2f * (qd.w * qd.x + qd.y * qd.z), 1f - 2f * (Mathf.Pow(qd.x, 2f) + Mathf.Pow(qd.y, 2f)));
        float _theta = Mathf.Asin(2f * (qd.w * qd.y - qd.z * qd.x));
        float _psi = Mathf.Atan2(2f * (qd.w * qd.z + qd.x * qd.y), 1f - 2f * (Mathf.Pow(qd.y, 2f) + Mathf.Pow(qd.z, 2f)));

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


    public void pointToTarget(Vector3 pointingDirection, float RATE)
    {

        Quaternion pointingDirectionLocal = Quaternion.LookRotation(pointingDirection, parentGameObject.transform.up);
        Quaternion commandModuleLocal = Quaternion.LookRotation(parentGameObject.transform.rotation * Vector3.forward, parentGameObject.transform.up);

        Vector3 result = quaternionPitchYawRoll( commandModuleLocal, pointingDirectionLocal);

        //Debug.Log(pointingDirection + " " + result * Mathf.Rad2Deg);

        float sensitivity = 5f;

        rcs_Pitch_Rate = Mathf.Clamp(result.x * sensitivity , -1, 1) * RATE;
        rcs_Yaw_Rate = Mathf.Clamp(result.y * sensitivity , -1, 1) * RATE;
        

        if (Mathf.Abs(result.x * Mathf.Rad2Deg) < 0.1f)
        {
            rcs_Pitch_Rate = 0;
        }

        if (Mathf.Abs(result.y * Mathf.Rad2Deg) < 0.1f)
        {
            rcs_Yaw_Rate = 0;
        }
    }




    
    private void mainEngines()
    {
        // Addforce for main engines
        if (engineThrottle > 0f)
        {
            foreach (MainEngine thisEngine in _inertiaTensor.ProgradeEngines)
            {
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
    private void addForcesAll()
    {

        addForceToThrusterGroup(_inertiaTensor.Thrusters);

    }

    private void addForcesTranslational()
    {

        addForceToThrusterGroup(_inertiaTensor.translate_Fwd);
        addForceToThrusterGroup(_inertiaTensor.translate_Back);

        addForceToThrusterGroup(_inertiaTensor.translate_Up);
        addForceToThrusterGroup(_inertiaTensor.translate_Down);

        addForceToThrusterGroup(_inertiaTensor.translate_Left);
        addForceToThrusterGroup(_inertiaTensor.translate_Right);

    }


    public void addForceToThrusterGroup(List<Thruster> thrusters)
    {

        // Overall addforce for RCS thruters
        foreach (Thruster thisThruster in thrusters)
        {
            float throttle = thisThruster.getFireThrottle();

            if (throttle > 0)
            {
                _inertiaTensor.addForceAtPos(thisThruster, thisThruster.thrustN * Mathf.Abs(throttle), Time.fixedDeltaTime);
            }
            // For drawing particle system.
            thisThruster.throttlePersistant = throttle;
        }
    }

    private void translationalThrottle()
    {

        // Forward
        if (z_Throttle > 0f)
        {
            foreach (Thruster thisThruster in _inertiaTensor.translate_Fwd)
            {
                thisThruster.throttlePersistant = Mathf.Abs(z_Throttle);
            }
        }

        // Aft
        if (z_Throttle < 0f)
        {
            foreach (Thruster thisThruster in _inertiaTensor.translate_Back)
            {
                thisThruster.throttlePersistant = Mathf.Abs(z_Throttle);
                Debug.Log("Z throttle aft = " + thisThruster.throttlePersistant);
            }
        }

        // Up
        if (y_Throttle > 0f)
        {
            foreach (Thruster thisThruster in _inertiaTensor.translate_Up)
            {
                thisThruster.throttlePersistant = Mathf.Abs(y_Throttle);
            }
        }

        // Dn
        if (y_Throttle < 0f)
        {
            foreach (Thruster thisThruster in _inertiaTensor.translate_Down)
            {
                thisThruster.throttlePersistant = Mathf.Abs(y_Throttle);
            }
        }

        // Left
        if (x_Throttle < 0f)
        {
            foreach (Thruster thisThruster in _inertiaTensor.translate_Left)
            {
                thisThruster.throttlePersistant = Mathf.Abs(x_Throttle);
            }
        }

        // Right
        if (x_Throttle > 0f)
        {
            foreach (Thruster thisThruster in _inertiaTensor.translate_Right)
            {
                thisThruster.throttlePersistant = Mathf.Abs(x_Throttle);
            }
        }
    }




    private void translate_Z_PID()
    {

        z_RateController.SetPoint = rcs_z_rate;
        z_RateController.ProcessVariable = localVelocity.z;

        z_Throttle = z_RateController.ControlVariable(Time.fixedDeltaTime, 0f);
        //Debug.Log("Z Axis " + " Target = " + rcs_z_rate + " current = " + localVelocity.z + " throttle = " + z_Throttle + " error = " + pitchController._error);
    }

    private void translate_Y_PID()
    {

        y_RateController.SetPoint = rcs_y_rate;
        y_RateController.ProcessVariable = localVelocity.y;

        y_Throttle = y_RateController.ControlVariable(Time.fixedDeltaTime, 0f);

    }

    private void translate_X_PID()
    {
        x_RateController.SetPoint = rcs_x_rate;
        x_RateController.ProcessVariable = localVelocity.x;

        x_Throttle = x_RateController.ControlVariable(Time.fixedDeltaTime, 0f);
        

    }


    private Vector3 CalculateAverageVector(List<Vector3> vectors)
    {
        Vector3 sum = Vector3.zero;

        foreach (Vector3 vector in vectors)
        {
            sum += vector;
        }

        return sum / vectors.Count;
    }

    private void PitchPID()
    {
        float throttle = 0;

        if (localAngularVelocity.x ==0)
        {
            localAngularVelocity.x = 0.0000001f;
        }

        pitchController.SetPoint = rcs_Pitch_Rate;
        pitchController.ProcessVariable = localAngularVelocity.x;
        pitchError= pitchController._error;

        throttle = pitchController.ControlVariable(Time.fixedDeltaTime, 0);
        pitchThrottle = throttle;

        //Debug.Log(throttle + " current = " + _angularVelocity.x * Mathf.Rad2Deg + " target = " + rcs_Pitch_Rate * Mathf.Rad2Deg);

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
                thisThruster.throttlePersistant = Mathf.Abs( throttle);
            }
        }


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
        yawError = yawController._error;
        throttle = yawController.ControlVariable(Time.fixedDeltaTime, 0);

        yawThrottle = throttle;

        //Debug.Log(throttle + " Current = " + _angularVelocity.y * Mathf.Rad2Deg + " Target = " + rcs_Yaw_Rate * Mathf.Rad2Deg );

        if (throttle > 0f)
        {
            foreach (Thruster thisThruster in _inertiaTensor.Yaw_Left)
            {
                thisThruster.throttlePersistant = Mathf.Abs(throttle);
                //_inertiaTensor.addForceAtPos(thisThruster, thisThruster.thrustN * Mathf.Abs(throttle), Time.fixedDeltaTime);
            }
        }

        if (throttle < 0f)
        {
            foreach (Thruster thisThruster in _inertiaTensor.Yaw_Right)
            {
                thisThruster.throttlePersistant = Mathf.Abs(throttle);
                //_inertiaTensor.addForceAtPos(thisThruster, thisThruster.thrustN * Mathf.Abs(throttle), Time.fixedDeltaTime);
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
        rollError = rollController._error;
        throttle = rollController.ControlVariable(Time.fixedDeltaTime, 0);
        rollThrottle = throttle;

        if (throttle < 0f)
        {
            foreach (Thruster thisThruster in _inertiaTensor.Roll_Left)
            {
                thisThruster.throttlePersistant = Mathf.Abs(throttle);
                //_inertiaTensor.addForceAtPos(thisThruster, thisThruster.thrustN * Mathf.Abs(throttle), Time.fixedDeltaTime);
            }
        }

        if (throttle > 0f)
        {
            foreach (Thruster thisThruster in _inertiaTensor.Roll_Right)
            {
                thisThruster.throttlePersistant = Mathf.Abs(throttle);
                //_inertiaTensor.addForceAtPos(thisThruster, thisThruster.thrustN * Mathf.Abs(throttle), Time.fixedDeltaTime);
            }
        }

    }

    private void dampOsc()
    {
        Vector3 difference = Vector3.zero;

        angularVelocityHistory.Add(localAngularVelocity);
        thrustSettingHistory.Add(new Vector3(rcs_Pitch_Rate, rcs_Yaw_Rate, rcs_Roll_Rate));

        if (historyTracker > historyLen - 1)
        {

            Vector3 averageAngularVel = CalculateAverageVector(angularVelocityHistory);
            averageSetting = CalculateAverageVector(thrustSettingHistory);

            difference = averageAngularVel - averageSetting;
            difference.x = Mathf.Abs(difference.x);
            difference.y = Mathf.Abs(difference.y);
            difference.z = Mathf.Abs(difference.z);

            // Pitch 
            if (difference.x < treashold)
            {
                localAngularVelocity.x = rcs_Pitch_Rate;
                _inertiaTensor.AngularVelocity = parentGameObject.transform.TransformDirection(localAngularVelocity);

            }

            // yaw 
            if (difference.y < treashold)
            {
                localAngularVelocity.y = rcs_Yaw_Rate;
                _inertiaTensor.AngularVelocity = parentGameObject.transform.TransformDirection(localAngularVelocity);
            }

            // roll 
            if (difference.z < treashold)
            {
                localAngularVelocity.z = rcs_Roll_Rate;
                _inertiaTensor.AngularVelocity = parentGameObject.transform.TransformDirection(localAngularVelocity);
            }

            thrustSettingHistory.Clear();
            angularVelocityHistory.Clear();
        }

    }

}
