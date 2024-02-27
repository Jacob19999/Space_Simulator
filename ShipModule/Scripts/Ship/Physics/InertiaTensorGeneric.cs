using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using static UnityEngine.Rendering.DebugUI.Table;
using UnityEngine.UIElements;
using Unity.Mathematics;
using static UnityEditor.Rendering.CameraUI;

public class InertiaTensorGeneric : MonoBehaviour
{
    private Rigidbody thisobject;

    public Vector3 VehicleInertiaTensorVector = Vector3.zero;
    public Vector3 AngularVelocity = Vector3.zero;
    public Vector3 AngularVelocityDelta = Vector3.zero;
    public Vector3d worldPosition = Vector3d.zero;

    public Vector3 localVelocity = Vector3.zero;
    public Vector3 worldVelocity = Vector3.zero;

    public Vector3 COM_Combined_World;
    public Vector3 COM_Combined_Local;

    public List<Transform> MainEngines = new List<Transform>();
    public List<Thruster> Thrusters = new List<Thruster>();
    public List<MainEngine> ProgradeEngines = new List<MainEngine>();
    public List<MainEngine> RetroEngines = new List<MainEngine>();
    private List<MainEngine> MainEnginesScript = new List<MainEngine>();

    public List<Thruster> Pitch_Up = new List<Thruster>();
    public List<Thruster> Pitch_Dn = new List<Thruster>();
    public List<Thruster> Roll_Left = new List<Thruster>();
    public List<Thruster> Roll_Right = new List<Thruster>();
    public List<Thruster> Yaw_Left = new List<Thruster>();
    public List<Thruster> Yaw_Right = new List<Thruster>();

    public List<Thruster> translate_Fwd = new List<Thruster>();
    public List<Thruster> translate_Back = new List<Thruster>();
    public List<Thruster> translate_Left = new List<Thruster>();
    public List<Thruster> translate_Right = new List<Thruster>();
    public List<Thruster> translate_Up = new List<Thruster>();
    public List<Thruster> translate_Down = new List<Thruster>();

    private Matrix4x4 VehicleInertiaTensorMatrix = Matrix4x4.zero;
    private Matrix4x4 VehicleInertiaTensorMatrixInverse = Matrix4x4.zero;
    private Vector3 worldAccelLoop = Vector3.zero;
    private Vector3 worldVelocityPrev = Vector3.zero;
    public Vector3 worldAccelVec = Vector3.zero;
    public Vector3 localAccelVec = Vector3.zero;

    public float Mass = 0;
    public bool toIntegrate = true;
    public Vector3 editorVelocity;
    public Vector3 editorPositionPrev;

    public void Awake()
    {
        thisobject = gameObject.GetComponent<Rigidbody>();
        thisobject.detectCollisions = false;
        thisobject.isKinematic = false;
        _init();
    }

    public void resetList()
    {

        MainEngines.Clear();
        Thrusters.Clear();

        ProgradeEngines.Clear();
        RetroEngines.Clear();

        MainEnginesScript.Clear();

        Pitch_Up.Clear();
        Pitch_Dn.Clear();
        Roll_Left.Clear();
        Roll_Right.Clear();
        Yaw_Left.Clear();
        Yaw_Right.Clear();

        translate_Fwd.Clear();
        translate_Back.Clear();
        translate_Left.Clear();
        translate_Right.Clear();
        translate_Up.Clear();
        translate_Down.Clear();

    }
    public void FixedUpdate()
    {
        if (toIntegrate)
        {
            integrate();
        }
        
    }

    public void Update()
    {
        editorVelocity = ((transform.position - editorPositionPrev)) / Time.deltaTime;
        editorPositionPrev = transform.position;

    }
    public float getForwardThrust()
    {
        _init();
        float thrust = 0;
        foreach (MainEngine thisEngine in ProgradeEngines)
        {
            thrust = thrust + thisEngine.thrustN;
        }

        return thrust;
    }

    public void integrate()
    {

        worldVelocityPrev = worldVelocity;

        // Integrate angular accleration
        Quaternion rotation = Quaternion.Euler(-AngularVelocity);

        // Integrate Translational Movement
        worldPosition = worldPosition + (Vector3d.toVector3d(worldVelocity) * Time.fixedDeltaTime);

        transform.rotation = rotation * transform.rotation;

        COM_Combined_World = thisobject.worldCenterOfMass;

        worldAccelVec = worldAccelLoop;
        localAccelVec = transform.InverseTransformDirection(worldAccelLoop);

        worldAccelLoop = Vector3.zero;

        //Debug.Log(worldPosition);

    }

    public void _init()
    {

        // We need to wait for freaking rigidbody to update.
        StartCoroutine(DelayedExecution(0.1f));

    }

    private IEnumerator DelayedExecution(float seconds_to_wait)
    {
        yield return new WaitForSeconds(seconds_to_wait);

        UpdateInertiaMatrix();

    }

    public void UpdateInertiaMatrix()
    {

        
        VehicleInertiaTensorVector = thisobject.inertiaTensor;
        Mass = thisobject.mass;

        thisobject.detectCollisions = true;
        thisobject.isKinematic = true;


        COM_Combined_World = thisobject.worldCenterOfMass;
        COM_Combined_Local = transform.InverseTransformDirection(COM_Combined_World - transform.position);

        VehicleInertiaTensorMatrix = convertTensorTo4x4(VehicleInertiaTensorVector);
        VehicleInertiaTensorVector = tensorMatrixToVec(VehicleInertiaTensorMatrix);
        VehicleInertiaTensorMatrixInverse = getInverse(VehicleInertiaTensorMatrix);


        findThrusters();
        getMainEngines();
        getThrusterMappingRotational();
        getThrusterMappingTranslational();



    }

    

    public void addForceAtPos(Thruster thruster, float thisThrust, float dt)
    {
        // Caluclate Rotational

        Vector3 angularVelocityDelta = transform.TransformDirection(getAngularVelocityChange(thruster, -thisThrust));

        AngularVelocityDelta = angularVelocityDelta;

        AngularVelocity.x = (AngularVelocity.x - (AngularVelocityDelta.x * dt));
        AngularVelocity.y = (AngularVelocity.y - (AngularVelocityDelta.y * dt));
        AngularVelocity.z = (AngularVelocity.z - (AngularVelocityDelta.z * dt));

        Vector3 thrustingDir = thruster.getThrustDirection(transform);
        Vector3 thrustingForce = Vector3.Normalize(thrustingDir) * -thisThrust;
        Vector3 translationalAccel = thrustingForce / thisobject.mass;

        Vector3 worldTranslationalAccel = transform.TransformDirection(translationalAccel);

        worldAccelLoop += worldTranslationalAccel;

        worldVelocity = worldVelocity + (worldTranslationalAccel * dt);
        localVelocity = transform.InverseTransformDirection(worldVelocity);

    }

    public void addForceAtPosMainEngine(MainEngine thisEngine, float thisThrust, float dt)
    {
        // Caluclate Rotational

        Vector3 distance = thisEngine.getLocalPosition(transform, thisobject.worldCenterOfMass);
        Vector3 thrustingDir = thisEngine.getThrustDirection(transform);
        Vector3 torque = Vector3.Cross(Vector3.Normalize(thrustingDir) * -thisThrust, distance);
        Vector3 deltaOmega = VehicleInertiaTensorMatrixInverse.MultiplyVector(torque);

        Vector3 angularVelocityDelta = -transform.TransformDirection(deltaOmega);

        AngularVelocity.x = (AngularVelocity.x - (angularVelocityDelta.x * dt));
        AngularVelocity.y = (AngularVelocity.y - (angularVelocityDelta.y * dt));
        AngularVelocity.z = (AngularVelocity.z - (angularVelocityDelta.z * dt));

        Vector3 thrustingForce = Vector3.Normalize(thrustingDir) * -thisThrust;
        Vector3 translationalAccel = thrustingForce / thisobject.mass;

        Vector3 worldTranslationalAccel = transform.TransformDirection(translationalAccel);

        worldAccelLoop += worldTranslationalAccel;

        worldVelocity = worldVelocity + (worldTranslationalAccel * dt);
        localVelocity = transform.InverseTransformDirection(worldVelocity);

    }


    private Matrix4x4 convertTensorTo4x4(Vector3 tensor)
    {

        Matrix4x4 result = new Matrix4x4();

        result.m00 = tensor.x;
        result.m11 = tensor.y;
        result.m22 = tensor.z;

        return result;
    }

    private Vector3 getAngularVelocityChange(Thruster thruster, float thisThrust)
    {

        Vector3 distance = thruster.getLocalPosition(transform, thisobject.worldCenterOfMass);

        Vector3 thrustingDir = thruster.getThrustDirection(transform);

        Vector3 torque = Vector3.Cross(Vector3.Normalize(thrustingDir) * -thisThrust, distance);

        Vector3 deltaOmega = VehicleInertiaTensorMatrixInverse.MultiplyVector(torque);

        return deltaOmega;
    }

    private void getThrusterMappingRotational()
    {
        float threshold = 0.01f;

        // Rotational Thrusters

        foreach (Thruster thisThruster in Thrusters)
        {
            // Calculate resultant angular velocity change given a specific thruster.
            thisThruster.deltaOmega = getAngularVelocityChange(thisThruster, thisThruster.thrustN);

            Vector3 delta = thisThruster.deltaOmega;

            // Calculate the Highest component.
            if (MathF.Abs(delta.x) > MathF.Abs(delta.y) && MathF.Abs(delta.x) > MathF.Abs(delta.z))
            {

                // Pitch Component X
                if (delta.x > threshold)
                {

                    // Ptich Up
                    Pitch_Up.Add(thisThruster);

                }
                else if (delta.x < -threshold)
                {
                    // Ptich Down 
                    Pitch_Dn.Add(thisThruster);
                }
            }
            else if (MathF.Abs(delta.y) > MathF.Abs(delta.x) && MathF.Abs(delta.y) > MathF.Abs(delta.z))
            {
                // Yaw Component
                if (delta.y > threshold)
                {
                    // Yaw left
                    Yaw_Left.Add(thisThruster);

                }
                else if (delta.y < -threshold)
                {
                    // Yaw Right
                    Yaw_Right.Add(thisThruster);
                }
            }
            else if (MathF.Abs(delta.z) > MathF.Abs(delta.y) && MathF.Abs(delta.z) > MathF.Abs(delta.x))
            {
                // Roll Component
                if (delta.z < -threshold)
                {
                    // Roll left
                    Roll_Left.Add(thisThruster);

                }
                else if (delta.z > threshold)
                {
                    // Roll Right
                    Roll_Right.Add(thisThruster);
                }
            }
        }
    }

   

    private void getThrusterMappingTranslational()
    {
        /*
        translate_Fwd = -Z
        translate_Back = +Z
        translate_Left = +X
        translate_Right = -X
        translate_Up = -Y
        translate_Down = +Y
        */

        // How close it is thruster pointing to desired direction
        float thrusterThreashold = 0.8f;

        // Translational Thrusters
        foreach (Thruster thisThruster in Thrusters)
        {

            Vector3 direction = Vector3.Normalize(thisThruster.getThrustDirection(transform));

            if (direction.z < -1 * thrusterThreashold)
            {
                translate_Fwd.Add(thisThruster);
            }

            if (direction.z > 1 * thrusterThreashold)
            {
                translate_Back.Add(thisThruster);
            }

            if (direction.y < -1 * thrusterThreashold)
            {
                translate_Up.Add(thisThruster);
            }

            if (direction.y > 1 * thrusterThreashold)
            {
                translate_Down.Add(thisThruster);
            }

            if (direction.x < -1 * thrusterThreashold)
            {
                translate_Right.Add(thisThruster);
            }

            if (direction.x > 1 * thrusterThreashold)
            {
                translate_Left.Add(thisThruster);
            }

        }
    }

    public void getMainEngines()
    {
        float thrusterThreashold = 0.8f;

        MainEngines = RecursiveFindChildTag(transform, MainEngines, "MainEngines");

        foreach (Transform thisEngine in MainEngines)
        {
            MainEngine thisEngineScript = thisEngine.GetComponent<MainEngine>();
            MainEnginesScript.Add(thisEngineScript);

            Vector3 direction = Vector3.Normalize(thisEngineScript.getThrustDirection(transform));

            if (direction.z < -1 * thrusterThreashold)
            {
                ProgradeEngines.Add(thisEngineScript);
            }

            if (direction.z > 1 * thrusterThreashold)
            {
                RetroEngines.Add(thisEngineScript);
            }

        }

    }

    private List<Transform> RecursiveFindChildTag(Transform parent, List<Transform> children, string tag)
    {

        foreach (Transform child in parent)
        {
            RecursiveFindChildTag(child, children, tag);
        }

        if (parent.gameObject.tag == tag)
        {
            children.Add(parent);
        }

        return children;
    }

    private void findThrusters()
    {
        List<Transform> AllThrustersGO = new List<Transform>();

        AllThrustersGO = RecursiveFindChild(transform, AllThrustersGO, "Thrusters");


        foreach (Transform child in AllThrustersGO)
        {
            Thrusters.Add(child.gameObject.GetComponent<Thruster>());

        }
    }

    private List<Transform> RecursiveFindChild(Transform parent, List<Transform> children, string LayerName)
    {

        foreach (Transform child in parent)
        {
            RecursiveFindChild(child, children, LayerName);
        }

        if (parent.gameObject.layer == LayerMask.NameToLayer(LayerName))
        {
            children.Add(parent);
        }

        return children;
    }
    private Vector3 tensorMatrixToVec(Matrix4x4 input)
    {
        Vector3 oputput = new Vector3();
        oputput.x = input.m00;
        oputput.y = input.m11;
        oputput.z = input.m22;

        return oputput;
    }
    private float4x4 getInverse(float4x4 m)
    {

        double det = m[0][0] * (m[1][1] * m[2][2] - m[2][1] * m[1][2]) -
                     m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
                     m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);

        //double det = m.m00 * (m.m11 * m.m22 - m.m21 * m.m12) -
        //             m.m01 * (m.m10 * m.m22 - m.m12 * m.m20) +
        //             m.m02 * (m.m10 * m.m21 - m.m11 * m.m20);

        double invdet = 1 / det;

        float4x4 output = new float4x4();

        output[0][0] = (m[1][1] * m[2][2] - m[2][1] * m[1][2]) * (float)invdet;
        //output.m00 = (m.m11 * m.m22 - m.m21 * m.m12) * invdet;

        output[0][1] = (m[0][2] * m[2][1] - m[0][1] * m[2][2]) * (float)invdet;
        //output.m01 = (m.m02 * m.m21 - m.m01 * m.m22) * invdet;

        output[0][2] = (m[0][1] * m[1][2] - m[0][2] * m[1][1]) * (float)invdet;
        //output.m02 = (m.m01 * m.m12 - m.m02 * m.m11) * invdet;

        output[1][0] = (m[1][2] * m[2][0] - m[1][0] * m[2][2]) * (float)invdet;
        //output.m10 = (m.m12 * m.m20 - m.m10 * m.m22) * invdet;

        output[1][1] = (m[0][0] * m[2][2] - m[0][2] * m[2][0]) * (float)invdet;
        //output.m11 = (m.m00 * m.m22 - m.m02 * m.m20) * invdet;

        output[1][2] = (m[1][0] * m[0][2] - m[0][0] * m[1][2]) * (float)invdet;
        //output.m12 = (m.m10 * m.m02 - m.m00 * m.m12) * invdet;

        output[2][0] = (m[1][0] * m[2][1] - m[2][0] * m[1][1]) * (float)invdet;
        //output.m20 = (m.m10 * m.m21 - m.m20 * m.m11) * invdet;

        output[2][1] = (m[2][0] * m[0][1] - m[0][0] * m[2][1]) * (float)invdet;
        //output.m21 = (m.m20 * m.m01 - m.m00 * m.m21) * invdet;

        output[2][2] = (m[0][0] * m[1][1] - m[1][0] * m[0][1]) * (float)invdet;
        //output.m22 = (m.m00 * m.m11 - m.m10 * m.m01) * invdet;


        return output;
    }










}
