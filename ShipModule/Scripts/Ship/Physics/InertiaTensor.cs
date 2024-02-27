using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Mathematics;
using Unity.VisualScripting;
using UnityEditor.VersionControl;
using UnityEngine;
using static Unity.VisualScripting.Metadata;
using UnityEditor;

public class InertiaTensor : MonoBehaviour
{
    
    private SpaceShip _shipController;
    private InteractionController interactionController;

    [Header("State")]
    public bool initState = false;
    public bool isPlayerShip = false;
    public bool integrateOrbit = false;

    [Header("Compute Composite COM ?")]
    public bool compositeCOM = true;

    [Header("Orbit Ancher")]
    public GameObject orbitAncherObject;
    public Orbit orbitAncherOrbit;

    [Header("Objects")]
    public GameObject parentGameObject;
    public Rigidbody parentRigidBody;
    public GameObject COM_Indicator;
    public FloatingOrigin _floatingOrigin;
    public Orbit thisOrbit;

    [Header("Modules")]
    public List<Rigidbody> VehicleModules = new List<Rigidbody>();

    [Header("Mass")]
    public float compositeMass;

    [Header("Center Of Mass")]
    public Vector3 COM_Combined_Local = Vector3.zero;
    public Vector3 COM_Combined_World = Vector3.zero;

    [Header("Inertia Tensor")]
    public Vector3 VehicleInertiaTensorVector = Vector3.zero;
    private Matrix4x4 VehicleInertiaTensorMatrix = Matrix4x4.zero;
    private Matrix4x4 VehicleInertiaTensorMatrixInverse = Matrix4x4.zero;

    [Header("Angular Rates")]
    public Vector3 AngularVelocity = Vector3.zero;
    public Vector3 AngularVelocityDelta = Vector3.zero;
    private Vector3 angularVelocityPrevious = Vector3.zero;

    [Header("Velocity")]
    public Vector3 localVelocity = Vector3.zero;
    public double localVelocityMagnitude = 0f;
    public double VelocityMagnitude = 0f;

    [Header("Acceleration")]
    public Vector3 localAccelVec = Vector3.zero;
    public Vector3 worldAccelVec = Vector3.zero;
    public Vector3 worldVelocity = Vector3.zero;
    public double worldAccel = 0f;
    public double localAccel = 0f;

    private Vector3 worldAccelLoop = Vector3.zero;
    private Vector3 localVelocityPrev = Vector3.zero;
    private Vector3 worldVelocityPrev = Vector3.zero;

    [Header("Position")]
    public Vector3d worldPosition = Vector3d.zero;
    public Vector3 editorVelocity = Vector3.zero;
    private Vector3 editorPositionPrev = Vector3.zero;

    [Header("Engines")]
    public List<Transform> MainEngines = new List<Transform>();
    private List<MainEngine> MainEnginesScript = new List<MainEngine>();

    [Header("Engine Mapping")]
    public List<MainEngine> ProgradeEngines = new List<MainEngine>();
    public List<MainEngine> RetroEngines = new List<MainEngine>();
    

    [Header("RCS Thrusters")]
    public List<Thruster> Thrusters = new List<Thruster>();

    [Header("RCS Mapping Rotational")]
    public List<Thruster> Pitch_Up = new List<Thruster>();
    public List<Thruster> Pitch_Dn = new List<Thruster>();
    public List<Thruster> Roll_Left = new List<Thruster>();
    public List<Thruster> Roll_Right = new List<Thruster>();
    public List<Thruster> Yaw_Left = new List<Thruster>();
    public List<Thruster> Yaw_Right = new List<Thruster>();

    [Header("RCS Mapping Translational")]
    public List<Thruster> translate_Fwd = new List<Thruster>();
    public List<Thruster> translate_Back = new List<Thruster>();
    public List<Thruster> translate_Left = new List<Thruster>();
    public List<Thruster> translate_Right = new List<Thruster>();
    public List<Thruster> translate_Up = new List<Thruster>();
    public List<Thruster> translate_Down = new List<Thruster>();

    
    // Utils
    public bool resume = false;
    public bool saved = false;
    


    public void Awake()
    {
        _init(true);
    }

    public void Update()
    {
        editorVelocity = ((transform.position - editorPositionPrev)) / Time.deltaTime;
        editorPositionPrev = transform.position;

    }

    public void resetList()
    {

        VehicleModules.Clear();
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

    public void disableTensor()
    {
        resetList();
        resume = false;
    }
    public void _init(bool resetState)
    {

        initState = false;

        // Empty all modules and thrusters
        resetList();

        // Get reuqired Objects
        parentGameObject = gameObject;
        parentRigidBody = gameObject.GetComponent<Rigidbody>();

        if (integrateOrbit)
        {
            orbitAncherOrbit = orbitAncherObject.GetComponent<Orbit>();
        }
        

        if (PlayerShipSingleton.Instance.gameObject == gameObject)
        {
            isPlayerShip = true;
            interactionController = InteractionControllerSingleton.Instance.GetComponent<InteractionController>();
            _floatingOrigin = FloatingOriginSingleton.Instance.gameObject.GetComponent<FloatingOrigin>();
            _floatingOrigin.setInitialOffset();
            _shipController = gameObject.transform.GetComponentInChildren<SpaceShip>();

        }

        if (resetState)
        {
            localVelocity = Vector3.zero;
            worldVelocity = Vector3.zero;
            AngularVelocity = Vector3.zero;
        }

        if (isPlayerShip)
        {
            // Get All Rb to list
            List<Transform> AllModulesGO = new List<Transform>();

            AllModulesGO = RecursiveFindChild(parentGameObject.transform, AllModulesGO, "Modules");
            AllModulesGO.Reverse();

            foreach (Transform t in AllModulesGO)
            {
                VehicleModules.Add(t.GetComponent<Rigidbody>());
            }


            // Set each body to non kinematic to get inertia tensor
            foreach (Rigidbody ve in VehicleModules)
            {
                ve.detectCollisions = false;
                ve.isKinematic = false;
                ve.WakeUp();
            }
        } else if (!isPlayerShip)
        {

            parentRigidBody.detectCollisions = false;
            parentRigidBody.isKinematic = false;
            parentRigidBody.WakeUp();

        }

        // We need to wait for freaking rigidbody to update.
        StartCoroutine(DelayedExecution(0.1f));

    }

    public void inertiaTensorUpdate(float _dt)
    {
        if (isPlayerShip)
        {
            // Stops ship rotation 
            if (interactionController.globalEnable == true)
            {
                _shipController.enabled = false;

            }
            else if (interactionController.globalEnable)
            {
                _shipController.enabled = true;

            }
        }


        localVelocityMagnitude = localVelocity.magnitude;
        VelocityMagnitude = worldVelocity.magnitude;
        worldAccel = worldAccelVec.magnitude;

        integrate(_dt);

        if (integrateOrbit)
        {
            //orbitAncherOrbit.helioCentricPosition = worldPosition;
            //orbitAncherOrbit.helioCentricVelocity = Vector3d.toVector3d(worldVelocity);
        }
    }
    private void FixedUpdate()
    {

        


    }



    public void integrate(float _dt)
    {
        if (initState)
        {
            worldVelocityPrev = worldVelocity;

            // Integrate angular accleration
            Quaternion rotation = Quaternion.Euler(-AngularVelocity);

            // Integrate Translational Movement
            worldPosition = worldPosition + (Vector3d.toVector3d(worldVelocity) * _dt);

            if (isPlayerShip)
            {
                // Update Floating Origin
                _floatingOrigin.worldPosPlayer = worldPosition;
                _floatingOrigin.updatePosition();
                COM_Combined_World = Vector3.zero;


            } else
            {
                COM_Combined_World = parentRigidBody.worldCenterOfMass;
            }
            
            // update Object along COM
            RotateAround(parentGameObject.transform, COM_Combined_World, rotation);

            worldAccelVec = worldAccelLoop;
            localAccelVec = parentGameObject.transform.InverseTransformDirection(worldAccelLoop);
            worldAccelLoop = Vector3.zero;

            // Update COM is present.
            if (COM_Indicator != null)
            {
                COM_Indicator.transform.position = COM_Combined_World;
            }
            //Debug.Log(worldPosition);
        }

    }

    private IEnumerator DelayedExecution(float seconds_to_wait)
    {
        yield return new WaitForSeconds(seconds_to_wait);

        initState = true;
        resume = true;

        UpdateInertiaMatrix();

    }

    public void UpdateInertiaMatrix()
    {
        if (isPlayerShip)
        {
            COM_Combined_World = getCompositeCOM();
            COM_Combined_Local = parentGameObject.transform.InverseTransformDirection(COM_Combined_World - parentGameObject.transform.position);

            VehicleInertiaTensorMatrix = getTensor();
            VehicleInertiaTensorVector = tensorMatrixToVec(VehicleInertiaTensorMatrix);
            VehicleInertiaTensorMatrixInverse = getInverse(VehicleInertiaTensorMatrix);

            foreach (Rigidbody ve in VehicleModules)
            {
                //Debug.Log(ve.name);
                ve.detectCollisions = true;
                ve.isKinematic = true;
            }

        }
        
        if (!isPlayerShip)
        {
            VehicleInertiaTensorVector = parentRigidBody.inertiaTensor;
            compositeMass = parentRigidBody.mass;

            COM_Combined_World = parentRigidBody.worldCenterOfMass;
            COM_Combined_Local = transform.InverseTransformDirection(COM_Combined_World - transform.position);

            VehicleInertiaTensorMatrix = convertTensorTo4x4(VehicleInertiaTensorVector);
            VehicleInertiaTensorVector = tensorMatrixToVec(VehicleInertiaTensorMatrix);
            VehicleInertiaTensorMatrixInverse = getInverse(VehicleInertiaTensorMatrix);

            parentRigidBody.detectCollisions = true;
            parentRigidBody.isKinematic = true;

        }

        findThrusters();
        getMainEngines();
        getThrusterMappingRotational();
        getThrusterMappingTranslational();

    }

    

    private static void RotateAround(Transform _transform, Vector3 pivotPoint, Quaternion rot)
    {

        _transform.position = rot * (_transform.position - pivotPoint) + pivotPoint;
        _transform.rotation = rot * _transform.rotation ;

    }

    public void addForceAtPos(Thruster thruster, float thisThrust, float dt)
    {
        // Caluclate Rotational

        Vector3 angularVelocityDelta = parentGameObject.transform.TransformDirection( getAngularVelocityChange(thruster, -thisThrust));

        AngularVelocityDelta = angularVelocityDelta;

        AngularVelocity.x = (AngularVelocity.x - (AngularVelocityDelta.x * dt));
        AngularVelocity.y = (AngularVelocity.y - (AngularVelocityDelta.y * dt));
        AngularVelocity.z = (AngularVelocity.z - (AngularVelocityDelta.z * dt));

        Vector3 thrustingDir = thruster.getThrustDirection(parentGameObject.transform);
        Vector3 thrustingForce = Vector3.Normalize(thrustingDir) * -thisThrust;
        Vector3 translationalAccel = thrustingForce / compositeMass;

        Vector3 worldTranslationalAccel = parentGameObject.transform.TransformDirection(translationalAccel);

        worldAccelLoop += (worldTranslationalAccel);

        worldVelocity = worldVelocity + (worldTranslationalAccel  * dt);
        localVelocity = parentGameObject.transform.InverseTransformDirection(worldVelocity);

    }

    public void addForceAtPosMainEngine(MainEngine thisEngine, float thisThrust, float dt)
    {
        // Caluclate Rotational

        Vector3 distance = thisEngine.getLocalPosition(parentGameObject.transform, COM_Combined_World);
        Vector3 thrustingDir = thisEngine.getThrustDirection(parentGameObject.transform);
        Vector3 torque = Vector3.Cross(Vector3.Normalize(thrustingDir) * -thisThrust, distance);

        Vector3 deltaOmega = VehicleInertiaTensorMatrixInverse.MultiplyVector(torque);

        Vector3 angularVelocityDelta = -parentGameObject.transform.TransformDirection(deltaOmega);

        AngularVelocity.x = (AngularVelocity.x - (angularVelocityDelta.x * dt));
        AngularVelocity.y = (AngularVelocity.y - (angularVelocityDelta.y * dt));
        AngularVelocity.z = (AngularVelocity.z - (angularVelocityDelta.z * dt));

        Vector3 thrustingForce = Vector3.Normalize(thrustingDir) * -thisThrust;
        Vector3 translationalAccel = thrustingForce / compositeMass;

        Vector3 worldTranslationalAccel = parentGameObject.transform.TransformDirection(translationalAccel);

        worldAccelLoop += worldTranslationalAccel;

        worldVelocity = worldVelocity + (worldTranslationalAccel * dt);
        localVelocity = parentGameObject.transform.InverseTransformDirection(worldVelocity);

    }



    private Vector3 getCompositeCOM()
    {

        Vector3 compositeCenterOfMass = Vector3.zero;
        compositeMass = 0.0f;

        if (VehicleModules.Count == 1)
        {
            compositeMass = VehicleModules[0].mass;
            compositeCenterOfMass = VehicleModules[0].worldCenterOfMass;

        } else
        {
            for ( int i = 0; i < VehicleModules.Count; i++)
            {
                compositeCenterOfMass = compositeCenterOfMass + VehicleModules[i].worldCenterOfMass * VehicleModules[i].mass;
                compositeMass = compositeMass + VehicleModules[i].mass;
            }

            compositeCenterOfMass = compositeCenterOfMass / compositeMass;
            //Debug.Log("Composite = " + compositeCenterOfMass);
        }

        return compositeCenterOfMass;

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

    public Matrix4x4 getTensor()
    {

        Matrix4x4 output = Matrix4x4.zero;

        if (VehicleModules.Count < 2)
        {
            output = convertTensorTo4x4(VehicleModules[0].inertiaTensor);
        }
        else
        {

            Matrix4x4 _identity = Matrix4x4.identity;
            _identity.m33 = 0;

            foreach (Rigidbody module in VehicleModules)
            {

                Vector3 offset = parentGameObject.transform.InverseTransformDirection( module.worldCenterOfMass - COM_Combined_World);

                Matrix4x4 offsetOuterProd = OuterProduct(offset);
                float offsetDot = Vector3.Dot(offset, offset);

                offsetOuterProd.m33 = 0;

                Matrix4x4 moduleInertiaTensor = convertTensorTo4x4(module.inertiaTensor);

                Matrix4x4 translation = matrixMatrixOperation("-", matrix4x4MultiplyScalar(offsetDot, _identity), offsetOuterProd);

                Matrix4x4 translatedTensor = matrixMatrixOperation("+", moduleInertiaTensor, matrix4x4MultiplyScalar(module.mass, translation));

                translatedTensor.m01 = 0;
                translatedTensor.m02 = 0;

                translatedTensor.m10 = 0;
                translatedTensor.m12 = 0;

                translatedTensor.m20 = 0;
                translatedTensor.m21 = 0;

                output = matrixMatrixOperation("+", output, translatedTensor);

            }
        }

        return output;
    }

    public void resetBody()
    {

        AngularVelocity = Vector3.zero;
        localVelocity = Vector3.zero;
        //worldVelocity = Vector3.zero;

        parentGameObject.transform.rotation = Quaternion.identity;

        parentGameObject.transform.position = parentGameObject.transform.position - COM_Combined_World;

        //_init();

    }

    public void getMainEngines()
    {
        float thrusterThreashold = 0.8f;

        MainEngines = RecursiveFindChildTag(parentGameObject.transform, MainEngines, "MainEngines");

        foreach (Transform thisEngine in MainEngines)
        {
            MainEngine thisEngineScript = thisEngine.GetComponent<MainEngine>();
            MainEnginesScript.Add(thisEngineScript);

            Vector3 direction = Vector3.Normalize(thisEngineScript.getThrustDirection(parentGameObject.transform));

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

            Vector3 direction = Vector3.Normalize(thisThruster.getThrustDirection(parentGameObject.transform));

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

    private void findThrusters()
    {
        Debug.Log("Find Thrusters " + parentGameObject.name);

        List<Transform> AllThrustersGO = new List<Transform>();

        AllThrustersGO = RecursiveFindChild(parentGameObject.transform, AllThrustersGO, "Thrusters");

        foreach (Transform child in AllThrustersGO)
        {
            Thrusters.Add(child.gameObject.GetComponent<Thruster>());

        }
    }

    private Vector3 getAngularVelocityChange(Thruster thruster, float thisThrust)
    {

        Vector3 distance = thruster.getLocalPosition(parentGameObject.transform, COM_Combined_World);

        Vector3 thrustingDir = thruster.getThrustDirection(parentGameObject.transform);

        Vector3 torque = Vector3.Cross(Vector3.Normalize(thrustingDir) * -thisThrust, distance);

        Vector3 deltaOmega = VehicleInertiaTensorMatrixInverse.MultiplyVector(torque);

        return deltaOmega;
    }


    private void getThrusterMappingRotational()
    {
        float threshold = 0.1f;

        // Rotational Thrusters
        
        foreach (Thruster thisThruster in Thrusters)
        {
            // Calculate resultant angular velocity change given a specific thruster.
            thisThruster.deltaOmega = getAngularVelocityChange(thisThruster, -thisThruster.thrustN);

            Vector3 delta = thisThruster.deltaOmega;

            // Calculate the Highest component.
            if (MathF.Abs(delta.x) > MathF.Abs(delta.y) && MathF.Abs(delta.x) > MathF.Abs(delta.z))
            {

                // Pitch Component X
                if (delta.x > threshold)
                {
                    // Ptich Down 
                    Pitch_Dn.Add(thisThruster);

                }
                else if (delta.x < -threshold)
                {
                    // Ptich Up
                    Pitch_Up.Add(thisThruster);
                }
            }
            else if (MathF.Abs(delta.y) > MathF.Abs(delta.x) && MathF.Abs(delta.y) > MathF.Abs(delta.z))
            {
                // Yaw Component
                if (delta.y > threshold)
                {
                    // Yaw Right
                    Yaw_Right.Add(thisThruster);
                   

                }
                else if (delta.y < -threshold)
                {
                    // Yaw left
                    Yaw_Left.Add(thisThruster);
                }
            }
            else if (MathF.Abs(delta.z) > MathF.Abs(delta.y) && MathF.Abs(delta.z) > MathF.Abs(delta.x))
            {
                // Roll Component
                if (delta.z < -threshold)
                {
                    // Roll Right
                    Roll_Right.Add(thisThruster);
                    

                }
                else if (delta.z > threshold)
                {
                    // Roll left
                    Roll_Left.Add(thisThruster);
                }
            }
        }
    }

    private Vector3 tensorMatrixToVec(Matrix4x4 input)
    {
        Vector3 oputput = new Vector3();
        oputput.x = input.m00;
        oputput.y = input.m11;
        oputput.z = input.m22;

        return oputput;
    }

    private Matrix4x4 convertTensorTo4x4(Vector3 tensor)
    {

        Matrix4x4 result = new Matrix4x4();

        result.m00 = tensor.x;
        result.m11 = tensor.y;
        result.m22 = tensor.z;

        return result;
    }

    private Matrix4x4 translational(Vector3 offset)
    {

        Matrix4x4 offsetmatrix = new Matrix4x4();

        offsetmatrix.m00 = MathF.Pow(offset.y, 2) + MathF.Pow(offset.z, 2);
        offsetmatrix.m01 = -offset.x * offset.y;
        offsetmatrix.m02 = -offset.x * offset.z;

        offsetmatrix.m10 = -offset.x * offset.y;
        offsetmatrix.m11 = MathF.Pow(offset.x, 2) + MathF.Pow(offset.z, 2);
        offsetmatrix.m12 = -offset.y * offset.z;

        offsetmatrix.m20 = -offset.x * offset.z;
        offsetmatrix.m21 = -offset.y * offset.z;
        offsetmatrix.m22 = MathF.Pow(offset.x, 2) + MathF.Pow(offset.y, 2);

        return offsetmatrix;

    }


    private Matrix4x4 OuterProduct(Vector3 vector)
    {

        Matrix4x4 outerProductMatrix = new Matrix4x4();

        outerProductMatrix.m00 = vector.x * vector.x;
        outerProductMatrix.m01 = vector.x * vector.y;
        outerProductMatrix.m02 = vector.x * vector.z;

        outerProductMatrix.m10 = vector.y * vector.x;
        outerProductMatrix.m11 = vector.y * vector.y;
        outerProductMatrix.m12 = vector.y * vector.z;

        outerProductMatrix.m20 = vector.z * vector.x;
        outerProductMatrix.m21 = vector.z * vector.y;
        outerProductMatrix.m22 = vector.z * vector.z;

        return outerProductMatrix;
    }

    private float4x4 matrix4x4MultiplyScalar(float scalar, float4x4 matrix1)
    {

        float4x4 resultMatrix = new float4x4();

        // Multiply the original matrix by the scalar
        for (int row = 0; row < 4; row++)
        {
            for (int col = 0; col < 4; col++)
            {
                resultMatrix[row][col] = matrix1[row][col] * scalar;
            }
        }

        return resultMatrix;
    }


    private Matrix4x4 matrixMatrixOperation(string sign, Matrix4x4 matrix1, Matrix4x4 matrix2)
    {

        Matrix4x4 resultMatrix = Matrix4x4.zero;

        if (sign != "+" && sign != "-" && sign != "*")
        {
            Debug.Log("Error matrixMatrixOperation input correct sign.");
            return resultMatrix;
        }

        for (int i = 0; i < 16; i++)
        {
            if (sign == "+")
            {
                resultMatrix[i] = matrix1[i] + matrix2[i];
            }
            if (sign == "-")
            {
                resultMatrix[i] = matrix1[i] - matrix2[i];
            }

        }

        if (sign == "*")
        {
            int rA = 4;
            int cA = 4;
            int cB = 4;

            float temp = 0;

            for (int i = 0; i < rA; i++)
            {
                for (int j = 0; j < cB; j++)
                {
                    temp = 0;
                    for (int k = 0; k < cA; k++)
                    {
                        temp += matrix1[i, k] * matrix2[k, j];
                    }
                    resultMatrix[i, j] = temp;
                }
            }
        }

        return resultMatrix;

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

[CustomEditor(typeof(InertiaTensor))]
public class InertiaTensorInspector : Editor
{

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        InertiaTensor _inertia = (InertiaTensor)target;

        if (GUILayout.Button("Update"))
        {
            _inertia._init(true);

        }

        if (GUILayout.Button("Reset Body"))
        {
            _inertia.resetBody();

        }

        if (GUILayout.Button("Clear list"))
        {
            _inertia.resetList();

        }



    }
}
