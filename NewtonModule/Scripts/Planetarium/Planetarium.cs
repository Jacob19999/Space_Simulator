using System;
using System.Collections;
using System.Collections.Generic;
using System.Xml;
using UnityEngine;

public class Planetarium : MonoBehaviour
{
    [Header("Active Camera")]
    public Camera activeCamera;

    [Header("Player Ship")]
    public GameObject playerGo;
    public InertiaTensor playerInertiaTensor;

    public DateTime initialDateTime = new DateTime(2023, 11, 30, 0, 0, 0, 0);
    public DateTime currentDateTime;
    private DrawOrbitLines _drawOrbitLines;

    [Header("Celestial Objects")]
    public Transform helioCentricBody;
    public List<Transform> celestialObjects = new List<Transform>();
    public List<Orbit> celestialObjectOrbit = new List<Orbit>();

    public List<Transform> satelliteObjects = new List<Transform>();
    public List<Orbit> satelliteObjectOrbit = new List<Orbit>();

    [Header("Simulation Speed")]
    [Range(1f,10000f)]
    public float gameSpeed = 1;

    [Header("Planetarium Scale")]
    public float planetariumScale = 100000;

    [Header("Planetarium TimeStep")]
    public float fixedTimeStep = 0;

    [Header("Floating Origin")]
    public GameObject focalPoint;
    public Orbit focalPointOrbit;

    public Vector3d floatingOriginReferencePos = Vector3d.zero;
    public Vector3d floatingOriginTransform = Vector3d.zero;
    public Vector3 floatingOriginTransformGamePos = Vector3.zero;

    private void getFloatingTransformation()
    {
        if (focalPointOrbit)
        {
            focalPointOrbit = focalPoint.GetComponent<Orbit>();
        }

        floatingOriginTransform = Vector3d.zero - focalPointOrbit.helioCentricPosition;
        floatingOriginTransformGamePos = Vector3d.toVector3(floatingOriginTransform / planetariumScale);
        focalPoint.transform.position = Vector3.zero;

        // Continuously set pos to zero for camera

    }



    private void Awake()
    {
        currentDateTime = initialDateTime;
        celestialObjects = RecursiveFindChildTag(helioCentricBody, celestialObjects, "celestialBody");

        // Get initial state vectors for celestial objects
        foreach (Transform thisBody in celestialObjects)
        {
            Orbit thisOrbit = thisBody.GetComponent<Orbit>();
            thisOrbit._init();

            if (!thisOrbit.isBodySun)
            {
                thisOrbit.getStateVectorsFromOrbitParameters(true);
                thisOrbit.meanAnomalyIntegration = thisOrbit.MeanAnomaly;
            }

            celestialObjectOrbit.Add(thisOrbit);

        }
        celestialObjectOrbit.Reverse();



        // Update helio centric position for celestial objects
        foreach (Orbit thisOrbit in celestialObjectOrbit)
        {
            thisOrbit.getHelioPosition();
            //thisOrbit.gameObject.transform.position = Vector3d.toVector3(thisOrbit.helioCentricPosition / planetariumScale) + floatingOriginTransformGamePos;
        }

        // Get initial state vectors satellite objects
        satelliteObjects = RecursiveFindChildTag(helioCentricBody, satelliteObjects, "satellite");

        foreach (Transform thisBody in satelliteObjects)
        {
            Orbit thisOrbit = thisBody.GetComponent<Orbit>();
            thisOrbit._init();

            if (!thisOrbit.isBodySun)
            {
                thisOrbit.getStateVectorsFromOrbitParameters(true);
                thisOrbit.getHelioPosition();
                thisOrbit.meanAnomalyIntegration = thisOrbit.MeanAnomaly;
            }

            satelliteObjectOrbit.Add(thisOrbit);
        }

        satelliteObjects.Reverse();



        // Update helio centric position for satellite objects
        foreach (Orbit thisOrbit in satelliteObjectOrbit)
        {
            thisOrbit.getHelioPosition();
            //thisOrbit.gameObject.transform.position = Vector3d.toVector3(thisOrbit.helioCentricPosition / planetariumScale) + floatingOriginTransformGamePos;
        }


        // Tool To draw orbit lines.
        if (!_drawOrbitLines)
        {
            _drawOrbitLines = gameObject.GetComponent<DrawOrbitLines>();
            _drawOrbitLines._init();
        }


        _drawOrbitLines.initState = true;

    }

    
    
    private void FixedUpdate()
    {
        // Init Player
        if (playerGo == null)
        {
            playerGo = PlayerShipSingleton.Instance.gameObject;
            playerInertiaTensor = playerGo.GetComponent<InertiaTensor>();
        }

        getFloatingTransformation();
        
        fixedTimeStep = Time.fixedDeltaTime * gameSpeed;

        // Integrate game time.
        currentDateTime = currentDateTime.AddSeconds(fixedTimeStep);

        // Integrate Orbit by calculating eccentric anaomaly angle in Rads for two body objects
        foreach (Orbit thisOrbit in celestialObjectOrbit)
        {
            if (thisOrbit.isBodySun == false && thisOrbit._orbitType == Orbit.orbitType.Two_Body)
            {
                // Integrate Orbit
                thisOrbit.UpdateOrbitAnomaliesByTime(fixedTimeStep);
                thisOrbit.updatePositionAndVelocityAtAnomaly();
                thisOrbit.getStateVectorsFromOrbitParameters(false);
                thisOrbit.getHelioPosition();
            }

            thisOrbit.gameObject.transform.position = Vector3d.toVector3(thisOrbit.helioCentricPosition / planetariumScale) + floatingOriginTransformGamePos;

        }
        

        // Integrate Orbit by calculating eccentric anaomaly angle in Rads for n body objects
        foreach (Orbit thisOrbit in satelliteObjectOrbit)
        {
            if (thisOrbit.isBodySun == false && thisOrbit._orbitType == Orbit.orbitType.N_Body)
            {
                
                thisOrbit.IntegrateRK4(fixedTimeStep);
                thisOrbit.getPositionRelativeToParent();
                thisOrbit.CalculateOrbitStateFromOrbitalVectors();
                
            }

            thisOrbit.gameObject.transform.position = Vector3d.toVector3(thisOrbit.helioCentricPosition / planetariumScale) + floatingOriginTransformGamePos;
        }


        // integrate PlayerShip
        //playerOrbit.IntegrateRK4(fixedTimeStep);
        //playerOrbit.getPositionRelativeToParent();
        //playerOrbit.CalculateOrbitStateFromOrbitalVectors();

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

    

}
