using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using UnityEngine.UIElements;
using static UnityEngine.GraphicsBuffer;
using UnityEditor;

public class Orbit : MonoBehaviour
{
    private static double gravConst = 0.0000000000667430;

    public GameObject VectorDebug;

    [Header("Helio Reference")]
    public bool isBodySun = false;

    [Header("Game Object")]
    public GameObject thisBody;
    public GameObject parentBody;
    public Orbit parentBodyOrbit;
    private Planetarium _planetarium;
    public double mass = 0;
    public double radius = 0;
    // Used by other obejct to store force.
    private double forceToObject;

    [Header("Orbit Line")]
    public LineRenderer orbitLineRender;

    [Header("Orbit physics to be used")]
    public orbitType _orbitType;
    public enum orbitType
    {
        Two_Body,
        N_Body
    }

    [Header("Initial Orbit Parameter Input")]
    public double SemiMajorAxisInputKM;
    public double EccentricityInput;
    public double InclinationInput;
    public double argumentOfPeriapsisDegInput;
    public double longitudeOfAscendngNodeDeg;
    public double meanAnomalyDegInput;
    
    
    // State vectors
    public Vector3d position = Vector3d.zero;
    public Vector3d velocity = Vector3d.zero;
    public Vector3d helioCentricPosition = Vector3d.zero;
    public Vector3d helioCentricVelocity = Vector3d.zero;


    [Header("Orbit State Vectors")]
    // Inspector Only
    public Vector3 positionVec3 = Vector3.zero;
    public Vector3 velocityVec3 = Vector3.zero;
    public Vector3 helioCentricPositionVec3 = Vector3.zero;
    public Vector3 helioCentricVelocityVec3 = Vector3.zero;


    public double velocityMag = 0;
    public double helioVelocityMag = 0;

    // List of parent bodies for updating position and vleocity
    private List<Orbit> parentOrbits = new List<Orbit>();

    // Orbital Planes
    /// Normal Vector of Ecliptic
    public Vector3d EclipticNormal = new Vector3d(0, 0, 1);

    /// Up direction on ecliptic plane (y-axis on xy ecliptic plane).
    public Vector3d EclipticUp = new Vector3d(0, 1, 0);

    /// Right vector on ecliptic plane (x-axis on xy ecliptic plane).
    public Vector3d EclipticRight = new Vector3d(1, 0, 0);

    // Nbody orbit Propogation
    public Vector3d futurePosition = Vector3d.zero;
    public Vector3d futureVelocity = Vector3d.zero;
    public Vector3d futureHelioCentricPosition = Vector3d.zero;
    public Vector3d futureHelioCentricVelocity = Vector3d.zero;
    public Vector3d helioCentricPositionPrev = Vector3d.zero;
    public Vector3d helioCentricVelocityPrev = Vector3d.zero;
    public List<Vector3d> nBodyOrbitPoints = new List<Vector3d>();
    public List<Vector3d> nBodyOrbitPointsOld = new List<Vector3d>();
    public List<Vector3d> orbitPoints = new List<Vector3d>();
    public double futureMeanAnomaly = 0;
    public double meanAnomalyIntegration = 0;

    

    public enum referenceFrame
    {
        Heliocentric,
        Parententric
    }
    public referenceFrame _referenceFrame = referenceFrame.Heliocentric;

    

    [Header("Orbit Parameters")]
    public double SemiMinorAxis;
    public double SemiMajorAxis;
    public double Eccentricity;
    public double Inclination;
    public Vector3d Periapsis;
    public double PeriapsisDistance;
    public Vector3d Apoapsis;
    public double ApoapsisDistance;
    public double ArgumentOfPeriapsis;
    public double LongitudeOfAscendingNode;
    public double Period;
    public double PeriodDays;
    public double TrueAnomaly;
    public double MeanAnomaly;
    public double EccentricAnomaly;
    public double MeanMotion;
    public Vector3d CenterPoint;
    public Vector3 CenterPointVec3;
    public Vector3d OrbitNormal;
    public Vector3 OrbitNormalVec3;
    public Vector3d SemiMinorAxisBasis;
    public Vector3 SemiMinorAxisVec3;
    public Vector3d SemiMajorAxisBasis;
    public Vector3 SemiMajorAxisVec3;
    public double FocalParameter;
    public double MG;
    public double distanceFromParent;
    public double OrbitCompressionRatio;
    
    // Orbital Energy
    public double EnergyTotal
    {
        get { return velocity.sqrMagnitude - 2 * (MG) / distanceFromParent; }
    }


    // Orientation of celestial body relative to world space
    public Quaternion thisCelestialAxis;

    /// if > 0, then orbit motion is clockwise
    public double OrbitNormalDotEclipticNormal;

    public bool initState = false;

    public void _init()
    {
        if (!isBodySun)
        {

            if (parentBody == null)
            {
                parentBody = transform.parent.gameObject;
            }
            
            parentBodyOrbit = parentBody.GetComponent<Orbit>();

            if (parentBody)
            {
                EclipticNormal = Vector3d.toVector3d(parentBody.transform.forward);
                EclipticUp = Vector3d.toVector3d(parentBody.transform.up);
                EclipticRight = Vector3d.toVector3d(parentBody.transform.right);
            }

        }

        orbitLineRender = gameObject.GetComponent<LineRenderer>();
        MG = mass * gravConst;
        _planetarium = PlanetariumSingleton.Instance.gameObject.GetComponent<Planetarium>();

        initState = true;
    }

    public void FixedUpdate()
    {
        if (!isBodySun)
        {

            EclipticNormal = Vector3d.toVector3d(parentBody.transform.forward);
            EclipticUp = Vector3d.toVector3d(parentBody.transform.up);
            EclipticRight = Vector3d.toVector3d(parentBody.transform.right);
            thisCelestialAxis = Quaternion.LookRotation(transform.forward, transform.up);
            
        }

        updateInspectorDisplay();
    }


    public void getHelioPosition()
    {
        parentOrbits = findAllParentBody(transform, "celestialBody");

        // Update hjeliocentric position and velocity.
        helioCentricPosition = position;
        helioCentricVelocity = velocity;

        foreach (Orbit thisParent in parentOrbits)
        {
            helioCentricPosition += thisParent.position;
            helioCentricVelocity += thisParent.velocity;
        }

    }

    public void getPositionRelativeToParent()
    {

        position =  helioCentricPosition - parentBodyOrbit.helioCentricPosition;
        velocity =  helioCentricVelocity - parentBodyOrbit.helioCentricVelocity;


    }



    //////////////////////////// Begin N-Body

    public Vector3d getAccelerationNbody(Vector3d KnX)
    {
        Vector3d _acceleration = Vector3d.zero;
        List<double> force = new List<double>();

        foreach (Orbit otherCelestialBody in _planetarium.celestialObjectOrbit)
        {
            if (_planetarium.celestialObjectOrbit.Count > 1 && otherCelestialBody != this)
            {
                // Calculate Acceleration for each body.
                double sqrDst = ((otherCelestialBody.helioCentricPosition - (helioCentricPosition + KnX))).sqrMagnitude;
                Vector3d forceDir = (otherCelestialBody.helioCentricPosition - (helioCentricPosition + KnX)).normalized;

                double _MG = gravConst * otherCelestialBody.mass;
                _acceleration = _acceleration + (forceDir * _MG) / sqrDst;

                // Calculate Resultant force for each body.
                double forceN = (_MG) / sqrDst;
                force.Add(forceN);
                otherCelestialBody.forceToObject = forceN;
            }
        }

        // Calculates sphere of infulence parent object.

        double maxForce = Mathd.Max(force.ToArray());
        foreach (Orbit otherBody in _planetarium.celestialObjectOrbit)
        {
            if (_planetarium.celestialObjectOrbit.Count > 1 && otherBody != this && otherBody != null)
            {
                if (otherBody.forceToObject == maxForce)
                {
                    transform.parent = otherBody.gameObject.transform;
                    parentBody = otherBody.gameObject;
                    parentBodyOrbit = otherBody;
                }
            }
        }

        return _acceleration;
    }

    public void IntegrateRK4(double _dt)
    {
        if (!_planetarium)
        {
            _init();
        }

        // RK4 implementation
        Vector3d k1v = _dt * getAccelerationNbody(Vector3d.zero);
        Vector3d k1x = _dt * helioCentricVelocity;

        Vector3d k2v = _dt * getAccelerationNbody(0.5f * k1x);
        Vector3d k2x = _dt * (helioCentricVelocity + 0.5f * k1v);

        Vector3d k3v = _dt * getAccelerationNbody( 0.5f * k2x);
        Vector3d k3x = _dt * (helioCentricVelocity + 0.5f * k2v);

        Vector3d k4v = _dt * getAccelerationNbody(k3x);
        Vector3d k4x = _dt * (helioCentricVelocity + k3v);

       
        helioCentricVelocity = helioCentricVelocity + ((k1v + (2 * k2v) + (2 * k3v) + k4v) / 6f);
        helioCentricPosition = helioCentricPosition + ((k1x + (2 * k2x) + (2 * k3x) + k4x) / 6f);

        //Debug.Log("RK4 Out " + helioCentricVelocity.magnitude);

    }

    public void IntegrateFuture(double _dt)
    {
        if (!_planetarium)
        {
            _init();
        }

        helioCentricPositionPrev = futureHelioCentricPosition;
        helioCentricVelocityPrev = futureHelioCentricVelocity;

        Vector3d k1v = _dt * getAccelerationNbodyFuture(Vector3d.zero);
        Vector3d k1x = _dt * futureHelioCentricVelocity;

        Vector3d k2v = _dt * getAccelerationNbodyFuture(1 / 2 * k1x);
        Vector3d k2x = _dt * (futureHelioCentricVelocity + 1 / 2 * k1v);

        Vector3d k3v = _dt * getAccelerationNbodyFuture(1 / 2 * k2x);
        Vector3d k3x = _dt * (futureHelioCentricVelocity + 1 / 2 * k2v);

        Vector3d k4v = _dt * getAccelerationNbodyFuture(k3x);
        Vector3d k4x = _dt * (futureHelioCentricVelocity + k3v);


        futureHelioCentricVelocity = futureHelioCentricVelocity + ((k1v + (2 * k2v) + (2 * k3v) + k4v) / 6f);
        futureHelioCentricPosition = futureHelioCentricPosition + ((k1x + (2 * k2x) + (2 * k3x) + k4x) / 6f);

        /*
        Vector3d k1v = _dt * getAccelerationNbodyFuture(helioCentricVelocityPrev);
        Vector3d k1x = _dt * futureHelioCentricVelocity;

        Vector3d k2v = (_dt * (4 / 27)) * getAccelerationNbodyFuture((4 / 27) * k1x);
        Vector3d k2x = (_dt * (4 / 27)) * (futureHelioCentricVelocity + (4 / 27) * k1v);

        Vector3d k3v = (_dt * (2 / 9)) * getAccelerationNbodyFuture((1 / 8) * (k1x + 3 * k2x));
        Vector3d k3x = (_dt * (2 / 9)) * (futureHelioCentricVelocity + (1 / 8) * (k1v + 3 * k2v));

        Vector3d k4v = (_dt * (1 / 3)) * getAccelerationNbodyFuture((1 / 12) * (k1x + 3 * k3x));
        Vector3d k4x = (_dt * (1 / 3)) * (futureHelioCentricVelocity + (1 / 12) * (k1v + 3 * k3v));

        Vector3d k5v = (_dt * (1 / 2)) * getAccelerationNbodyFuture((1 / 8) * (k1x + 3 * k4x));
        Vector3d k5x = (_dt * (1 / 2)) * (futureHelioCentricVelocity + (1 / 128) * (k1v + 3 * k4v));

        Vector3d k6v = (_dt * (2 / 3)) * getAccelerationNbodyFuture((1 / 54) * (13 * k1x - 27 * k3x + 42 * k4x + 8 * k5x));
        Vector3d k6x = (_dt * (2 / 3)) * (futureHelioCentricVelocity + (1 / 54) * (k1v - 27 * k3v + 42 * k4v + 8 * k5v));

        Vector3d k7v = (_dt * (1 / 6)) * getAccelerationNbodyFuture((1 / 4320) * (389 * k1x - 54 * k3x + 966 * k4x - 824 * k5x + 243 * k6x));
        Vector3d k7x = (_dt * (1 / 6)) * (futureHelioCentricVelocity + (1 / 4320) * (389 * k1v - 54 * k3v + 966 * k4v - 824 * k5v + 243 * k6v));

        Vector3d k8v = (_dt) * getAccelerationNbodyFuture((1 / 20) * (-234 * k1x + 81 * k3x - 1164 * k4x + 656 * k5x - 122 * k6x + 800 * k7x));
        Vector3d k8x = (_dt) * (futureHelioCentricVelocity + (1 / 20) * (-234 * k1v + 81 * k3v - 1164 * k4v + 656 * k5v - 122 * k6v + 800 * k7v));

        Vector3d k9v = (_dt * (5 / 6)) * getAccelerationNbodyFuture((5 / 6) * (-127 * k1x + 18 * k3x - 678 * k4x + 456 * k5x - 9 * k6x + 576 * k7x + 4 * k8x));
        Vector3d k9x = (_dt * (5 / 6)) * (futureHelioCentricVelocity + (5 / 6) * (-127 * k1v + 18 * k3v - 678 * k4v + 456 * k5v - 9 * k6v + 576 * k7v + 4 * k8v));

        Vector3d k10v = (_dt) * getAccelerationNbodyFuture((1 / 820) * (1481 * k1x - 81 * k3x + 7104 * k4x - 3376 * k5x + 72 * k6x - 5040 * k7x - 60 * k8x + 720 * k9x));
        Vector3d k10x = (_dt) * (futureHelioCentricVelocity + (1 / 820) * (1481 * k1v - 81 * k2v + 7104 * k4v - 3376 * k5v + 72 * k6v - 5040 * k7v - 60 * k8v + 720 * k9v));

        futureHelioCentricVelocity = futureHelioCentricVelocity + ((_dt / 840) * (41 * k1v + 27 * k4v + 272 * k5v + 27 * k6v + 216 * k7v + 216 * k9v + 41 * k10v));
        futureHelioCentricPosition = futureHelioCentricPosition + ((_dt / 840) * (41 * k1x + 27 * k4x + 272 * k5x + 27 * k6x + 216 * k7x + 216 * k9x + 41 * k10x));

        */


    }

    public Vector6d firstOrderIntegrator(Vector6d y, double _dt)
    {
        return getAccelerationNbodyFuture(_dt, y);
    }

    public (Vector3d, Vector3d) RungeKutta_4(Vector3d k_v, Vector3d k_x, double _dt)
    {
        // y = initial Position

        Vector3d k_1v = k_v + getVelocityNbodyFuture(_dt, k_x);
        Vector3d k_1x = k_x + (_dt * k_1v);

        Vector3d k_2v = k_v + getVelocityNbodyFuture(_dt / 2, (_dt / 2) * k_1x);
        Vector3d k_2x = k_x + ((_dt / 2)* k_1v);

        Vector3d k_3v = k_v + getVelocityNbodyFuture(_dt / 2, (_dt / 2) * k_2x);
        Vector3d k_3x = k_x + ((_dt / 2) * k_2v);

        Vector3d k_4v = k_v + getVelocityNbodyFuture(_dt, _dt * k_3v);
        Vector3d k_4x = k_x + (_dt * k_3v);


        Vector3d k_v_new = k_v + (_dt / 6) * (k_1v + 2 * k_2v + 2 * k_3v + k_4v);
        Vector3d k_x_new = k_x + (_dt / 6) * (k_1x + 2 * k_2x + 2 * k_3x + k_4x);

        return (k_x_new, k_v_new);
    }

    public Vector3d getVelocityNbodyFuture(double _dt, Vector3d position)
    {

        Vector3d _acceleration = Vector3d.zero;

        foreach (Orbit otherCelestialBody in _planetarium.celestialObjectOrbit)
        {
            if (_planetarium.celestialObjectOrbit.Count > 1 && otherCelestialBody != this)
            {

                // Calculate Acceleration for each body.
                double sqrDst = ((otherCelestialBody.futureHelioCentricPosition - (position))).sqrMagnitude;
                Vector3d forceDir = (otherCelestialBody.futureHelioCentricPosition - (position)).normalized;

                _acceleration = _acceleration + (forceDir * otherCelestialBody.MG) / sqrDst;

            }
        }

        return _acceleration * _dt;

    }

    // Runga kutta 8th order integrator
    public (Vector3d,Vector3d) RungeKutta_8(Vector3d _initialPosition, Vector3d _initialVelocity, double _dt)
    {
        Vector6d y = new Vector6d(_initialPosition, _initialVelocity);

        Vector6d k_1 = getAccelerationNbodyFuture(0, y);

        Vector6d k_2 = getAccelerationNbodyFuture(_dt * (4 / 27), y + ((_dt * 4 / 27) * k_1));

        Vector6d k_3 = getAccelerationNbodyFuture(_dt * (2 / 9),  y + (_dt / 18) * (k_1 + 3 * k_2));

        Vector6d k_4 = getAccelerationNbodyFuture(_dt * (1 / 3),  y + (_dt / 12) * (k_1 + 3 * k_3));

        Vector6d k_5 = getAccelerationNbodyFuture(_dt * (1 / 2),  y + (_dt / 8) * (k_1 + 3 * k_4));

        Vector6d k_6 = getAccelerationNbodyFuture(_dt * (2 / 3),  y + (_dt / 54) * (13 * k_1 - 27 * k_3 + 42 * k_4 + 8 * k_5));

        Vector6d k_7 = getAccelerationNbodyFuture(_dt * (1 / 6),  y + (_dt / 4320) * (389 * k_1 - 54 * k_3 + 966 * k_4 - 824 * k_5 + 243 * k_6));

        Vector6d k_8 = getAccelerationNbodyFuture(_dt          ,  y + (_dt / 20) * (-234 * k_1 + 81 * k_3 - 1164 * k_4 + 656 * k_5 - 122 * k_6 + 800 * k_7));

        Vector6d k_9 = getAccelerationNbodyFuture(_dt * (5 / 6),  y + (_dt / 288) * (-127 * k_1 + 18 * k_3 - 678 * k_4 + 456 * k_5 - 9 * k_6 + 576 * k_7 + 4 * k_8));

        Vector6d k_10 = getAccelerationNbodyFuture(_dt         ,  y + (_dt / 820) * (1481 * k_1 - 81 * k_3 + 7104 * k_4 - 3376 * k_5 + 72 * k_6 - 5040 * k_7 - 60 * k_8 + 720 * k_9));

        Vector6d y_result = y + _dt / 840 * (41 * k_1 + 27 * k_4 + 272 * k_5 + 27 * k_6 + 216 * k_7 + 216 * k_9 + 41 * k_10);


        return (y_result.getPosition(), y_result.getVeloicty());

   }


    



    public Vector6d getAccelerationNbodyFuture(double _dt, Vector6d thisStateVector)
    {

        Vector3d _acceleration = Vector3d.zero;

        foreach (Orbit otherCelestialBody in _planetarium.celestialObjectOrbit)
        {
            if (_planetarium.celestialObjectOrbit.Count > 1 && otherCelestialBody != this)
            {
                
                // Calculate Acceleration for each body.
                double sqrDst = ((otherCelestialBody.futureHelioCentricPosition - (thisStateVector.getPosition()))).sqrMagnitude;
                Vector3d forceDir = (otherCelestialBody.futureHelioCentricPosition - (thisStateVector.getPosition())).normalized;

                _acceleration = _acceleration + (forceDir * otherCelestialBody.MG) / sqrDst;

            }
        }



        // Get current pos and velocity
        Vector3d currentVeloicty = thisStateVector.getVeloicty();
        Vector3d currentPosition = thisStateVector.getPosition();

        Debug.Log(gameObject.name);

        // Integrate via time step.
        Vector3d newVeloicty = currentVeloicty + (_acceleration * _dt);
        Vector3d newPosition = currentPosition + (newVeloicty * _dt);

        // Return value.
        thisStateVector.setVelocity(newVeloicty);
        thisStateVector.setPosition(newPosition);

        Debug.Log("Acceleration " + _acceleration);
        Debug.Log("Current Velocity " + currentVeloicty + " Integrated Veloicty " + newVeloicty);
        Debug.Log("Current Position " + currentPosition + " Integrated Position " + newPosition);

        return thisStateVector;
    }

    public Vector3d getAccelerationNbodyFuture(Vector3d KnX)
    {
        Vector3d _acceleration = Vector3d.zero;

        foreach (Orbit otherCelestialBody in _planetarium.celestialObjectOrbit)
        {
            if (_planetarium.celestialObjectOrbit.Count > 1 && otherCelestialBody != this)
            {
                // Calculate Acceleration for each body.
                double sqrDst = ((otherCelestialBody.futureHelioCentricPosition - (futureHelioCentricPosition + KnX))).sqrMagnitude;
                Vector3d forceDir = (otherCelestialBody.futureHelioCentricPosition - (futureHelioCentricPosition + KnX)).normalized;

                double _MG = gravConst * otherCelestialBody.mass;
                _acceleration = _acceleration + (forceDir * _MG) / sqrDst;

            }
        }

        return _acceleration;
    }

    //////////////////////////// End N-Body

    public List<Orbit> findAllParentBody(Transform thisBody, string tag)
    {
        List<Orbit> _orbits = new List<Orbit>();

        Transform t = thisBody;
        while (t.parent != null)
        {
            if (t.parent.tag == tag)
            {
                // Found a parent
                _orbits.Add(t.parent.GetComponent<Orbit>());
            }
            t = t.parent.transform;
        }
        return _orbits;
    }


    public void updateHelioPositionVelocity()
    {
        helioCentricPosition = parentBodyOrbit.position + position;
        helioCentricVelocity = parentBodyOrbit.velocity + velocity;
    }

    public double getFutureEccentricAnomaly(double elapsedTimeS)
    {
        double futureEccentricAnomaly = 0;

        futureMeanAnomaly = MeanAnomaly + (MeanMotion * elapsedTimeS);
        if (Eccentricity < 1.0)
        {
            // Get future eccentric Anomaly
            futureMeanAnomaly %= OrbitUtils.PI_2;
            if (futureMeanAnomaly < 0)
            {
                futureMeanAnomaly = OrbitUtils.PI_2 - futureMeanAnomaly;
            }

            futureEccentricAnomaly = OrbitUtils.KeplerSolver(futureMeanAnomaly, Eccentricity);
        }
        else if (Eccentricity > 1.0)
        {
            futureEccentricAnomaly = OrbitUtils.KeplerSolverHyperbolicCase(futureMeanAnomaly, Eccentricity);

        } else
        {
            futureEccentricAnomaly = OrbitUtils.ConvertMeanToEccentricAnomaly(futureMeanAnomaly, Eccentricity);
        }

        return futureEccentricAnomaly;
    }


    public void UpdateOrbitAnomaliesByTime(double deltaTime)
    {
        if (!_planetarium)
        {
            _init();
        }

        if (Eccentricity < 1.0)
        {
            MeanAnomaly += MeanMotion * deltaTime;
            MeanAnomaly %= OrbitUtils.PI_2;

            if (MeanAnomaly < 0)
            {
                MeanAnomaly = OrbitUtils.PI_2 - MeanAnomaly;
            }

            EccentricAnomaly = OrbitUtils.KeplerSolver(MeanAnomaly, Eccentricity);
            double cosE = Math.Cos(EccentricAnomaly);
            TrueAnomaly = Math.Acos((cosE - Eccentricity) / (1 - Eccentricity * cosE));
            if (MeanAnomaly > Math.PI)
            {
                TrueAnomaly = OrbitUtils.PI_2 - TrueAnomaly;
            }
        }
        else if (Eccentricity > 1.0)
        {
            MeanAnomaly = MeanAnomaly + MeanMotion * deltaTime;
            EccentricAnomaly = OrbitUtils.KeplerSolverHyperbolicCase(MeanAnomaly, Eccentricity);
            TrueAnomaly = Math.Atan2(Math.Sqrt(Eccentricity * Eccentricity - 1.0) * Math.Sinh(EccentricAnomaly), Eccentricity - Math.Cosh(EccentricAnomaly));
        }
        else
        {
            MeanAnomaly = MeanAnomaly + MeanMotion * deltaTime;
            EccentricAnomaly = OrbitUtils.ConvertMeanToEccentricAnomaly(MeanAnomaly, Eccentricity);
            TrueAnomaly = EccentricAnomaly;
        }
        //Debug.Log("Mean = " + MeanAnomaly + " True = " + TrueAnomaly);
    }

    public void updatePositionAndVelocityAtAnomaly()
    {

        CalculateOrbitStateFromOrbitalElements();
    }

    public void getStateVectorsFromEccentricAnomaly(float eccentricAnomaly)
    {

        position = GetCentralPositionAtEccentricAnomaly(eccentricAnomaly);
        TrueAnomaly = OrbitUtils.ConvertEccentricToTrueAnomaly(eccentricAnomaly, Eccentricity);
        velocity = GetVelocityAtTrueAnomaly(TrueAnomaly);

    }


    public void getStateVectorsFromOrbitParameters(bool init)
    {

        SemiMajorAxis = SemiMajorAxisInputKM * 1000;

        // Handle Inclination input
        InclinationInput %= 360;
        if (InclinationInput > 180) { InclinationInput -= 360; }

        // Handle AOP
        argumentOfPeriapsisDegInput %= 360;
        if (argumentOfPeriapsisDegInput > 180) { argumentOfPeriapsisDegInput -= 360; }

        // Handle LAN
        if (InclinationInput == 0)
        {
            longitudeOfAscendngNodeDeg = 0;
        }
        else
        {
            longitudeOfAscendngNodeDeg %= 360;
            if (longitudeOfAscendngNodeDeg > 180) { longitudeOfAscendngNodeDeg -= 360; }
        }

        // Handle Eccentricity
        Eccentricity = EccentricityInput;

        if (Eccentricity < 1.0)
        {
            SemiMinorAxis = SemiMajorAxis * Math.Sqrt(1 - Eccentricity * Eccentricity);
        }
        else if (Eccentricity > 1.0)
        {
            SemiMinorAxis = SemiMajorAxis * Math.Sqrt(Eccentricity * Eccentricity - 1);
        }
        else
        {
            SemiMajorAxis = 0;
        }



        var normal = -EclipticUp.normalized;
        var ascendingNode = -EclipticRight.normalized;

        // Rotate relative to local;
        ascendingNode = OrbitUtils.RotateVectorByAngle(ascendingNode, longitudeOfAscendngNodeDeg * OrbitUtils.Deg2Rad, normal).normalized;
        normal = OrbitUtils.RotateVectorByAngle(normal, InclinationInput * OrbitUtils.Deg2Rad, ascendingNode).normalized;

        Periapsis = ascendingNode;
        Periapsis = OrbitUtils.RotateVectorByAngle(Periapsis, argumentOfPeriapsisDegInput * OrbitUtils.Deg2Rad, normal).normalized;

        SemiMajorAxisBasis = Periapsis;
        SemiMinorAxisBasis = Vector3d.Cross(Periapsis, normal);

        if (init)
        {
            // Initial Condition
            MeanAnomaly = meanAnomalyDegInput * OrbitUtils.Deg2Rad;
            EccentricAnomaly = OrbitUtils.ConvertMeanToEccentricAnomaly(MeanAnomaly, Eccentricity);
            TrueAnomaly = OrbitUtils.ConvertEccentricToTrueAnomaly(EccentricAnomaly, Eccentricity);
        }

        CalculateOrbitStateFromOrbitalElements();


        PeriodDays = Period / (60 * 60 * 24);

    }

    public void updateInspectorDisplay()
    {

        CenterPointVec3 = Vector3d.toVector3(CenterPoint);
        OrbitNormalVec3 = Vector3d.toVector3(OrbitNormal);
        SemiMinorAxisVec3 = Vector3d.toVector3(SemiMinorAxisBasis);
        SemiMajorAxisVec3 = Vector3d.toVector3(SemiMajorAxisBasis);
        positionVec3 = Vector3d.toVector3(position);
        velocityVec3 = Vector3d.toVector3(velocity);
        helioCentricPositionVec3 = Vector3d.toVector3(helioCentricPosition);
        helioCentricVelocityVec3 = Vector3d.toVector3(helioCentricVelocity);
        velocityMag = velocity.magnitude;
        helioVelocityMag = helioCentricVelocityVec3.magnitude;
    }

    /// <summary>
    /// Calculates the full state of orbit from current orbital elements: eccentricity, mean anomaly, semi major and semi minor axis.
    /// </summary>
    public void CalculateOrbitStateFromOrbitalElements()
    {
        //Debug.Log(transform.name);
        MG = parentBodyOrbit.mass * gravConst;
        OrbitNormal = -Vector3d.Cross(SemiMajorAxisBasis, SemiMinorAxisBasis).normalized;
        OrbitNormalDotEclipticNormal = Vector3d.Dot(OrbitNormal, EclipticNormal);

        if (Eccentricity < 1.0)
        {
            OrbitCompressionRatio = 1 - Eccentricity * Eccentricity;
            CenterPoint = -SemiMajorAxisBasis * SemiMajorAxis * Eccentricity;
            Period = OrbitUtils.PI_2 * Math.Sqrt(Math.Pow(SemiMajorAxis, 3) / MG);
            MeanMotion = OrbitUtils.PI_2 / Period;

            Apoapsis = CenterPoint - SemiMajorAxisBasis * SemiMajorAxis;
            Periapsis = CenterPoint + SemiMajorAxisBasis * SemiMajorAxis;
            PeriapsisDistance = Periapsis.magnitude - parentBodyOrbit.radius;
            ApoapsisDistance = Apoapsis.magnitude - parentBodyOrbit.radius;
            // All anomalies state already preset.
        }
        else if (Eccentricity > 1.0)
        {
            CenterPoint = SemiMajorAxisBasis * SemiMajorAxis * Eccentricity;
            Period = double.PositiveInfinity;
            MeanMotion = Math.Sqrt(MG / Math.Pow(SemiMajorAxis, 3));


            Apoapsis = new Vector3d(double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity);
            Periapsis = CenterPoint - SemiMajorAxisBasis * (SemiMajorAxis);
            PeriapsisDistance = Periapsis.magnitude - parentBodyOrbit.radius;
            ApoapsisDistance = double.PositiveInfinity - parentBodyOrbit.radius;
        }
        else
        {
            // Escape Orbit
            CenterPoint = new Vector3d();
            Period = double.PositiveInfinity;
            MeanMotion = Math.Sqrt(MG * 0.5 / Math.Pow(PeriapsisDistance, 3));

            Apoapsis = new Vector3d(double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity);
            PeriapsisDistance = SemiMajorAxis - parentBodyOrbit.radius;
            SemiMajorAxis = 0;
            Periapsis = -PeriapsisDistance * SemiMajorAxisBasis;
            ApoapsisDistance = double.PositiveInfinity - parentBodyOrbit.radius;
        }

        position = GetFocalPositionAtEccentricAnomaly(EccentricAnomaly);
        double compresion = Eccentricity < 1 ? (1 - Eccentricity * Eccentricity) : (Eccentricity * Eccentricity - 1);
        FocalParameter = SemiMajorAxis * compresion;
        velocity = GetVelocityAtTrueAnomaly(TrueAnomaly);

        //distanceFromParent = Vector3d.Distance(parentBodyOrbit.position, position);
        //Debug.Log("Position = " + position + " Velocity " + velocity);
    }

    public Vector3d GetVelocityAtTrueAnomaly(double trueAnomaly)
    {
        if (FocalParameter <= 0)
        {
            return new Vector3d();
        }

        double sqrtMGdivP = Math.Sqrt(MG / FocalParameter);
        double vX = sqrtMGdivP * (Eccentricity + Math.Cos(trueAnomaly));
        double vY = sqrtMGdivP * Math.Sin(trueAnomaly);
        return -SemiMinorAxisBasis * vX - SemiMajorAxisBasis * vY;
    }


    public Vector3d GetCentralPositionAtEccentricAnomaly(double eccentricAnomaly)
    {
        if (Eccentricity < 1.0)
        {

            Vector3d result = new Vector3d(Math.Sin(eccentricAnomaly) * SemiMinorAxis, -Math.Cos(eccentricAnomaly) * SemiMajorAxis);

            return -SemiMinorAxisBasis * result.x - SemiMajorAxisBasis * result.y;
        }
        else if (Eccentricity > 1.0)
        {
            Vector3d result = new Vector3d(Math.Sinh(eccentricAnomaly) * SemiMinorAxis, Math.Cosh(eccentricAnomaly) * SemiMajorAxis);
            return -SemiMinorAxisBasis * result.x - SemiMajorAxisBasis * result.y;
        }
        else
        {
            var pos = new Vector3d(
                PeriapsisDistance * Math.Sin(eccentricAnomaly) / (1.0 + Math.Cos(eccentricAnomaly)),
                PeriapsisDistance * Math.Cos(eccentricAnomaly) / (1.0 + Math.Cos(eccentricAnomaly)));
            return -SemiMinorAxisBasis * pos.x + SemiMajorAxisBasis * pos.y;
        }
    }

    /// <summary>
    /// Gets the focal position at eccentric anomaly.
    public Vector3d GetFocalPositionAtEccentricAnomaly(double eccentricAnomaly)
    {
        return GetCentralPositionAtEccentricAnomaly(eccentricAnomaly) + CenterPoint;
    }


    public void inspectorCalculateStateVector()
    {
        getStateVectorsFromOrbitParameters(true);
        getHelioPosition();
        updateInspectorDisplay();
    }

    public void updateGameObjectPos()
    {

        inspectorCalculateStateVector();
        transform.position = Vector3d.toVector3(helioCentricPosition / 100000f);

    }

    public void CalculateOrbitStateFromOrbitalVectors()
    {
        
        Vector3d angularMomentumVector = Vector3d.Cross(position, velocity);

        OrbitNormal = angularMomentumVector.normalized;

        distanceFromParent = position.magnitude;

        MG = parentBodyOrbit.mass * gravConst;

        Vector3d eccVector;
        if (OrbitNormal.sqrMagnitude < 0.99)
        {
            OrbitNormal = Vector3d.Cross(position, EclipticUp).normalized;
            eccVector = new Vector3d();
        }
        else
        {
            eccVector = Vector3d.Cross(velocity, angularMomentumVector) / MG - position / distanceFromParent;
        }


        OrbitNormalDotEclipticNormal = Vector3d.Dot(OrbitNormal, EclipticNormal);
        FocalParameter = angularMomentumVector.sqrMagnitude / MG;

        Eccentricity = eccVector.magnitude;
        SemiMinorAxisBasis = Vector3d.Cross(angularMomentumVector, -eccVector).normalized;


        if (SemiMinorAxisBasis.sqrMagnitude < 0.99)
        {
            SemiMinorAxisBasis = Vector3d.Cross(OrbitNormal, position).normalized;
        }

        SemiMajorAxisBasis = Vector3d.Cross(OrbitNormal, SemiMinorAxisBasis).normalized;
        if (Eccentricity < 1.0)
        {
            OrbitCompressionRatio = 1 - Eccentricity * Eccentricity;
            SemiMajorAxis = FocalParameter / OrbitCompressionRatio;
            SemiMinorAxis = SemiMajorAxis * Math.Sqrt(OrbitCompressionRatio);
            CenterPoint = -SemiMajorAxis * eccVector;
            var p = Math.Sqrt(Math.Pow(SemiMajorAxis, 3) / MG);
            Period = OrbitUtils.PI_2 * p;
            MeanMotion = 1d / p;
            Apoapsis = CenterPoint - (SemiMajorAxisBasis * SemiMajorAxis);
            Periapsis = CenterPoint + (SemiMajorAxisBasis * SemiMajorAxis);
            PeriapsisDistance = Periapsis.magnitude - parentBodyOrbit.radius;
            ApoapsisDistance = Apoapsis.magnitude - parentBodyOrbit.radius;
            TrueAnomaly = Vector3d.Angle(position, SemiMajorAxisBasis) * OrbitUtils.Deg2Rad;
            if (Vector3d.Dot(Vector3d.Cross(position, -SemiMajorAxisBasis), OrbitNormal) < 0)
            {
                TrueAnomaly = OrbitUtils.PI_2 - TrueAnomaly;
            }

            EccentricAnomaly = OrbitUtils.ConvertTrueToEccentricAnomaly(TrueAnomaly, Eccentricity);
            MeanAnomaly = EccentricAnomaly - Eccentricity * Math.Sin(EccentricAnomaly);
        }
        else if (Eccentricity > 1.0)
        {
            OrbitCompressionRatio = Eccentricity * Eccentricity - 1;
            SemiMajorAxis = FocalParameter / OrbitCompressionRatio;
            SemiMinorAxis = SemiMajorAxis * Math.Sqrt(OrbitCompressionRatio);
            CenterPoint = SemiMajorAxis * eccVector;
            Period = double.PositiveInfinity;
            MeanMotion = Math.Sqrt(MG / Math.Pow(SemiMajorAxis, 3));
            Apoapsis = new Vector3d(double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity);
            Periapsis = CenterPoint - SemiMajorAxisBasis * (SemiMajorAxis);
            PeriapsisDistance = Periapsis.magnitude - parentBodyOrbit.radius;
            ApoapsisDistance = double.PositiveInfinity;
            TrueAnomaly = Vector3d.Angle(position, eccVector) * OrbitUtils.Deg2Rad;
            if (Vector3d.Dot(Vector3d.Cross(position, -SemiMajorAxisBasis), OrbitNormal) < 0)
            {
                TrueAnomaly = -TrueAnomaly;
            }

            EccentricAnomaly = OrbitUtils.ConvertTrueToEccentricAnomaly(TrueAnomaly, Eccentricity);
            MeanAnomaly = Math.Sinh(EccentricAnomaly) * Eccentricity - EccentricAnomaly;
        }
        else
        {
            OrbitCompressionRatio = 0;
            SemiMajorAxis = 0;
            SemiMinorAxis = 0;
            PeriapsisDistance = (angularMomentumVector.sqrMagnitude / MG) - parentBodyOrbit.radius;
            CenterPoint = new Vector3d();
            Periapsis = -PeriapsisDistance * SemiMinorAxisBasis;
            Period = double.PositiveInfinity;
            MeanMotion = Math.Sqrt(MG / Math.Pow(PeriapsisDistance, 3));
            Apoapsis = new Vector3d(double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity);
            ApoapsisDistance = double.PositiveInfinity;
            TrueAnomaly = Vector3d.Angle(position, eccVector) * OrbitUtils.Deg2Rad;
            if (Vector3d.Dot(Vector3d.Cross(position, -SemiMajorAxisBasis), OrbitNormal) < 0)
            {
                TrueAnomaly = -TrueAnomaly;
            }

            EccentricAnomaly = OrbitUtils.ConvertTrueToEccentricAnomaly(TrueAnomaly, Eccentricity);
            MeanAnomaly = Math.Sinh(EccentricAnomaly) * Eccentricity - EccentricAnomaly;
        }

        // Inclination
        Inclination = Mathd.Acos(Vector3d.Dot(OrbitNormal, EclipticUp)) * Mathf.Rad2Deg;

        // Longitude of ascending Node
        var ascNodeDir = Vector3d.Cross(EclipticNormal, OrbitNormal).normalized;
        var LANangle = Mathd.Acos(Vector3d.Dot(ascNodeDir, EclipticRight));

        if (Vector3d.Dot(Vector3d.Cross(ascNodeDir, EclipticRight), EclipticNormal) >= 0)
        {
            LongitudeOfAscendingNode = OrbitUtils.PI_2 - LANangle;
        } else
        {
            LongitudeOfAscendingNode = LANangle;
        }

        // Argument of periapsis
        var AOPangle = Math.Acos(Vector3d.Dot(ascNodeDir, eccVector) / (ascNodeDir.magnitude * eccVector.magnitude));
        if (eccVector.z < 0)
        {
            ArgumentOfPeriapsis = OrbitUtils.PI_2 - AOPangle;
        } else
        {
            ArgumentOfPeriapsis = AOPangle;
        }
        ArgumentOfPeriapsis = ArgumentOfPeriapsis * Mathf.Rad2Deg;




        LongitudeOfAscendingNode = LongitudeOfAscendingNode * Mathf.Rad2Deg;

    }

    public Vector3d GetFocalPositionAtTrueAnomaly(double trueAnomaly)
    {
        return GetCentralPositionAtTrueAnomaly(trueAnomaly) + CenterPoint;
    }

    public Vector3d GetCentralPositionAtTrueAnomaly(double trueAnomaly)
    {
        double ecc = OrbitUtils.ConvertTrueToEccentricAnomaly(trueAnomaly, Eccentricity);
        return GetCentralPositionAtEccentricAnomaly(ecc);
    }

    public List<Vector3d> GetOrbitPoints(int pointsCount, double maxDistance = 100000f)
    {
        List<Vector3d> orbitPoints = new List<Vector3d>();

        if (pointsCount < 2)
        {
            orbitPoints.Add(Vector3d.zero);
            return orbitPoints;
        }

        if (Eccentricity < 1)
        {

            if (ApoapsisDistance < maxDistance)
            {
                for (int i = 0; i < pointsCount; i++)
                {
                    orbitPoints.Add(GetFocalPositionAtEccentricAnomaly(i * OrbitUtils.PI_2 / (pointsCount - 1d))  + parentBodyOrbit.helioCentricPosition);
                   
                }
            }
            else
            {
                double maxAngle = OrbitUtils.CalcTrueAnomalyForDistance(maxDistance, Eccentricity, SemiMajorAxis, PeriapsisDistance);
                for (int i = 0; i < pointsCount; i++)
                {
                    orbitPoints.Add(GetFocalPositionAtTrueAnomaly(-maxAngle + i * 2d * maxAngle / (pointsCount - 1)) + parentBodyOrbit.helioCentricPosition);
                }
            }
        }

        if (Eccentricity > 1)
        {
            if (maxDistance < PeriapsisDistance)
            {
                orbitPoints.Add(Vector3d.zero);
                return orbitPoints;
            }
            double maxAngle = OrbitUtils.CalcTrueAnomalyForDistance(maxDistance, Eccentricity, SemiMajorAxis, PeriapsisDistance);

            for (int i = 0; i < pointsCount; i++)
            {
                orbitPoints.Add(GetFocalPositionAtTrueAnomaly(-maxAngle + i * 2d * maxAngle / (pointsCount - 1)) + parentBodyOrbit.helioCentricPosition);
            }
        }

        return orbitPoints;
    }




}

public struct Vector6d
{
    public double x;
    public double y;
    public double z;
    public double w;
    public double v;
    public double u;

    public Vector6d(Vector3d position, Vector3d veloicty)
    {

        this.x = position.x;
        this.y = position.y;
        this.z = position.z;

        this.w = veloicty.x;
        this.v = veloicty.y;
        this.u = veloicty.z;

    }

    public void setPosition(Vector3d position)
    {
        this.x = position.x;
        this.y = position.y;
        this.z = position.z;


    }

    public void setVelocity(Vector3d veloicty)
    {
        this.w = veloicty.x;
        this.v = veloicty.y;
        this.u = veloicty.z;
    }

    public Vector3d getPosition()
    {
        return new Vector3d(x, y, z);
    }

    public Vector3d getVeloicty()
    {
        return new Vector3d(w, v, u);
    }

    public Vector6d Add(Vector6d otherVector)
    {

        Vector3d _Position = this.getPosition() + otherVector.getPosition();
        Vector3d _Veloicty = this.getVeloicty() + otherVector.getVeloicty();

        return new Vector6d(_Position, _Veloicty);
    }

    public Vector6d Multiply(double scalar)
    {

        Vector3d _Position = this.getPosition() * scalar;
        Vector3d _Veloicty = this.getVeloicty() * scalar;

        return new Vector6d(_Position, _Veloicty);
    }

    public static Vector6d operator * (Vector6d vector, double scalar)
    {
        Vector3d _Position = vector.getPosition() * scalar;
        Vector3d _Veloicty = vector.getVeloicty() * scalar;

        return new Vector6d(_Position, _Veloicty);
    }

    public static Vector6d operator * (double scalar, Vector6d vector)
    {
        Vector3d _Position = vector.getPosition() * scalar;
        Vector3d _Veloicty = vector.getVeloicty() * scalar;

        return new Vector6d(_Position, _Veloicty);
    }

    public static Vector6d operator + (Vector6d vector, Vector6d vector2)
    {
        Vector3d _Position = vector.getPosition() + vector2.getPosition();
        Vector3d _Veloicty = vector.getVeloicty() + vector2.getVeloicty();

        return new Vector6d(_Position, _Veloicty);
    }

    public static Vector6d operator - (Vector6d vector, Vector6d vector2)
    {
        Vector3d _Position = vector.getPosition() - vector2.getPosition();
        Vector3d _Veloicty = vector.getVeloicty() - vector2.getVeloicty();

        return new Vector6d(_Position, _Veloicty);
    }


}




[CustomEditor(typeof(Orbit))]
public class EditorPhysicsDebug : Editor
{

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        Orbit _orbit = (Orbit)target;

        if (GUILayout.Button("Calculate State Vector"))
        {
            if (!_orbit.isBodySun)
            {
                _orbit.inspectorCalculateStateVector();
            }
        }
    }
}
