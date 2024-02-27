using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.Rendering.VirtualTexturing;
using UnityEngine.UIElements;

//[ExecuteInEditMode]
public class Thruster : MonoBehaviour
{

    [SerializeField] public ObjectType thruster_type;
    [SerializeField] public GameObject jetFireGO;
    [SerializeField] public GameObject embersGO;
    [SerializeField] public float thrustN;
    [SerializeField] public float _scale;

    // State 
    public Vector3 deltaOmega;

    public Vector3 localPos;
    public Vector3 LocalRot;

    //public Vector3 thrustingDir;
    //public Vector3 thrustingNormalized;
    //public Vector3 torque;

    // Used by ship controller to keep track of thruster firing state. 
    public bool toFire = false;

    // Particle System
    public float throttlePersistant = 0f;

    public float averageThrottle = 0f;

    private ParticleSystem jetFire;
    private ParticleSystem embers;

    private ParticleSystem.EmissionModule jetFireModule;
    private ParticleSystem.EmissionModule embersModule;
    private List<float> averageInput = new List<float>();

    private float shapeJetFire;
    private float shapeEmbers;

    // Other stuff
    private float elapsedTime = 0f;
    public string _name {get; private set;}
    
    public float getFireThrottle()
    {
        return Math.Clamp(throttlePersistant, 0, 1);
    }

    public enum ObjectType
    {
        Main_Engine,
        RCS_Thruster
    }

    public enum FuelCombination
    {
        LOX_RP1,
        MMH_NTO,
        UDMH_NTO,
        NITROGEN

    }

    public void Awake()
    {

        _name = gameObject.name;

        jetFire = jetFireGO.GetComponent<ParticleSystem>();
        jetFireModule = jetFire.emission;

        embers = embersGO.GetComponent<ParticleSystem>();
        embersModule = embers.emission;

        shapeJetFire = jetFireModule.rateOverTime.constant;
        shapeEmbers = embersModule.rateOverTime.constant;

    }

    public void FixedUpdate()
    {

        if (elapsedTime == 0f)
        {
            averageInput.Clear();
            throttlePersistant = 0;
        }

        averageInput.Add(throttlePersistant);
        
        elapsedTime = elapsedTime + 1f;

        if (elapsedTime > 25f) {

            averageThrottle = averageInput.Average();
            SetScale(3f * MathF.Abs(averageThrottle));
            elapsedTime = 0f;
        }

    }

    public void SetScale(float scale)
    {

        jetFireModule.rateOverTime = shapeJetFire * scale;
        embersModule.rateOverTime = shapeEmbers * scale;

    }

    public Vector3 getLocalPosition(Transform parentTransform, Vector3 worldCOM)
    {

        localPos = parentTransform.InverseTransformDirection(gameObject.transform.position - worldCOM) ;
        return localPos;
    }

    public Vector3 getThrustDirection(Transform parentTransform)
    {

        LocalRot = parentTransform.InverseTransformDirection(gameObject.transform.forward);
        return LocalRot;
    }


}
