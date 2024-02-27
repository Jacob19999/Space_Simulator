using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MainEngine : MonoBehaviour
{
    // Engine parameters
    [SerializeField] public float thrustN;
    [Range(0f, 1f)]
    [SerializeField] public float throttle;

    // Exhaust particle Effects
    [SerializeField] public GameObject jetFireGO;
    [SerializeField] public GameObject intenseFireGO;
    [SerializeField] public float exhaustScaleMin;
    [SerializeField] public float exhaustScaleMax;
    private ParticleSystem jetFirePS;
    private ParticleSystem intenseFirePS;
    private ParticleSystem.MainModule jetFireMod;
    private ParticleSystem.MainModule intenseFireMod;


    // If engine has rotating parts
    [SerializeField]
    public GameObject rotationFeature;
    private ObjectRotate rotationFeatureScript;
    private float rotationSpeed;


    // Positional and rotational
    public Vector3 localPos;
    public Vector3 LocalRot;

    void Update()
    {


        jetFirePS = jetFireGO.GetComponent<ParticleSystem>();
        intenseFirePS = intenseFireGO.GetComponent<ParticleSystem>();

        jetFireMod = jetFirePS.main;
        intenseFireMod = intenseFirePS.main;

        if (rotationFeature != null)
        {
            rotationFeatureScript = rotationFeature.GetComponentInChildren<ObjectRotate>();

        } 

        if (rotationFeatureScript != null)
        {
            rotationFeatureScript.SpinSpeed = 500 * throttle;
        }

        if ( throttle < 0.05f)
        {
            throttle = 0f;
        }

        if (throttle == 0f)
        {
            jetFireMod.startLifetime = 0;
            intenseFireMod.startLifetime = 0;

        }
        if (throttle > 0f)
        {
            jetFireGO.SetActive(true);
            intenseFireGO.SetActive(true);
            float scaleSetting = Remap(throttle, 0, 1, exhaustScaleMin, exhaustScaleMax);
            jetFireMod.startLifetime = scaleSetting;
            intenseFireMod.startLifetime = scaleSetting;
        }



    }

    private float Remap(float value, float from1, float to1, float from2, float to2)
    {
        return (value - from1) / (to1 - from1) * (to2 - from2) + from2;
    }

    public Vector3 getThrustDirection(Transform parentTransform)
    {
        LocalRot = parentTransform.InverseTransformDirection(gameObject.transform.forward);
        return LocalRot;
    }

    public Vector3 getLocalPosition(Transform parentTransform, Vector3 worldCOM)
    {

        localPos = parentTransform.InverseTransformDirection(gameObject.transform.position - worldCOM);
        return localPos;
    }


}
