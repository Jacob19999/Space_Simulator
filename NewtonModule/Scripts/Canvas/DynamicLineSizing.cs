using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class DynamicLineSizing : MonoBehaviour
{
    [SerializeField]
    public int minSize = 1;
    public int maxSize = 100;

    private Vector2 minMaxLineWidth;

    [SerializeField]
    [Range(3, 5000)]
    private int lineResolution = 100;

    [SerializeField]
    public Camera observerCam;

    [SerializeField]
    private Color lineColor = Color.white;

    private LineRenderer _lineRenderer;
    private Orbit _orbit;
    private Planetarium _planetarium;

    private void Awake()
    {
        minMaxLineWidth.x = minSize;
        minMaxLineWidth.y = maxSize;

        _lineRenderer = gameObject.GetComponent<LineRenderer>();
        _orbit = gameObject.GetComponent<Orbit>();

        _lineRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        _lineRenderer.receiveShadows = false;

        
        //observerCam = Camera.current;
    }


    private void Update()
    {
        observerCam = Camera.main;

        if (_planetarium == null)
        {
            _planetarium = PlanetariumSingleton.Instance.GetComponent<Planetarium>();
            observerCam = _planetarium.activeCamera;
        }

        if (_lineRenderer.enabled)
        {
            updateOrbitLineSize();
        }

        
    }


    private void updateOrbitLineSize()
    {

        Vector3 observerPoint = observerCam.transform.position;

        float distance = Vector3.Distance(gameObject.transform.position, observerPoint);

        float widthMultiplier = Mathf.Min(distance * (minMaxLineWidth.x / 1000f), minMaxLineWidth.y);

        //Debug.Log("distToOrbit = " + distance + " widthMultiplier = " + widthMultiplier);
        _lineRenderer.widthMultiplier = widthMultiplier;
        _lineRenderer.startColor = lineColor;
        _lineRenderer.endColor = lineColor;

    }


    private float DistanceToOrbitalPlane(Vector3 point, Vector3 orbitCentre, float orbitRadius)
    {
        Vector3 direction = point - orbitCentre;
        Vector3 projectedPoint = orbitCentre + Vector3.ProjectOnPlane(direction, Vector3.up).normalized * orbitRadius;
        return Vector3.Distance(point, projectedPoint);
    }


}
