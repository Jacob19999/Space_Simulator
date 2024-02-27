using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class DrawOrbitLines : MonoBehaviour
{

    public Planetarium _planetarium;

    [Header("Global State")]
    public bool drawLines = false;
    public bool drawLinesN_Body = true;
    public bool drawGizmos = false;

    [Header("Line Settings Two Body")]
    public float maxRenderDistance = 100000f;
    public int twoBodySegmentCount = 50;

    [Header("Line Settings N-Body")]
    [Range(10, 50000)]
    public int nBodySegmentCount = 1000;
    [Range(1f, 100f)]
    public float iteraionPerframe = 2f;
    [Range(.1f, 100f)]
    public float nBodySegmentLength = 1f;
    public float currentSegment = 0f;
    public float elapsedTime = 0f;

    [Header("offsetVector")]
    public Vector3 offset = Vector3.zero;


    List<Vector3> nBodyPoints = new List<Vector3>();

    public bool initState = false;
    public void _init()
    {
        _planetarium = GetComponent<Planetarium>();

    }

    void FixedUpdate()
    {
        if (_planetarium == null)
        {
            _init();
        }

        if (_planetarium.gameSpeed < 5)
        {
            drawLinesN_Body = true;
        } else
        {
            drawLinesN_Body = false;
        }


        if (!initState || !drawLines)
        {
            currentSegment = 0;
            elapsedTime = 0;
            return;
        }

        // Reset the future position to current position.
        for (int i = 0; i < iteraionPerframe; i++)
        {
            iteration();
        }

        drawRenderLinesN_Body();

    }

    private void iteration()
    {
        if (currentSegment == 0f)
        {
            foreach (Orbit thisNbodyOrbit in _planetarium.satelliteObjectOrbit)
            {
                thisNbodyOrbit.futureHelioCentricPosition = thisNbodyOrbit.helioCentricPosition;
                thisNbodyOrbit.futureHelioCentricVelocity = thisNbodyOrbit.helioCentricVelocity;
                thisNbodyOrbit.nBodyOrbitPoints.Clear();
                thisNbodyOrbit.nBodyOrbitPointsOld.Clear();
            }
        }

        elapsedTime = nBodySegmentLength * currentSegment;

        if (drawLinesN_Body)
        {
            propogateNBodyOrbit(elapsedTime);
        } else
        {
            currentSegment = 0f;
        }
 
        currentSegment += 1;

        if (currentSegment > nBodySegmentCount)
        {
            currentSegment = 0f;
            elapsedTime = 0f;
        }
    }
    public void drawOrbits()
    {
        drawRenderLinesTwo_Body();
        drawLines = true;

        foreach (Orbit thisNbodyOrbit in _planetarium.satelliteObjectOrbit)
        {
            if (thisNbodyOrbit.orbitLineRender != null)
            {
                thisNbodyOrbit.orbitLineRender.enabled = true;
            }
        }

        foreach (Orbit thisNbodyOrbit in _planetarium.celestialObjectOrbit)
        {
            if (thisNbodyOrbit.orbitLineRender != null)
            {
                thisNbodyOrbit.orbitLineRender.enabled = true;
            }
        }


    }

    public void clearAllOrbitPoints()
    {
        currentSegment = 0;
        elapsedTime = 0;
        drawLines = false;

        foreach (Orbit thisNbodyOrbit in _planetarium.satelliteObjectOrbit)
        {

            thisNbodyOrbit.nBodyOrbitPoints.Clear();
            thisNbodyOrbit.nBodyOrbitPointsOld.Clear();
            thisNbodyOrbit.futureHelioCentricPosition = thisNbodyOrbit.helioCentricPosition;
            thisNbodyOrbit.futureHelioCentricVelocity = thisNbodyOrbit.helioCentricVelocity;
            if (thisNbodyOrbit.orbitLineRender != null)
            {
                thisNbodyOrbit.orbitLineRender.enabled = false;
            }
        }

        foreach (Orbit thisNbodyOrbit in _planetarium.celestialObjectOrbit)
        {
            Debug.Log(thisNbodyOrbit + " off");
            if (thisNbodyOrbit.orbitLineRender != null)
            {
                thisNbodyOrbit.orbitLineRender.enabled = false;
            }
            
        }
    }
    private void propogateNBodyOrbit(float timeFromCurrentS)
    {
        foreach (Orbit thisOrbit in _planetarium.celestialObjectOrbit)
        {
            if (!thisOrbit.isBodySun && thisOrbit._orbitType == Orbit.orbitType.Two_Body)
            {
                double futureEccVec = thisOrbit.getFutureEccentricAnomaly(timeFromCurrentS);
                thisOrbit.futurePosition = thisOrbit.GetFocalPositionAtEccentricAnomaly(futureEccVec);
                thisOrbit.futureHelioCentricPosition = thisOrbit.futurePosition + thisOrbit.parentBodyOrbit.helioCentricPosition;
            }
        }

        foreach (Orbit thisNbodyOrbit in _planetarium.satelliteObjectOrbit)
        {

            thisNbodyOrbit.IntegrateFuture(nBodySegmentLength);
            thisNbodyOrbit.futurePosition = thisNbodyOrbit.futureHelioCentricPosition - thisNbodyOrbit.parentBodyOrbit.futureHelioCentricPosition;
            
            if (thisNbodyOrbit._referenceFrame == Orbit.referenceFrame.Heliocentric)
            {
                thisNbodyOrbit.nBodyOrbitPoints.Add(thisNbodyOrbit.futureHelioCentricPosition);
            }

            if (thisNbodyOrbit._referenceFrame == Orbit.referenceFrame.Parententric)
            {
                thisNbodyOrbit.nBodyOrbitPoints.Add(thisNbodyOrbit.futurePosition);

            }

        }

    }

    

    private void drawRenderLinesN_Body()
    {
        
        // Draw new lines
        foreach (Orbit thisNbodyOrbit in _planetarium.satelliteObjectOrbit)
        {
            
            if (thisNbodyOrbit.orbitLineRender != null)
            {
                //Debug.Log(thisNbodyOrbit.name + " Draw Line");

                // Draw New Line
                Vector3d parentBodyHelioPos = Vector3d.zero;
                if (thisNbodyOrbit._referenceFrame == Orbit.referenceFrame.Heliocentric)
                {
                    

                }
                if (thisNbodyOrbit._referenceFrame == Orbit.referenceFrame.Parententric)
                {
                    parentBodyHelioPos = thisNbodyOrbit.parentBodyOrbit.helioCentricPosition;
                }

                thisNbodyOrbit.orbitLineRender.positionCount = thisNbodyOrbit.nBodyOrbitPoints.Count- 1;

                for (int i = 0; i < thisNbodyOrbit.nBodyOrbitPoints.Count - 1; i++)
                {
                    Vector3 point = Vector3d.toVector3((thisNbodyOrbit.nBodyOrbitPoints[i] + parentBodyHelioPos) / _planetarium.planetariumScale) + offset;
                    thisNbodyOrbit.orbitLineRender.SetPosition(i, new Vector3(point.x, point.y, point.z) + _planetarium.floatingOriginTransformGamePos);

                }
            }
            
        }


    }

    public void drawRenderLinesTwo_Body()
    {
        foreach (Orbit thisOrbit in _planetarium.celestialObjectOrbit)
        {
            // This only draws for 2 body objects
            if (!thisOrbit.isBodySun && thisOrbit._orbitType == Orbit.orbitType.Two_Body && thisOrbit.orbitLineRender != null)
            {

                thisOrbit.orbitPoints = thisOrbit.GetOrbitPoints(nBodySegmentCount, maxRenderDistance * _planetarium.planetariumScale);
                thisOrbit.orbitLineRender.positionCount = thisOrbit.orbitPoints.Count-1;

                for (int i = 0; i < thisOrbit.orbitPoints.Count - 1; i++)
                {
                    var point = Vector3d.toVector3(thisOrbit.orbitPoints[i] / _planetarium.planetariumScale);

                    thisOrbit.orbitLineRender.SetPosition(i, new Vector3(point.x, point.y, point.z) + _planetarium.floatingOriginTransformGamePos);
                }
            }
        }

    }

    // Gizmo Functions
    private void drawNBodyOrbits()
    {
        foreach (Orbit thisNbodyOrbit in _planetarium.satelliteObjectOrbit)
        {

            // Draw New Line
            Vector3d parentBodyHelioPos = Vector3d.zero;
            if (thisNbodyOrbit._referenceFrame == Orbit.referenceFrame.Heliocentric)
            {
                parentBodyHelioPos = thisNbodyOrbit.parentBodyOrbit.helioCentricPosition;

            }
            if (thisNbodyOrbit._referenceFrame == Orbit.referenceFrame.Parententric)
            {
                // Zero.
            }

            for (int i = 0; i < thisNbodyOrbit.nBodyOrbitPoints.Count - 1; i++)
            {
                Vector3 p1 = Vector3d.toVector3((thisNbodyOrbit.nBodyOrbitPoints[i] + parentBodyHelioPos) / _planetarium.planetariumScale);
                Vector3 p2 = Vector3d.toVector3((thisNbodyOrbit.nBodyOrbitPoints[i + 1] + parentBodyHelioPos) / _planetarium.planetariumScale);

                Gizmos.DrawLine(new Vector3(p1.x, p1.y, p1.z), new Vector3(p2.x, p2.y, p2.z));
            }


        }
    }


    // Drawing Gizmos
    private void drawNBodyOrbitsOld()
    {
        foreach (Orbit thisNbodyOrbit in _planetarium.satelliteObjectOrbit)
        {
            // Draw Old Line

            Vector3d parentBodyHelioPos = Vector3d.zero;
            if (thisNbodyOrbit._referenceFrame == Orbit.referenceFrame.Heliocentric)
            {
                parentBodyHelioPos = thisNbodyOrbit.parentBodyOrbit.helioCentricPosition;

            }
            if (thisNbodyOrbit._referenceFrame == Orbit.referenceFrame.Parententric)
            {
                // Zero.
            }

            for (int i = 0; i < thisNbodyOrbit.nBodyOrbitPointsOld.Count - 1; i++)
            {

                Vector3 p1 = Vector3d.toVector3((thisNbodyOrbit.nBodyOrbitPointsOld[i] + parentBodyHelioPos) / _planetarium.planetariumScale);
                Vector3 p2 = Vector3d.toVector3((thisNbodyOrbit.nBodyOrbitPointsOld[i + 1] + parentBodyHelioPos) / _planetarium.planetariumScale);

                Gizmos.DrawLine(new Vector3(p1.x, p1.y, p1.z), new Vector3(p2.x, p2.y, p2.z));
            }
        }
    }





    
    private void OnDrawGizmos()
    {
        if (Application.isPlaying && drawLines && drawGizmos)
        {
            propogateTwoBodyOrbit();
            drawNBodyOrbits();
            drawNBodyOrbitsOld();
        }
    }



    public void propogateTwoBodyOrbit()
    {
        foreach (Orbit thisOrbit in _planetarium.celestialObjectOrbit)
        {
            // This only draws for 2 body objects
            if (!thisOrbit.isBodySun && thisOrbit._orbitType == Orbit.orbitType.Two_Body)
            {

                thisOrbit.orbitPoints = thisOrbit.GetOrbitPoints(nBodySegmentCount, maxRenderDistance * _planetarium.planetariumScale);

                for (int i = 0; i < thisOrbit.orbitPoints.Count - 1; i++)
                {
                    var p1 = thisOrbit.orbitPoints[i] / _planetarium.planetariumScale;
                    var p2 = thisOrbit.orbitPoints[i + 1] / _planetarium.planetariumScale;

                    Gizmos.DrawLine(new Vector3((float)p1.x, (float)p1.y, (float)p1.z), new Vector3((float)p2.x, (float)p2.y, (float)p2.z));
                }
            }
        }
    }



}


[CustomEditor(typeof(DrawOrbitLines))]
public class DrawOrbitLinesEditor : Editor
{

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        DrawOrbitLines _orbitLines = (DrawOrbitLines)target;

        if (GUILayout.Button("Draw Orbit"))
        {
            _orbitLines.drawOrbits();
        }

        if (GUILayout.Button("Draw Orbit Off"))
        {

            _orbitLines.clearAllOrbitPoints();
        }

    }
}
