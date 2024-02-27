using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(ShipThrottle))]
public class ShipThrottleDebug : Editor
{

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        ShipThrottle throttle1 = (ShipThrottle)target;

        if (GUILayout.Button("Update"))
        {
            throttle1.getEngines();

        }




    }
}
