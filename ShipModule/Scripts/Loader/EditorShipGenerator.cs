using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using static UnityEngine.GraphicsBuffer;

 
[CustomEditor(typeof(ShipGenerator))]
public class EditorShipGenerator : Editor
{
    
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        ShipGenerator generator = (ShipGenerator)target;

        if (GUILayout.Button("Load into Scene"))
        {
            generator.loadPrefabs();

        }

  
    }
}