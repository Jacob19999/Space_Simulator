using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using static UnityEngine.GraphicsBuffer;

 
[CustomEditor(typeof(GUIDPersistant))]
public class EditorGUID : Editor
{
    
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        GUIDPersistant guidPersistant = (GUIDPersistant)target;

        if (GUILayout.Button("Generate GUID"))
        {
            guidPersistant.setAllGUID();
        }

        if (GUILayout.Button("Get GUID"))
        {
            guidPersistant.getGUIDFromObj();
        }



    }
}