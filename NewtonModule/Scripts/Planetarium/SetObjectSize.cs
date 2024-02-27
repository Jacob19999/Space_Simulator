using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEditor;

public class SetObjectSize : MonoBehaviour
{
    [Header("Object Size To Set")]
    public GameObject thisObject;
    public double scale = 100000f;
    public double actualSize = 1;

    public void updateSize()
    {
        thisObject.transform.localScale = Vector3d.toVector3( new Vector3d((actualSize / scale),(actualSize / scale), (actualSize / scale)));

    }

}



[CustomEditor(typeof(SetObjectSize))]
public class SetObjectSizeDebug : Editor
{

    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        SetObjectSize _size = (SetObjectSize)target;

        if (GUILayout.Button("Update Size"))
        {
            _size.updateSize();
        }
    }
}
