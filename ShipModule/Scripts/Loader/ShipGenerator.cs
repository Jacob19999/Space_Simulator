using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using Unity.VisualScripting;
using UnityEditor;
using UnityEditor.EditorTools;
using UnityEngine;
using static UnityEditor.ShaderGraph.Internal.KeywordDependentCollection;

public class ShipGenerator : MonoBehaviour
{
    private GUIDPersistant GUID_Persistant;

    [Header("Prefab List")]
    public List<GameObject> prefabs = new List<GameObject>();
    public List<GameObject> instances = new List<GameObject>();

    public Vector3 spawnLocation = Vector3.zero;

    private Vector3 PrevspawnLocation = Vector3.zero;

    public void loadPrefabs()
    {
        GUID_Persistant = gameObject.GetComponent<GUIDPersistant>();

        instances.Clear();

        PrevspawnLocation = spawnLocation;
        foreach (GameObject go in prefabs)
        {
            GameObject instance = Instantiate(go, new Vector3(10, 0, 0), Quaternion.identity);
            instances.Add(instance);

            GUID_Persistant.setGUID(instance);

            Vector3 size = Vector3.zero;

            MeshRenderer renderer = go.GetComponent<MeshRenderer>();

            if (renderer == null)
            {
                size = new Vector3(5,5, 5);

            } else
            {
                size = renderer.bounds.size;
            }
                

            instance.transform.position = PrevspawnLocation;
            PrevspawnLocation.x = PrevspawnLocation.x + size.x + 1;

        }

    }



}
