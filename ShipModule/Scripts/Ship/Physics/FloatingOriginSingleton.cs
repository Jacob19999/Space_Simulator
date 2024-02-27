using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FloatingOriginSingleton : MonoBehaviour
{
    public static FloatingOriginSingleton Instance { get; private set; }

    private void Awake()
    {
        if (Instance == null)
        {
            Instance = this;

        }
        else
        {
            Destroy(gameObject);
        }
    }

}


