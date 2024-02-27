using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlayerShipSingleton : MonoBehaviour
{
    public static PlayerShipSingleton Instance { get; private set; }

    private void Awake()
    {
        if (Instance == null)
        {
            Instance = this;
            //DontDestroyOnLoad(gameObject); // This will make the object persist between scenes
        }
        else
        {
            Destroy(gameObject);
        }
    }
}
