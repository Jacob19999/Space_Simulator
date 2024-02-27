using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InteractionControllerSingleton : MonoBehaviour
{
    public static InteractionControllerSingleton Instance { get; private set; }

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
