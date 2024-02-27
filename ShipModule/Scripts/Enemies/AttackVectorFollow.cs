using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class AttackVectorFollow : MonoBehaviour
{

    void Update()
    {
        transform.position = PlayerShipSingleton.Instance.transform.position;

    }
}
