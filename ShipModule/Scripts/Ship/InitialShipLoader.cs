using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class InitialShipLoader : MonoBehaviour
{

    public ShipLoader _loader;

    // Start is called before the first frame update
    void Awake()
    {

        _loader = gameObject.GetComponent<ShipLoader>();
        StartCoroutine(DelayedExecution(0.5f));
    }

    private IEnumerator DelayedExecution(float seconds_to_wait)
    {
        yield return new WaitForSeconds(seconds_to_wait);

        _loader.loadShipData();

    }


}
