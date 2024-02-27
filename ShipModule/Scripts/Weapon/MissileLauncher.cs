using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.WSA;

public class MissileLauncher : MonoBehaviour
{
    [Header("General Parameters:")]
    [Tooltip("Gets from ship target manager.")]
    public Transform target;

    [Tooltip("Number of missiles available.")]
    public float magazineSize = 4;

    [Tooltip("Prefab of missile to be launched.")]
    public GameObject missilePrefab;

    //public Transform launchPoint;

    //public List<Missile> missileList;
    private TargetManager playerShipTargetManager;
    

    // Start is called before the first frame update
    void Start()
    {
        
    }

    // Update is called once per frame
    void Update()
    {
        if (playerShipTargetManager == null)
        {
            playerShipTargetManager = PlayerShipSingleton.Instance.gameObject.GetComponent<TargetManager>();
        }

        target = playerShipTargetManager.target;

        if (Input.GetKeyDown(KeyCode.L))
        {
            CreateMissile();
            magazineSize = magazineSize - 1;
        }
    }

    private Missile CreateMissile()
    {
        GameObject newMissile = Instantiate(missilePrefab, transform);

        //FloatingOriginSingleton.Instance.gameObject.GetComponent<FloatingOrigin>().addObject(newMissile);

        Missile newMissileScript =  newMissile.GetComponent<Missile>();

        newMissileScript.target = target;
        newMissileScript.ownShip = transform.root;

        // Attach the missile to the hardpoint by its attach point if possible.
        //if (newMissileScript.attachPoint != null)
        //{
        //    newMissileScript.transform.localPosition = -newMissileScript.attachPoint.localPosition;
        //    newMissileScript.transform.localEulerAngles = -newMissileScript.attachPoint.localEulerAngles;
        //}
        //else
        //{
        //    newMissileScript.transform.localPosition = Vector3.zero;
        //    newMissileScript.transform.localEulerAngles = Vector3.zero;
        //}

        newMissileScript.Launch(target);

        return newMissileScript;
    }




}
