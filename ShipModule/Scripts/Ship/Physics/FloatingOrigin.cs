using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class FloatingOrigin : MonoBehaviour
{

    public List<GameObject>  floatingObjectsShips;
    public List<ThisFloatingObject> floatingObjectsScript;
    public float activeThresholdStation = 100000f;
    public float activeThresholdShips = 10000f;

    private GameObject playerShip;
    private Vector3 playerShipPosition;
    public Vector3d worldPosPlayer; 
    private InertiaTensor _inertiaTensor;
    

    void Awake()
    {
        if (floatingObjectsShips == null)
        {
            //floatingObjectsShips = GameObject.FindGameObjectsWithTag("FloatingObjectShips");
        }

        playerShip = PlayerShipSingleton.Instance.gameObject;
        playerShipPosition = playerShip.transform.position;

    }

    public void setInitialOffset()
    {

        foreach (GameObject go in floatingObjectsShips)
        {
            Debug.Log(go.name);
            ThisFloatingObject thisObject = go.GetComponent<ThisFloatingObject>();
            floatingObjectsScript.Add(thisObject);
            thisObject.initialOffset = thisObject.editorPos - playerShipPosition;
        }

    }

    public void addObject(GameObject Go)
    {

        floatingObjectsShips.Add(Go);
        ThisFloatingObject thisObject = Go.GetComponent<ThisFloatingObject>();
        floatingObjectsScript.Add(thisObject);
        thisObject.initialOffset = Go.transform.position;

    }

    // Update is called once per frame
    public void updatePosition()
    {
        if (_inertiaTensor == null){

            _inertiaTensor = playerShip.GetComponent<InertiaTensor>();
            _inertiaTensor._init(true);

        }

        foreach (ThisFloatingObject thisfloatingObj in floatingObjectsScript)
        {

            thisfloatingObj.worldPosPlayer = worldPosPlayer;

            if (thisfloatingObj.gameObject.tag == "FloatingObjectStation")
            {
                if (Vector3d.Distance(thisfloatingObj.worldPos, _inertiaTensor.worldPosition) < activeThresholdStation)
                {
                    thisfloatingObj.setObjActive();
                }
                else
                {
                    thisfloatingObj.setInactive();
                }

            } else if (thisfloatingObj.gameObject.tag == "FloatingObjectShips")
            {
                if (Vector3d.Distance(thisfloatingObj.worldPos, _inertiaTensor.worldPosition) < activeThresholdShips)
                {
                    thisfloatingObj.setObjActive();
                }
                else
                {
                    thisfloatingObj.setInactive();
                }
            }
        }
    }
}
