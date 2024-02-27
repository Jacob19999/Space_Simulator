using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ThisFloatingObject : MonoBehaviour
{
    public Vector3 editorPos;
    public Vector3 gamePosition;
    public Vector3 initialOffset;

    public Vector3d worldPosPlayer;
    public Vector3d worldPos;
    public Vector3 worldPosVector3;

    public InertiaTensor _inertiaTensor;

    public bool ObjectState = true;

    void Awake()
    {
        ObjectState = true;
        editorPos = transform.position;

        _inertiaTensor = gameObject.GetComponent<InertiaTensor>();

    }

    private void FixedUpdate()
    {

        if (ObjectState)
        {

            if (_inertiaTensor != null)
            {
                gamePosition = initialOffset - Vector3d.toVector3(worldPosPlayer) + Vector3d.toVector3(_inertiaTensor.worldPosition);

            } else
            {
                gamePosition = initialOffset - Vector3d.toVector3(worldPosPlayer);
            }

            worldPos = Vector3d.toVector3d(gamePosition);
            transform.position = gamePosition;

        }

        worldPosVector3 = Vector3d.toVector3(worldPos);

    }


    public void setInactive()
    {
        ObjectState = false;
        gameObject.SetActive(false);
        
    }

    public void setObjActive()
    {
        ObjectState = true;
        gameObject.SetActive(true);

    }


}
