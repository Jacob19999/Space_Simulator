using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DockingPort : MonoBehaviour
{
    [Header("Objects")]
    public GameObject thisDockingPort;
    public GameObject playerShip;
    public SpaceShip shipController;
    public ShipInput shipInput;
    public InertiaTensor inertiaTensor;
    public DockingPort otherDockingPort;

    [Header("Docking Port Settings")]
    public Vector3 portForwardVector = Vector3.zero;
    public portType _PortType;

    public enum portType
    {
        passive,
        active
    }

    // Start is called before the first frame update
    void Awake()
    {
        playerShip = PlayerShipSingleton.Instance.gameObject;
        shipController = playerShip.GetComponent<SpaceShip>();
        inertiaTensor = playerShip.GetComponent<InertiaTensor>();
        shipInput = playerShip.GetComponent<ShipInput>();

    }

    // Update is called once per frame
    void Update()
    {
        portForwardVector = transform.TransformDirection(Vector3.up) * 50f;

        if (thisDockingPort && _PortType == portType.active)
        {
            rayCastFromPort();
        }

    }

    private void rayCastFromPort()
    {
        RaycastHit objectHit;

        int dockingPortLayer = LayerMask.NameToLayer("DockingPort");
        int layerMask = 1 << dockingPortLayer;

        // Send out raycast
        if (Physics.Raycast(thisDockingPort.transform.position, portForwardVector, out objectHit, 10f, layerMask))
        {

            Vector3 distanceVector = thisDockingPort.transform.position - objectHit.collider.transform.position;
            float distance = distanceVector.magnitude;

            // Check for alignment.
            otherDockingPort = objectHit.collider.transform.GetComponent<DockingPort>();
            float portAlignment = Vector3.Angle(portForwardVector, -otherDockingPort.portForwardVector);
            Vector3 pitchRollyaw = quaternionPitchYawRoll(thisDockingPort.transform.rotation, otherDockingPort.transform.rotation);
            Debug.Log("Docking Port " + objectHit.collider.transform.name + " distance = " + distance + " vector Dist = " + distanceVector + " alignment Deg = " + pitchRollyaw.y * Mathf.Rad2Deg + " Angle " + portAlignment);

            // Docking Parameters
            if (Mathf.Abs (pitchRollyaw.y * Mathf.Rad2Deg) < 15 && portAlignment < 15 && distance < 2f)
            {
                Debug.Log("DockingPort Successful");
            }

        }
    }


    private void OnDrawGizmos()
    {

        Gizmos.color = Color.red;
        Vector3 direction = transform.TransformDirection(Vector3.up) * 10f;
        Gizmos.DrawRay(transform.position , direction);

    }


    private static Vector3 quaternionPitchYawRoll(Quaternion q1, Quaternion q2)
    {

        Quaternion qd = Conjugate(q1) * q2;

        float _phi = Mathf.Atan2(2f * (qd.w * qd.x + qd.y * qd.z), 1f - 2f * (Mathf.Pow(qd.x, 2f) + Mathf.Pow(qd.y, 2f)));
        float _theta = Mathf.Asin(2f * (qd.w * qd.y - qd.z * qd.x));
        float _psi = Mathf.Atan2(2f * (qd.w * qd.z + qd.x * qd.y), 1f - 2f * (Mathf.Pow(qd.y, 2f) + Mathf.Pow(qd.z, 2f)));

        return new Vector3(_phi, _theta, _psi);

    }
    private static Quaternion Conjugate(Quaternion value)
    {
        Quaternion ans;

        ans.x = -value.x;
        ans.y = -value.y;
        ans.z = -value.z;
        ans.w = value.w;

        return ans;
    }



}

