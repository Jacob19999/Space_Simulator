using System.Collections;
using System.Collections.Generic;
using System.Threading;
using Unity.VisualScripting;
using UnityEditor.XR.LegacyInputHelpers;
using UnityEngine;
using UnityEngine.UIElements;
using static UnityEngine.GraphicsBuffer;

public class OrbitCam : MonoBehaviour
{


    private Transform targetTransform; // The target object for the camera to look at
    [SerializeField]
    public GameObject targetObject;

    [SerializeField]
    private bool applyConstraints = false;

    private Camera thisCamera;
    private Vector3 prevCamPos;

    // Our desired distance from the target object.
    [SerializeField]
    private float _distance = 5;

    [SerializeField]
    private float _damping = 2;

    [SerializeField]
    private float scrollMultiplyerMax = 100f;
    public float scrollMultiplyer = 1f;

    // These will store our currently desired angles
    private Quaternion _pitch;
    private Quaternion _yaw;

    // this is where we want to go.
    private Quaternion _targetRotation;
    private Vector3 _targetPosition;
    


    public float Yaw
    {
        get { return _yaw.eulerAngles.y; }
        private set { _yaw = Quaternion.Euler(0, value, 0); }
    }

    public float Pitch
    {
        get { return _pitch.eulerAngles.x; }
        private set { _pitch = Quaternion.Euler(value, 0, 0); }
    }

    public void Move(float yawDelta, float pitchDelta)
    {
        _yaw = _yaw * Quaternion.Euler(0, yawDelta, 0);
        _pitch = _pitch * Quaternion.Euler(pitchDelta, 0, 0);

        if (applyConstraints)
        {
            ApplyConstraints();
        }
        
    }

    private void ApplyConstraints()
    {
        Quaternion targetYaw = Quaternion.Euler(0, targetTransform.rotation.eulerAngles.y, 0);
        Quaternion targetPitch = Quaternion.Euler(targetTransform.rotation.eulerAngles.x, 0, 0);

        float yawDifference = Quaternion.Angle(_yaw, targetYaw);
        float pitchDifference = Quaternion.Angle(_pitch, targetPitch);

        float yawOverflow = yawDifference - 90f;
        float pitchOverflow = pitchDifference - 90f;

        // We'll simply use lerp to move a bit towards the focus target's orientation. Just enough to get back within the constraints.
        // This way we don't need to worry about wether we need to move left or right, up or down.
        if (yawOverflow > 0) { _yaw = Quaternion.Slerp(_yaw, targetYaw, yawOverflow / yawDifference); }
        if (pitchOverflow > 0) { _pitch = Quaternion.Slerp(_pitch, targetPitch, pitchOverflow / pitchDifference); }
    }


    public GameObject TargetObject
    {
        get
        {
            return targetObject;
        }
        set
        {
            targetObject = value;
            targetTransform = targetObject.transform;
        }
    }

    public Plane InteractionPlane
    {
        get
        {
            if (targetTransform == null) return new Plane();

            Vector3 planeNormal = thisCamera.transform.forward;
            Vector3 planePoint = targetTransform.position; // This sets the plane to intersect the camera's target.
            return new Plane(planeNormal, planePoint);
        }
    }

    private float planeDistance = 0.0f; // The distance from the camera to the interaction plane

    public float PlaneDistance
    {
        get
        {
            return planeDistance;
        }
    }

    private void Awake()
    {
        if (TargetObject == null)
        {
            TargetObject = PlayerShipSingleton.Instance.gameObject;
        }
        thisCamera = GetComponent<Camera>();
        if (thisCamera == null)
        {
            Debug.LogError("CameraController is attached to an object without a Camera component!", this);
        }

        // initialise our pitch and yaw settings to our current orientation.

        _pitch = Quaternion.Euler(  this.transform.rotation.eulerAngles.x, 0, 0);
        _yaw = Quaternion.Euler(0, this.transform.rotation.eulerAngles.y, 0);
        
    }



    private void Update()
    {
        targetTransform = targetObject.transform;

        // set the distance from the camera to the interaction plane
        InteractionPlane.Raycast(thisCamera.ScreenPointToRay(Input.mousePosition), out planeDistance);

        float scrollInput = Input.GetAxis("Mouse ScrollWheel");
        _distance -= scrollInput * 4.0f * scrollMultiplyer;
        _distance = Mathf.Clamp(_distance , 2.0f, 100000f);

        _targetRotation = _yaw * _pitch;
        _targetPosition = targetTransform.position + _targetRotation * (-Vector3.forward * _distance);

        this.transform.rotation = Quaternion.Lerp(this.transform.rotation, _targetRotation, Mathf.Clamp01(Time.smoothDeltaTime * _damping));

        // offset the camera at distance from the target position.
        Vector3 offset = this.transform.rotation * (-Vector3.forward * _distance);
        this.transform.position = targetTransform.position + offset;


        if (_distance > 1000)
        {
            scrollMultiplyer = scrollMultiplyerMax ;
        } 
        if (_distance < 1000 && _distance < 500)
        {
            scrollMultiplyer = scrollMultiplyerMax * 0.5f;
        }
        if (_distance < 500 && _distance < 200)
        {
            scrollMultiplyer = scrollMultiplyerMax * 0.3f;
        }
        if (_distance < 200 && _distance < 100)
        {
            scrollMultiplyer = scrollMultiplyerMax * 0.1f;
        }
        if (_distance < 100)
        {
            scrollMultiplyer = scrollMultiplyerMax * 0.05f;
        }
        if (_distance < 50)
        {
            scrollMultiplyer = scrollMultiplyerMax * 0.01f;
        }
    }

}
