using System;
using System.Collections;
using System.Collections.Generic;
using System.Drawing;
using UnityEngine;
using UnityEngine.UIElements;

public class InteractionController : MonoBehaviour
{
    #region Fields

    [Header("Camera & Target (leave Empty)")]
    // Camera references
    public bool globalEnable = false;
    public bool sessionInit = false;

    [SerializeField]
    public bool alignedToAttatchementPoint = false;

    [SerializeField]
    private OrbitCam mainCameraController;
    public Camera mainCamera;
    /// <summary>
    /// The transform of the object that the camera is looking at.
    /// </summary>
    [SerializeField]
    private Transform targetTransform;
    /// <summary>
    /// The object that the camera is looking at.
    /// </summary>
    [SerializeField]
    private GameObject targetObject;

    [SerializeField]
    public GameObject editModeButton;

    //[Header("Startup Config")]


    public bool holdingObject = false;


    private bool hoveringAttachmentPoint = false;
    private AttachmentPoint lastHoveredAttachmentPoint = null;
    private List<AttachmentPoint> validAttachmentPoints = new List<AttachmentPoint>();

    private IAttachable heldObject = null;
    private GameObject heldObjectGameObject = null;
    private Rigidbody heldObjectRigidbody = null;

    [Header("Held Effects")]
    // Object highlighting
    public Material highlightMaterial;
    public Material attachmentPointHighlight;
    public Material attachmentPointActiveHighlight;

    [Header("Release Parameters")]
    // Object release
    [SerializeField]
    private int velocityFrameCount = 5;
    [SerializeField]
    private float maxVelocityMagnitude = 0.5f;
    private Queue<Vector3> positionHistory = new Queue<Vector3>();
    private Vector3 releaseVelocity;
    public int reachDistance = 30; // The maximum distance from the target (usually the player ship) that other modules can be interacted with

    #endregion

    #region Properties

    private Vector3 objectOrientAdjust = Vector3.zero;
    public IAttachable HeldObject
    {
        get => heldObject;
        set
        {
            heldObject = value;
            heldObjectGameObject = heldObject != null ? ((Component)heldObject).gameObject : null;
            heldObjectRigidbody = heldObjectGameObject?.GetComponent<Rigidbody>();
        }
    }

    public void stopInteraction()
    {
        validAttachmentPoints.Clear();
        sessionInit = false;
        globalEnable = false;
        editModeButton.GetComponent<ShipBuilderMode>().modeSelector(false);

    }

    public void startInteraction()
    {
        validAttachmentPoints.Clear();
        sessionInit = false;
        globalEnable = true;
        editModeButton.GetComponent<ShipBuilderMode>().modeSelector(true);

    }

    public OrbitCam MainCameraController
    {
        get => mainCameraController;
        set
        {
            mainCameraController = value;
            mainCamera = mainCameraController.GetComponent<Camera>();
            targetObject = mainCameraController.TargetObject;
            targetTransform = targetObject.transform;
        }
    }

    #endregion

    #region MonoBehaviour Callbacks

    private void OnDestroy()
    {
        //OnObjectHeldStateChanged -= HandleObjectHeldStateChange;
    }

    private void Update()
    {
        if (globalEnable && sessionInit == false )
        {

            // Init, only runs once .
            if (MainCameraController == null)
            {
                MainCameraController = MainCameraSingleton.Instance.GetComponent<OrbitCam>();
            }

            // Find all attatchment points
            validAttachmentPoints = PlayerShipSingleton.Instance.GetComponent<CommandModule>().GetAllAttachmentPoints();

            sessionInit = true;
        }



        if (globalEnable)
        {
           
            HandleMouseInput();


            if (holdingObject)
            {
                HandleAttachmentPointHover();
                HeldObjectUpdate();
            }
        }
    }


    #endregion

    #region Keyboard Inputs
    private void keyboardInput()
    {

        // Check if the specified key is pressed down
        if (Input.GetKeyDown(KeyCode.W))
        {
            // Pitch Object
            objectOrientAdjust.x = 10;
        }
        if (Input.GetKeyDown(KeyCode.S))
        {
            // Pitch Object
            objectOrientAdjust.x = -10;
        }

        if (Input.GetKeyDown(KeyCode.D))
        {
            // Yaw Object 
            objectOrientAdjust.y = 10;
        }
        if (Input.GetKeyDown(KeyCode.A))
        {
            // Yaw Object 
            objectOrientAdjust.y = -10;
        }

        if (Input.GetKeyDown(KeyCode.E))
        {
            // Yaw Object 
            objectOrientAdjust.z = 10;
        }
        if (Input.GetKeyDown(KeyCode.Q))
        {
            // Yaw Object 
            objectOrientAdjust.z = -10;
        }

    }




    #endregion

    #region Private Methods

    private void HandleAttachmentPointHover()
    {

        Ray ray1 = Camera.main.ScreenPointToRay(Input.mousePosition);
        RaycastHit hit;
        bool isHit = false;

        int attachmentPointsLayer = LayerMask.NameToLayer("AttachmentPoints");
        int layerMask = 1 << attachmentPointsLayer;

        if (Physics.Raycast(ray1, out hit, 1000, layerMask) )
        {
            
            AttachmentPoint attachmentPoint = hit.collider.GetComponent<AttachmentPoint>();
            
            if (attachmentPoint != null && attachmentPoint.ParentGameObject.name != heldObjectGameObject.name)
            {
                isHit = true;
                hoveringAttachmentPoint = true;
                if (lastHoveredAttachmentPoint != attachmentPoint)
                {
                    // Reset the material of the previously hovered attachment point, if there was one.
                    if (lastHoveredAttachmentPoint != null)
                    {
                        lastHoveredAttachmentPoint.hoveringMaterial();

                    }

                    // Change the material of the currently hovered attachment point.
                    attachmentPoint.hoveringMaterial();
                    lastHoveredAttachmentPoint = attachmentPoint;
                    
                    //Debug.Log("lastHovered = " + lastHoveredAttachmentPoint.gameObject.name);
                }
            }
        }


        if (!isHit)
        {
            //Debug.Log("No Hit");
            hoveringAttachmentPoint = false;
            // If not hovering over any valid attachment point, reset the last hovered one.
            if (lastHoveredAttachmentPoint != null)
            {
                lastHoveredAttachmentPoint.setStartingMaterial();
                lastHoveredAttachmentPoint = null;
                alignedToAttatchementPoint = false;
            }
        }

        


    }


    private void HandleMouseInput()
    {
        if (Input.GetMouseButtonDown(0) && !holdingObject)
        {
            PickUpObject();
            

        }
        else if (Input.GetMouseButtonUp(0) && holdingObject)
        {
            ReleaseObject();
        }
    }

    private void PickUpObject()
    {
        //Debug.Log(debugObject.name + " in layer0 " + debugObject.GetComponent<AttachmentPoint>().getLayer());

        Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
        RaycastHit hit;

        if (Physics.Raycast(ray, out hit))
        {

            GameObject thisModule = FindModuleInHierarchy(hit.transform);
            if (thisModule == null)
            {
                Debug.LogError("Could not find module in hierarchy for object: " + hit.transform.name);
                return;
            }


            IAttachable attachable = thisModule.GetComponent<IAttachable>();
            if (attachable != null)
            {
                float distanceToTarget = Vector3.Distance(hit.transform.position, targetTransform.position);

                if (distanceToTarget <= reachDistance)
                {
                    
                    HeldObject = attachable;
                    HeldObject.PickUp();

                    holdingObject = true;

                    // Change material of all attatchment empty Points.
                    validAttachmentPoints = PlayerShipSingleton.Instance.GetComponent<CommandModule>().GetAllAttachmentPoints();
                    foreach (var point in validAttachmentPoints) {

                        point.ShowMesh();

                    }
                    
                }
                
            }
            else
            {
                Debug.Log($"Object '{thisModule.name}' identified as a module but does not have an IAttachable component. Original hit was: '{hit.transform.name}'");
            }
        }

    }

    public GameObject FindModuleInHierarchy(Transform startTransform)
    {
        if (startTransform == null)
        {
            return null; // Reached the top of the hierarchy without finding a module
        }

        if (startTransform.gameObject.layer == LayerMask.NameToLayer("Modules"))
        {
            return startTransform.gameObject;
        }

        return FindModuleInHierarchy(startTransform.parent);
    }

    private void ReleaseObject()
    {
        if(hoveringAttachmentPoint && lastHoveredAttachmentPoint != null)
        {
            HeldObject.Attach(lastHoveredAttachmentPoint);
            
        }
        else
        {
            HeldObject.Release(releaseVelocity);
        }
        HeldObject = null;
        holdingObject = false;

        foreach (var point in validAttachmentPoints)
        {

            point.HideMesh();

        }

    }

    private void HeldObjectUpdate()
    {

        if (!holdingObject || HeldObject == null) return;

        //if (!ObjectHeld || HeldObject == null) return;

        Ray ray = mainCamera.ScreenPointToRay(Input.mousePosition);
        if (mainCameraController.InteractionPlane.Raycast(ray, out float planeDistance))
        {
            UpdateObjectPositionAndVelocity(ray, planeDistance);
        }



    }

    private void UpdateObjectPositionAndVelocity(Ray ray, float planeDistance)
    {
        Vector3 hitPoint = ray.GetPoint(planeDistance);

        GameObject holdingObject = heldObject.GetGameObject;
        AttachmentPoint thisConnectionPoint = heldObject.FindThisConnectionPoint();
        if (thisConnectionPoint == null)
        {
            return;
        }

        Vector3 offsetHeldObjectPoint = holdingObject.transform.position - thisConnectionPoint.gameObject.transform.position;
        Vector3 dragPoint = hitPoint + offsetHeldObjectPoint;

        // Update position history
        positionHistory.Enqueue(hitPoint);
        if (positionHistory.Count > velocityFrameCount)
        {
            positionHistory.Dequeue();
        }

        // Calculate the release velocity based on the averaged movement
        if (positionHistory.Count == velocityFrameCount)
        {
            releaseVelocity = (hitPoint - positionHistory.Peek()) / (velocityFrameCount * Time.deltaTime) ;
            releaseVelocity = Vector3.ClampMagnitude(releaseVelocity, maxVelocityMagnitude) + PlayerShipSingleton.Instance.GetComponent<InertiaTensor>().worldVelocity;

        }

        // Check if object is beyond reach and release if necessary
        if (Vector3.Distance(hitPoint, targetTransform.position) > reachDistance)
        {
            ReleaseObject();
            return;
        }

        // If hovering over an attachment point, align the object with the attachment point
        if (hoveringAttachmentPoint && lastHoveredAttachmentPoint != null)
        {
            AlignWithAttachmentPoint(objectOrientAdjust);
        }
        else
        {

            // If not hovering over an attachment point, allow normal dragging behavior
            heldObjectGameObject.transform.position = dragPoint;
        }


    }

    private void AlignWithAttachmentPoint(Vector3 objectOrientAdjust)
    {

        //Debug.Log("AlignWithAttachmentPoint");

        if (lastHoveredAttachmentPoint == null || heldObject == null)
            return;

        AttachmentPoint thisConnectionPoint = heldObject.FindThisConnectionPoint();
        if (thisConnectionPoint == null)
            return;

        
        //Debug.Log("Last Held Point = " + lastHoveredAttachmentPoint + " This Point = "+ thisConnectionPoint + " held object "+ heldObjectGameObject);

        if (alignedToAttatchementPoint == false)
        {
            Transform holdingObject = heldObject.GetGameObject.transform;
            Transform parentObject = lastHoveredAttachmentPoint.ParentGameObject.transform;

            Vector3 translation = lastHoveredAttachmentPoint.GetTransform.position - thisConnectionPoint.GetTransform.position;
            heldObjectGameObject.transform.position += translation;

            // Set initial rotation based on command module
            heldObjectGameObject.transform.rotation = parentObject.transform.rotation * Quaternion.Euler(heldObject.GetGameObject.GetComponent<AttachableModule>().partPlacementOffset);

            // Align axis with attatchment point
            Quaternion rotation = Quaternion.FromToRotation(-thisConnectionPoint.transform.forward, lastHoveredAttachmentPoint.GetTransform.forward);
            heldObjectGameObject.transform.rotation = rotation * holdingObject.rotation;

            alignedToAttatchementPoint = true;

        } else
        {
            // After initial alignment, user can change object orientation
            if (heldObject != null)
            {
                // Rotate object based on keyboard input
                heldObjectGameObject.transform.RotateAround(lastHoveredAttachmentPoint.transform.position, lastHoveredAttachmentPoint.transform.up, objectOrientAdjust.y);
                heldObjectGameObject.transform.RotateAround(lastHoveredAttachmentPoint.transform.position, lastHoveredAttachmentPoint.transform.forward, objectOrientAdjust.x);
                heldObjectGameObject.transform.RotateAround(lastHoveredAttachmentPoint.transform.position, lastHoveredAttachmentPoint.transform.right, objectOrientAdjust.z);
                objectOrientAdjust = Vector3.zero;
            }
        }
    }



    #endregion

}

