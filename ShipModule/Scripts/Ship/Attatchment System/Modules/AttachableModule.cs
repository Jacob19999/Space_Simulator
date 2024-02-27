using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using UnityEditor;
using UnityEngine;

public class AttachableModule : MonoBehaviour, IAttachable
{
    [SerializeField]
    public string UniqueID;
    [SerializeField]
    public Vector3 partPlacementOffset = Vector3.zero;

    [Header("Module Properties")]
    public string theName = "";
    [SerializeField]
    public int health = 2000;
    [SerializeField]
    public int maxHealth = 2000;

    [Header("Materials & Startup Config")]
    [SerializeField]
    protected bool isAttached;
    [SerializeField]
    private Material highlightMaterial;
    private Rigidbody objectRigidbody;
    private bool isHeld = false;
    public bool isConnected = false;
    public AttachmentPoint connectedToPoint;
    public List<IAttachable> AttachedModules { get; set; }
    private List<Collider> allColliders = new List<Collider>();
    private List<MeshRenderer> allMeshRenderers = new List<MeshRenderer>();
    private Dictionary<MeshRenderer, Material[]> originalMeshMaterials = new Dictionary<MeshRenderer, Material[]>();
    private GameObject _parentGameObject;
    private List<IModule> _parentModule = new List<IModule>();
    public string GUID { get; set; }
    public InertiaTensor _inertiaTensor;

    private GUIDPersistantGenerator GUID_ = new GUIDPersistantGenerator();

    // Public properties related to the module
    public bool IsAttached => isAttached;
    public GameObject GetGameObject => gameObject;
    public string Name
    {
        get => theName;
        set => theName = value;
    }
    
    List<IModule> observerModules { get; set; }

    public int MaxHealth => maxHealth;
    public int Health
    {
        get => health;
        set => health = Mathf.Clamp(value, 0, MaxHealth);
    }
    public bool IsHeld
    {
        get => isHeld;
    }
    public Vector3 getPosition()
    {
        return gameObject.transform.position;
    }
    public Quaternion getRotation()
    {
        return gameObject.transform.rotation;
    }

    public string getParentObjectName()
    {
        _parentGameObject = gameObject.transform.parent.gameObject;

        return _parentGameObject.name;

    }

    public string getPrefabName()
    {
        return UniqueID;
    }
       

    // --------------------------------------------
    public List<AttachmentPoint> getOccupiedAttatchmentPoints()
    {

        AttachmentPoint[] points = gameObject.GetComponentsInChildren<AttachmentPoint>();

        List<AttachmentPoint> output = new List<AttachmentPoint>();

        foreach (AttachmentPoint point in points)
        {
            if (point.ConnectedToModule!= null && point.ParentGameObject == gameObject)
            {
                output.Add(point);
            }
        }
        return output;
    }


    public void subscribe(){

        foreach(IModule module in AttachedModules) {
            parentModule.observerModules.Add(AttachedModules)
        }
    }

    public void Attach(AttachmentPoint attachmentPoint)
    {
        
        isConnected = true;
        connectedToPoint = attachmentPoint;

        AttachmentPoint thisConnectionPoint = FindThisConnectionPoint();

        // Copy the Z rotation from attachmentPoint
        //Vector3 eulerRotation = gameObject.transform.eulerAngles;
        //eulerRotation.z = attachmentPoint.GetTransform.eulerAngles.z;
        //gameObject.transform.rotation = Quaternion.Euler(eulerRotation);


        //Vector3 translation = attachmentPoint.GetTransform.position - thisConnectionPoint.GetTransform.position;
        //gameObject.transform.position += translation;

        // Set the gameObject as a child and set the rigidbody to kinematic
        gameObject.transform.SetParent(attachmentPoint.ParentGameObject.transform);
        gameObject.GetComponent<Rigidbody>().isKinematic = true;


        // Restore original materials and enable colliders
        RestoreOriginalMaterialsAndEnableColliders();

        
        // Get the IModule from the gameobject that attachmentPoint is attached to
        GameObject parentGameObject = attachmentPoint.ParentGameObject;
        _parentGameObject = parentGameObject;
        IModule parentModule = parentGameObject.GetComponent<IModule>();

        // Add this module to the parent module's list of attached modules
        //parentModule.AttachedModules.Add(this);

        // Replaced Observer pattern.
        foreach (IModule module in AttachedModules){
            observerModules.Add(module);
        }


        // Set the Occupied to true for both attachment points
        attachmentPoint.Occupied = true;
        thisConnectionPoint.Occupied = true;

        thisConnectionPoint.ConnectedPoint = attachmentPoint;

        attachmentPoint.ConnectedToModule = this;
        attachmentPoint.ConnectedPoint = thisConnectionPoint;

        attachmentPoint.HideMesh();
        thisConnectionPoint.HideMesh();

        Debug.Log("attatch / update");

        _inertiaTensor._init(false);


    }

    public void Disconnect(bool removeFromList = true)
    {
        if (!isConnected) return;
        AttachmentPoint thisConnectionPoint = FindThisConnectionPoint();

        thisConnectionPoint.Occupied = false;
        connectedToPoint.Occupied = false;

        GameObject parentGameObject = gameObject.transform.parent.gameObject;
        IModule parentModule = parentGameObject.GetComponent<IModule>();
        if (removeFromList)
        {
            parentModule.AttachedModules.Remove(this);
        }

        gameObject.transform.SetParent(null);

        isConnected = false;
        connectedToPoint = null;

        //foreach (IAttachable attachedModule in AttachedModules)
        //{
        //    attachedModule.Eject();
        //}

        // Notification of observer pattern.
        foreach (IModule module in observerModules) {
            observerModules.Eject();
        }

        AttachedModules.Clear();

        Debug.Log("Disconnect / update");

        _inertiaTensor._init(false);
    }

    public void Eject()
    {

        // Access all child modules attatched and call their respective disconnect function.
        foreach (IModule module in observerModules) {
            observerModules.Disconnect();
        }

        Disconnect(false);

        if (objectRigidbody == null) objectRigidbody = GetComponent<Rigidbody>();

        AttachmentPoint thisConnectionPoint = FindThisConnectionPoint();

        if (thisConnectionPoint != null)
        {
            Vector3 ejectDirection = thisConnectionPoint.GetTransform.forward;

            // Define an eject force magnitude
            float ejectForce = -25f; // Adjust this value as needed

            gameObject.GetComponent<Rigidbody>().isKinematic = false;

            // Restore original materials and enable colliders for ejection
            RestoreOriginalMaterialsAndEnableColliders();

            objectRigidbody.AddForce(ejectDirection * ejectForce, ForceMode.Impulse);
        }

        Debug.Log("Disconnect / update");

        _inertiaTensor._init(false);


    }


    private void CollectChildrenComponents(Transform startTransform)
    {
        // Skip objects with an 'AttachmentPoint' class in them
        if (startTransform.GetComponent<AttachmentPoint>() != null)
        {
            return;
        }

        // Check if the current GameObject is in the "Modules" layer, and if so, return
        // This is to prevent the function from collecting components from other modules that might be attached to this one.
        if (startTransform.gameObject.layer == LayerMask.NameToLayer("Modules") && startTransform != transform)
        {
            return;
        }


        // Collect colliders
        Collider collider = startTransform.GetComponent<Collider>();
        if (collider != null)
        {
            allColliders.Add(collider);

        }



        // Collect MeshRenderers
        MeshRenderer meshRenderer = startTransform.GetComponent<MeshRenderer>();
        if (meshRenderer != null)
        {
            allMeshRenderers.Add(meshRenderer);
        }


        // Recursively call the function for all children
        foreach (Transform child in startTransform)
        {
            // Skip children with an 'AttachmentPoint' class before recursion
            if (child.GetComponent<AttachmentPoint>() == null)
            {
                CollectChildrenComponents(child);
            }
        }
    }

    private void ApplyHighlightMaterialsAndDisableColliders()
    {

        foreach (Collider collider in allColliders)
        {
            collider.enabled = false;
        }

        originalMeshMaterials = new Dictionary<MeshRenderer, Material[]>();

        foreach (MeshRenderer renderer in allMeshRenderers)
        {
            originalMeshMaterials[renderer] = renderer.materials;

            Material[] tempMaterials = new Material[renderer.materials.Length];
            for (int i = 0; i < tempMaterials.Length; i++)
            {
                tempMaterials[i] = highlightMaterial;
            }
            renderer.materials = tempMaterials;
        }
    }

    private void RestoreOriginalMaterialsAndEnableColliders()
    {
        foreach (Collider collider in allColliders)
        {
            collider.enabled = true;
        }

        foreach (KeyValuePair<MeshRenderer, Material[]> pair in originalMeshMaterials)
        {
            pair.Key.materials = pair.Value;
        }
    }



    public void PickUp()
    {
        if (isConnected)
        {
            Disconnect();
        }

        isHeld = true;

        if (objectRigidbody == null) objectRigidbody = GetComponent<Rigidbody>();
        objectRigidbody.isKinematic = true;

        allColliders.Clear();
        allMeshRenderers.Clear();
        CollectChildrenComponents(transform);
        ApplyHighlightMaterialsAndDisableColliders();
    }

    public void Release(Vector3 releaseVelocity)
    {
        isHeld = false;

        if (objectRigidbody == null) objectRigidbody = GetComponent<Rigidbody>();
        objectRigidbody.velocity = releaseVelocity;
        objectRigidbody.isKinematic = false;

        RestoreOriginalMaterialsAndEnableColliders();
    }


    public AttachmentPoint FindThisConnectionPoint()
    {
        foreach (Transform child in gameObject.transform)
        {
            AttachmentPoint attachmentPoint = child.GetComponent<AttachmentPoint>();
            if (attachmentPoint != null && attachmentPoint.ConnectionPoint)
            {
                return attachmentPoint;
            }
        }
        return null;
    }

    public void Awake()
    {
        if (GUID == null)
        {
            GUID = GUID_.Generate();
        }

        Rigidbody rb = GetComponent<Rigidbody>();
        if (rb != null)
        {
            //rb.mass = Mass;
        }

        InteractionController controller = InteractionControllerSingleton.Instance.GetComponent<InteractionController>();
        _inertiaTensor = PlayerShipSingleton.Instance.GetComponent<InertiaTensor>();

        highlightMaterial = controller.highlightMaterial;

        AttachedModules = new List<IAttachable>();
        theName = gameObject.name;

    }

    public void FixedUpdate()
    {
        // Potential physics-related calculations here
    }
}

