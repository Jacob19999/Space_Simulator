using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

public class CommandModule : MonoBehaviour, IModule
{
    [SerializeField]
    public string GUID = "";
    [SerializeField]
    public string UniqueID;
    [Header("Module Properties")]
    [SerializeField]
    private string theName;
    [SerializeField]
    private int health;
    [SerializeField]
    private int maxHealth;

    private GUIDPersistantGenerator GUID_ = new GUIDPersistantGenerator();

    [SerializeField]
    public List<IAttachable> AttachedModules { get; set; }

    public GameObject GetGameObject => gameObject; // Retrieve the GameObject this MonoBehaviour is attached to
    

    public string getPrefabName()
    {
        return UniqueID;
    }


    public string Name
    {
        get => theName;
        set => theName = value;
    }

    public int MaxHealth
    {
        get => maxHealth;
    }

    public int Health
    {
        get => health;
        set => health = Mathf.Clamp(value, 0, MaxHealth);
    }

    public CommandModule()
    {
        Name = "Command Module Test";
        maxHealth = 100;
        Health = MaxHealth;
    }

    public Vector3 getPosition()
    {
        return gameObject.transform.position;
    }

    public Quaternion getRotation()
    {
        return gameObject.transform.rotation;
    }

    public List<AttachmentPoint> getOccupiedAttatchmentPoints()
    {

        AttachmentPoint[] points = gameObject.GetComponentsInChildren<AttachmentPoint>();

        List <AttachmentPoint> output = new List <AttachmentPoint>();

        foreach (AttachmentPoint point in points)
        {
            if (point.Occupied && point.ParentGameObject == gameObject)
            {
                output.Add(point);
            }
        }

        return output;
    }
    public List<AttachmentPoint> GetAllAttachmentPoints()
    {
        AttachmentPoint[] points = FindObjectsOfType<AttachmentPoint>();

        List<AttachmentPoint> output = new List<AttachmentPoint>();

        foreach (AttachmentPoint point in points)
        {
            if (point.Occupied == false)
            {
                output.Add(point);
            }
        }

        return output;

    }

    public List<AttachableModule> GetAllModulesPoints()
    {
        return GetGameObject.GetComponentsInChildren<AttachableModule>()
                   .ToList();

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

        List<AttachmentPoint> attachmentPoints = GetAllAttachmentPoints();
        foreach (AttachmentPoint attachmentPoint in attachmentPoints)
        {
            //attachmentPoint.GetGameObject.layer = LayerMask.NameToLayer("AttachmentPoints");
        }

        AttachedModules = new List<IAttachable>();


    }
}

