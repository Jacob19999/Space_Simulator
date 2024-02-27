using Unity.VisualScripting;
using UnityEditor.Experimental.GraphView;
using UnityEngine;

/// <summary>
/// A point on a game object that can be used to attach other game objects to.
/// </summary>
/// <remarks>
/// This class only contains and modifies state information about the attachment point.
/// </remarks>
public class AttachmentPoint : MonoBehaviour
{
    #region Fields
    [SerializeField]
    public string GUID = "";
    [SerializeField]
    private bool connectionPoint;
    [SerializeField]        
    public string _Parent;
    [SerializeField]   
    public int _layer;
    [SerializeField]
    public GameObject parentGo;
    [SerializeField]
    public Rigidbody parentrb;

    [SerializeField]
    public bool detectColiison = false;

    private GUIDPersistantGenerator GUID_ = new GUIDPersistantGenerator();

    #endregion

    #region Properties

    private MeshRenderer displayMesh;

    /// <summary>
    /// The game object to which this script is attached.
    /// </summary>
    public GameObject GetGameObject => gameObject;

    /// <summary>
    /// The transform for the game object to which this script is attached.
    /// </summary>
    public Transform GetTransform => transform;

    /// <summary>
    /// If this is true, this attachment point is the one that will be used to connect to other attachment points.
    /// </summary>
    /// <remarks>
    /// If a game object has multiple attachment points with this set to true - the first one found will be used.
    /// </remarks>
    public bool ConnectionPoint
    {
        get { return connectionPoint; }
        set { connectionPoint = value; }
    }

    /// <summary>
    /// Whether or not this attachment point is connected to another attachment point.
    /// </summary>
    public bool Occupied { get; set; }

    /// <summary>
    /// The attachment point to which this attachment point is connected.
    /// </summary>
    public AttachmentPoint ConnectedPoint { get; set; }

    /// <summary>
    /// The module to which this attachment point is connected.
    /// </summary>
    public AttachableModule ConnectedToModule { get; set; }

    /// <summary>
    /// Gets the parent of the game object to which this script is attached.
    /// </summary>
    /// 


    public string getLayer()
    {
        return LayerMask.LayerToName(gameObject.layer);

    }
    public GameObject ParentGameObject
    {
        get { return gameObject.transform.parent != null ? gameObject.transform.parent.gameObject : null; }
    }

    #endregion

    #region Unity Methods

    public void Update()
    {
            
        //if (gameObject.name == "AttachmentPoint0 '33097959-5d53-46d4-9798-edfa1a715b40'")
        //{

            //Debug.Log(gameObject + " in layer " + getLayer());

        //}
    }

    private void Awake()
    {
        displayMesh = GetComponent<MeshRenderer>();

        HideMesh();
        setStartingMaterial();

    }

    private void OnDrawGizmos()
    {
        Gizmos.DrawIcon(transform.position, "attachmentIcon.png", true);
        if (connectionPoint)
        {
            Gizmos.color = new Color(1f, 0f, 1f, 1f);
            Gizmos.DrawWireSphere(transform.position, 0.1f);
        }
        else
        {
            Gizmos.color = Color.green;
            Gizmos.DrawWireSphere(transform.position, 0.1f);
        }
    }

    #endregion

    public void setStartingMaterial()
    {
        Material greenGlow = displayMesh.GetComponent<Renderer>().material;
        greenGlow.color = Color.green;
        greenGlow.EnableKeyword("_EMISSION");
        greenGlow.SetColor("_EmissionColor", Color.green);
    }
    public void hoveringMaterial()
    {
        Material redGlow = displayMesh.GetComponent<Renderer>().material;
        redGlow.color = Color.red;
        redGlow.EnableKeyword("_EMISSION");
        redGlow.SetColor("_EmissionColor", Color.red);
        

    }


    public void ShowMesh()
    {
        displayMesh.enabled = true;

    }

    public void HideMesh()
    {
        displayMesh.enabled = false;
    }
}

