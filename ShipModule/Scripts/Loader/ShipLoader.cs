using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;
using UnityEngine.UI;
using Unity.VisualScripting;
using UnityEditor;
using System;
using static Unity.VisualScripting.Metadata;
using System.Drawing;
using System.Reflection;
using static UnityEditor.Progress;

public class ShipLoader : MonoBehaviour
{

    [SerializeField]
    public Button saveShipButton;
    public Button loadShipButton;
    private GameObject playerShip;
    private InteractionController interactionController;
    private InertiaTensor _inertiaTensor;

    [Header("Prefab List")]
    public List<GameObject> prefabs = new List<GameObject>();

    private string FileName;
    private shipData _shipData;
    private TextAsset jsonFile;
    
    private CommandModule commandModuleScript;

    private Vector3 shipPosition = Vector3.zero;
    private Quaternion shipRotation = Quaternion.identity;

    private GUIDPersistant GUIDPersistant_;
    private GUIDPersistantGenerator GUIDObj;

    void Start()
    {
        
        FileName = "ShipSaves.json";
        //Application.persistentDataPath

        saveShipButton.onClick.AddListener(SaveIntoJson);
        loadShipButton.onClick.AddListener(loadShipData);


    }


    public void SaveIntoJson()
    {
        _init();

        // Make sure GUID is set for each object before saving.
        GUIDPersistant_.setGUID(playerShip);

        // Get Ship save data
        commandModuleScript = playerShip.GetComponent<CommandModule>();
        _shipData = new shipData(commandModuleScript);

        _shipData.getShipSaveData();

        // Save data into JSON
        string saveDataJSON = JsonUtility.ToJson(_shipData);
        Debug.Log(Application.dataPath + FileName);
        System.IO.File.WriteAllText(Application.dataPath + FileName, saveDataJSON);

    }

    public void _init()
    {
        playerShip = PlayerShipSingleton.Instance.gameObject;
        interactionController = InteractionControllerSingleton.Instance.gameObject.GetComponent<InteractionController>();
        _inertiaTensor = PlayerShipSingleton.Instance.gameObject.GetComponent<InertiaTensor>();
        _inertiaTensor.disableTensor();

        GUIDPersistant_ = gameObject.GetComponent<GUIDPersistant>();
        _inertiaTensor.resume = false;
        GUIDObj = new GUIDPersistantGenerator();
        
    }

    public void loadShipData()
    {
        Debug.Log("Load Ship Data");

        _init();

        List<GameObject> instantiatedModules = new List<GameObject>();

        CommandModule commandModuleScript = playerShip.GetComponent<CommandModule>();
        GUIDPersistant commandModuleGuidPersistant = playerShip.GetComponent<GUIDPersistant>();

        interactionController.stopInteraction();

        // Delete all old modules.
        List<Transform> allModules = new List<Transform>();
        allModules = GUIDPersistant_.RecursiveFindChild(playerShip.transform, allModules, "Modules");

        foreach(Transform module in allModules)
        {
            if (module != playerShip.transform) {
                
                Destroy(module.gameObject);
            }
        }

        // https://docs.unity3d.com/2020.1/Documentation/Manual/JSONSerialization.html
        string jsonSaveData = File.ReadAllText(Application.dataPath + FileName);
        if (jsonSaveData != null)
        {
            // Parse JSON data into class
            shipData shipData1 = shipData.CreateFromJSON(jsonSaveData);

            // Player ship transform Command Module
            playerShip.transform.position = shipData1.position;
            playerShip.transform.rotation = shipData1.rotation;

            playerShip.name = shipData1.shipName;

            foreach (ShipModuleInfo moduleInfo in shipData1.modulesInfo)
            {
                GameObject instance;
                GameObject parentGameObject = findObjectbyGUID(GUIDObj.getGUID(moduleInfo.parentGameObject)).gameObject;

                // Spawn by prefab name
                for (int i = 0; i < prefabs.Count; i++)
                {
                    if (prefabs[i].GetComponent<AttachableModule>().getPrefabName() == moduleInfo.prefabName)
                    {
                        
                        // instance prefab
                        instance = Instantiate(prefabs[i], moduleInfo.position, moduleInfo.rotation);
                        //Debug.Log(prefabs[i].GetComponent<AttachableModule>().getPrefabName());
                        //Debug.Log(instance.transform.position);
                        instance.transform.SetParent(parentGameObject.transform);
                        instance.name = moduleInfo.name;
                        instantiatedModules.Add(instance);  

                        // 

                        instantiatedModulesScript = instantiatedModules.GetComponent<AttachableModule>();
                        instantiatedModulesScript.subscribe();

                        // Set module connection list and state.
                        IModule parentModule = parentGameObject.GetComponent<IModule>();
                        AttachableModule instanceModule = instance.GetComponent<AttachableModule>();

                        // Connection state
                        instanceModule.isConnected = true;
                        parentModule.AttachedModules.Add(instanceModule);


                    }
                }
            }

            moduleAttatchmentPointStates(shipData1);

            commandModuleAttatchmentPointStates(shipData1);
        }

        _inertiaTensor._init(true);
        playerShip.GetComponent<SpaceShip>().initPID();
    }

    public void commandModuleAttatchmentPointStates(shipData saveData1)
    {
        GameObject commandModuleObj = findObjectbyGUID(GUIDObj.getGUID(saveData1.shipName)).gameObject;
        CommandModule commandModuleScript = findObjectbyGUID(GUIDObj.getGUID(saveData1.shipName)).GetComponent<CommandModule>();

        List<Transform> childAttachmentPoints = new List<Transform>();

        foreach (Transform child in commandModuleObj.transform)
        {
            if (child.GetComponent<AttachmentPoint>() != null)
            {
                childAttachmentPoints.Add(child);
            }
        }

        foreach (AttatchmentPointInfo attatchmentPointInfo in saveData1.attachmentPointsInfo)
        {

            for (int i = 0; i < saveData1.attachmentPointsInfo.Count; i++)
            {
                childAttachmentPoints[i].name = saveData1.attachmentPointsInfo[i].thisGameObjectName;
                childAttachmentPoints[i].GetComponent<AttachmentPoint>().ConnectionPoint = saveData1.attachmentPointsInfo[i].isConnectionPoint;
                childAttachmentPoints[i].GetComponent<AttachmentPoint>().Occupied = saveData1.attachmentPointsInfo[i].isOccupied;

            }
        }
    }


    public void moduleAttatchmentPointStates(shipData saveData1)
    {
        foreach (ShipModuleInfo moduleInfo in saveData1.modulesInfo)
        {

            //Debug.Log(moduleInfo.name);
            GameObject moduleObject = findObjectbyGUID(GUIDObj.getGUID(moduleInfo.name)).gameObject;
            AttachableModule moduleScript = moduleObject.GetComponent<AttachableModule>();
            moduleScript.connectedToPoint = findObjectbyGUID(GUIDObj.getGUID(moduleInfo.connectedToPoint)).gameObject.GetComponent<AttachmentPoint>() ;

            List<Transform> childAttachmentPoints = new List<Transform>();


            // Save child into list and set each attatchment point name and isconnectionpoint state.
            foreach (Transform child in moduleObject.transform )
            {
                if (child.GetComponent<AttachmentPoint>() != null)
                {
                    childAttachmentPoints.Add(child);
                }
                
            }

            for (int i = 0; i < moduleInfo.attachmentPointsInfo.Count; i++)
            {
                childAttachmentPoints[i].name = moduleInfo.attachmentPointsInfo[i].thisGameObjectName;
                childAttachmentPoints[i].GetComponent<AttachmentPoint>().ConnectionPoint = moduleInfo.attachmentPointsInfo[i].isConnectionPoint;
                childAttachmentPoints[i].GetComponent<AttachmentPoint>().Occupied = moduleInfo.attachmentPointsInfo[i].isOccupied;

            }

        }
    }

    public Transform findObjectbyGUID(string _GUID)
    {
        if (string.IsNullOrEmpty(_GUID))
        {
            return null;

        } else
        {
            GameObject[] allGameObjects = GameObject.FindObjectsOfType<GameObject>();

            foreach (GameObject go in allGameObjects)
            {
                // Check if the GameObject's name contains the target substring.
                if (go.name.Contains(_GUID))
                {
                    return go.transform;
                }
            }
        }
        return null;
    }
}


[System.Serializable]
public class shipData
{
    public string shipName;
    public float health;
    public Vector3 position = Vector3.zero;
    public Quaternion rotation = Quaternion.identity;

    public List<AttatchmentPointInfo> attachmentPointsInfo = new List<AttatchmentPointInfo>();
    public List<ShipModuleInfo> modulesInfo = new List<ShipModuleInfo>();
    
    private CommandModule commandModule;

    public static shipData CreateFromJSON(string jsonString)
    {
        return JsonUtility.FromJson<shipData>(jsonString);
    }

    public shipData(CommandModule _commandModule)
    {

        commandModule = _commandModule;
    }

    public void getShipSaveData()
    {
        // Reset 
        attachmentPointsInfo = new List<AttatchmentPointInfo>();
        modulesInfo = new List<ShipModuleInfo>();

        shipName = commandModule.gameObject.name;
        //guid = commandModule.gameObject.GetComponent<GUIDPersistant>().guidAsString;

        // Get stats
        health = commandModule.Health;

        position = commandModule.getPosition();
        rotation = commandModule.getRotation();

        // Get all attatchment attatched to the ship that is occupied.
        //Debug.Log("Command Module -----------------  ");

        foreach (AttachmentPoint point in commandModule.getOccupiedAttatchmentPoints())
        {
            //Debug.Log(point.name + " attatched to " + point.ConnectedToModule);
            AttatchmentPointInfo pointInfo = new AttatchmentPointInfo(point);
            attachmentPointsInfo.Add(pointInfo);
        }

        //Debug.Log("--------------------------------");

        // Each module in the list
        foreach (AttachableModule _module in commandModule.GetAllModulesPoints())
        {

            ShipModuleInfo module = new ShipModuleInfo(_module);
            modulesInfo.Add(module);

        }
    }

}

[System.Serializable]
public class ShipModuleInfo
{
    
    public string name = "";
    public string prefabName = "";
    public float health = 0;
    public Vector3 position = Vector3.zero;
    public Quaternion rotation = Quaternion.identity;
    public string parentGameObject;
    public string connectedToPoint;

    public List<AttatchmentPointInfo> attachmentPointsInfo = new List<AttatchmentPointInfo>();

    public ShipModuleInfo(AttachableModule _module)
    {
        
        name = _module.gameObject.name;

        prefabName = _module.getPrefabName();
        health = _module.Health;
        position = _module.getPosition();
        rotation = _module.getRotation();
        connectedToPoint = _module.connectedToPoint.name;

        parentGameObject = _module.getParentObjectName();

        Debug.Log("Child");

        // Get all attatchment Points
        foreach (Transform child in _module.gameObject.transform)
        {
            Debug.Log(child);
            if (child.gameObject.layer == LayerMask.NameToLayer("AttachmentPoints"))
            {

                AttatchmentPointInfo pointInfo = new AttatchmentPointInfo(child.GetComponent<AttachmentPoint>());
                attachmentPointsInfo.Add(pointInfo);
            }
        }
    }

}

[System.Serializable]
public class AttatchmentPointInfo
{
    public string thisGameObjectName = "";
    public string connectedPointName = "";
    public bool isConnectionPoint;
    public bool isOccupied;

    public AttatchmentPointInfo(AttachmentPoint _point)
    {
        thisGameObjectName = _point.gameObject.name;
        isConnectionPoint = _point.ConnectionPoint;
        isOccupied = _point.Occupied;

        if (_point.ConnectedPoint != null)
        {
            connectedPointName = _point.ConnectedPoint.gameObject.name;
        }

    }
}




