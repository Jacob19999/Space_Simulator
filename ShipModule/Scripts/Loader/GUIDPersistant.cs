using JetBrains.Annotations;
using System;
using System.Collections.Generic;
using System.Data.SqlTypes;
using Unity.VisualScripting;
using UnityEngine;

public class GUIDPersistantGenerator
{
    public string guidAsString;
    private System.Guid _guid;
    public System.Guid guid
    {
        get
        {
            if (_guid == System.Guid.Empty)
            {
                _guid = new System.Guid(guidAsString);
            }
            return _guid;
        }
    }

    public string Generate()
    {
        _guid = System.Guid.NewGuid();
        guidAsString = guid.ToString();

        guidAsString.Substring(0, 36);

        return guidAsString;
    }

    public string getGUID(string _name)
    {

        string searchStr = "'";

        int index = _name.IndexOf(searchStr);

        if (index == -1)
        {
            return null;
        }
        else
        {
            string output = _name.Substring(index + 1, _name.Length - index - 2);
            return output;
        }

    }

}

public class GUIDPersistant : MonoBehaviour
{

    private GUIDPersistantGenerator GUID_ = new GUIDPersistantGenerator();

    [SerializeField]
    public GameObject CommandModule;

    [SerializeField]
    public GameObject getGUIDObject;

    [SerializeField]
    public string GUIDOutput;

    public void setGUID(GameObject thisObject)
    {
        List<Transform> allModules = new List<Transform>();
        List<Transform> allAttatchmentPoints = new List<Transform>();

        allModules = RecursiveFindChild(thisObject.transform, allModules, "Modules");
        allAttatchmentPoints = RecursiveFindChild(thisObject.transform, allAttatchmentPoints, "AttachmentPoints");

        foreach (Transform guidObj in allModules)
        {

            string guid = GUID_.getGUID(guidObj.name);

            if (guid == null)
            {
                guidObj.name = guidObj.name + " '" + GUID_.Generate() + "'";
            }

        }
        foreach (Transform guidObj in allAttatchmentPoints)
        {

            string guid = GUID_.getGUID(guidObj.name);

            if (guid == null)
            {
                guidObj.name = guidObj.name + " '" + GUID_.Generate() + "'";
            }
        }

    }
    public void setAllGUID()
    {
        setGUID(CommandModule);

    }

    public void getGUIDFromObj()
    {

        GUIDOutput = GUID_.getGUID(getGUIDObject.name);

    }



    public List<Transform> RecursiveFindChild(Transform parent, List<Transform> children, string LayerName)
    {

        foreach (Transform child in parent)
        {
            if (child != null) { 
                
                RecursiveFindChild(child, children, LayerName); 
            }
            else {
                return null;
            }
            

        }

        if (parent.gameObject.layer == LayerMask.NameToLayer(LayerName))
        {
            children.Add(parent);
        }

        return children;
    }


}