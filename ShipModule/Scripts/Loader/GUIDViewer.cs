using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GUIDViewer : MonoBehaviour
{

    [SerializeField]
    public string GUID;

    void getGUID()
    {
        if (gameObject.GetComponent<AttachableModule>() != null)
        {

            GUID = gameObject.GetComponent<AttachableModule>().GUID;

        }

        if (gameObject.GetComponent<AttachmentPoint>() != null)
        {

            GUID = gameObject.GetComponent<AttachmentPoint>().GUID;

        }
    }
}
