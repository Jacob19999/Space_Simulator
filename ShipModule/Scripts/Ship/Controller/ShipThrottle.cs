using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class ShipThrottle : MonoBehaviour
{
    [Range(-1f, 1f)]
    public float main_throttle = 0f;

    public List<Transform> MainEngines = new List<Transform>();
    public List<MainEngine> ProgradeEngines = new List<MainEngine>();
    public List<MainEngine> RetroEngines = new List<MainEngine>();
    private List<MainEngine> MainEnginesScript = new List<MainEngine>();

    private GameObject _CommandModule;
    private GameObject _Controller;
    private SpaceShip _ControllerScript;
    


    void Awake()
    {

        _CommandModule = PlayerShipSingleton.Instance.gameObject;
        _ControllerScript = _CommandModule.transform.GetComponent<SpaceShip>();


    }

// Update is called once per frame
    void Update()
    {
        if (main_throttle > 0f)
        {
            foreach (MainEngine engine in ProgradeEngines)
            {
                engine.throttle = main_throttle;
            }

        }

        if (main_throttle < 0f )
        {
            foreach (MainEngine engine in RetroEngines)
            {
                engine.throttle = Mathf.Abs( main_throttle);
            }
        }



    }

    public void getEngines()
    {
        float thrusterThreashold = 0.8f;

        MainEngines = RecursiveFindChild(_CommandModule.transform, MainEngines, "MainEngines");

        foreach (Transform thisEngine in MainEngines)
        {
            MainEngine thisEngineScript = thisEngine.GetComponent<MainEngine>();
            MainEnginesScript.Add(thisEngineScript);

            Vector3 direction = Vector3.Normalize(thisEngineScript.getThrustDirection(_CommandModule.transform));

            if (direction.z < -1 * thrusterThreashold)
            {
                ProgradeEngines.Add(thisEngineScript);
            }

            if (direction.z > 1 * thrusterThreashold)
            {
                RetroEngines.Add(thisEngineScript);
            }

        }

    }

    private List<Transform> RecursiveFindChild(Transform parent, List<Transform> children, string tag)
    {

        foreach (Transform child in parent)
        {
            RecursiveFindChild(child, children, tag);
        }

        if (parent.gameObject.tag == tag)
        {
            children.Add(parent);
        }

        return children;
    }

    




}
