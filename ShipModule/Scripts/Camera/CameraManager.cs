using FLFlight;
using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.UI;
using static CameraManager;

public class CameraManager : MonoBehaviour
{

    [SerializeField] GameObject FreeLookCam;
    private OrbitCam FreeLookCamScript;

    [SerializeField] GameObject ShipControlRig;
    private MouseFlightController ShipControlRigScript;

    [SerializeField] public GameObject ShipControlHud;

    private ShipInput _ShipInput;

    private string activeCamera;
    private bool camStateChange = false;

    private ShipBuilderMode shipBuilderButton;
    private Button saveShipButton;
    private Button loadShipButton;




    public enum activeCam
    {
        FreeLookCam,
        ShipControlCam

    }
    public activeCam _activeCam;



    private void Awake()
    {

        ShipControlHud.SetActive(false);

        _activeCam = activeCam.FreeLookCam;
        camStateChange = true;
    }

    private void Update()
    {
        _init();

        if (Input.GetKeyDown(KeyCode.B))
        {
            if (shipBuilderButton.GetComponent<ShipBuilderMode>().inEditMode)
            {
                return;
            }

            if (_activeCam == activeCam.FreeLookCam)
            {
                //Debug.Log("Set Control Cam Active");
                _activeCam = activeCam.ShipControlCam;
                camStateChange = true;

            } else if (_activeCam == activeCam.ShipControlCam)
            {
                //Debug.Log("Set free Cam Active");
                _activeCam = activeCam.FreeLookCam;
                camStateChange = true;

            }

        }

        if (camStateChange) {

            // Set freelook as active.
            if (_activeCam == activeCam.FreeLookCam)
            {

                EnableFreeCam();

                camStateChange = false;
            }


            if (_activeCam == activeCam.ShipControlCam)
            {

                EnableShipControlCam();

                camStateChange = false;
            }

        }

    }

    private void _init()
    {
        activeCamera = Camera.main.name;

        if (_ShipInput == null)
        {
            _ShipInput = PlayerShipSingleton.Instance.gameObject.GetComponentInChildren<ShipInput>();
            ShipControlRigScript = ShipControlRig.GetComponent<MouseFlightController>();
            FreeLookCamScript = FreeLookCam.GetComponent<OrbitCam>();

        }
        if (shipBuilderButton == null)
        {
            shipBuilderButton = GameObject.Find("Ship Builder Button").GetComponent<ShipBuilderMode>();
            saveShipButton = GameObject.Find("Save Ship Button").GetComponent<Button>();
            loadShipButton = GameObject.Find("Load Ship Button").GetComponent<Button>();
        }

        if (ShipControlHud == null)
        {
            ShipControlHud = GameObject.Find("ShipControlHud");

        }


    }

    private void EnableFreeCam()
    {

        _ShipInput.zeroAllInputs();
        _ShipInput.controlsEnable = false;
        _ShipInput._controlMode = ShipInput.controlMode.NORMAL;

        ShipControlHud.SetActive(false);
        ShipControlRigScript.enabled = false;
        ShipControlRig.SetActive(false);


        FreeLookCamScript.enabled = true;
        FreeLookCam.SetActive(true);

        Cursor.visible = true;
        Cursor.lockState = CursorLockMode.None;

        shipBuilderButton.editorButton.gameObject.SetActive(true);
        saveShipButton.gameObject.SetActive(true);
        loadShipButton.gameObject.SetActive(true);

    }


    private void EnableShipControlCam()
    {
        

        _ShipInput.controlsEnable = true;
        _ShipInput._controlMode = ShipInput.controlMode.MANUAL;


        ShipControlRigScript.enabled = true;
        ShipControlRig.SetActive(true);
        ShipControlHud.SetActive(true);


        FreeLookCamScript.enabled = false;
        FreeLookCam.SetActive(false);
        
        shipBuilderButton.editorButton.gameObject.SetActive(false);
        saveShipButton.gameObject.SetActive(false);
        loadShipButton.gameObject.SetActive(false);

        Cursor.visible = false;
        Cursor.lockState = CursorLockMode.Confined;

        ShipControlRig.transform.parent = transform;
    }

    public void setActiveCam(activeCam thisActivecam)
    {
        _activeCam = thisActivecam;
        camStateChange = true;
    }


}
