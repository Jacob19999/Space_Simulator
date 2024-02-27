
using UnityEngine.UI;
using TMPro;
using UnityEngine;
using FLFlight;

public class ShipBuilderMode : MonoBehaviour
{
    public Button editorButton;
    private TextMeshProUGUI buttonText;
    public InteractionController _interactionController;
    public ShipInput _shipInput;
    public SpaceShip _shipController;
    public bool inEditMode;
    private CameraManager _CameraManager;

    void Awake()
    {
        editorButton = gameObject.GetComponentInChildren<Button>();
        buttonText = editorButton.GetComponentInChildren<TextMeshProUGUI>();
        _interactionController = InteractionControllerSingleton.Instance.GetComponent<InteractionController>();

        _shipInput = PlayerShipSingleton.Instance.transform.GetComponent<ShipInput>();
        _shipController = PlayerShipSingleton.Instance.transform.GetComponent<SpaceShip>();
        _CameraManager = GameObject.Find("CameraManager").gameObject.GetComponent<CameraManager>();

        editorButton.onClick.AddListener(TaskOnClick);

    }

    public void modeSelector(bool globalMode)
    {
        if (globalMode)
        {
            buttonText.text = "Exit Edit Mode";
        } else
        {
            buttonText.text = "Edit Mode";
        }

    }


    void TaskOnClick()
    {
        Debug.Log("Button Clicked");

        if (_shipInput == null)
        {
            _shipInput = PlayerShipSingleton.Instance.transform.Find("ShipController").GetComponent<ShipInput>();
            _shipController = PlayerShipSingleton.Instance.transform.Find("ShipController").GetComponent<SpaceShip>();
        }


        if (inEditMode)
        {

            Debug.Log("Exit Edit mode");
            buttonText.text = "Edit Mode";
            _interactionController.globalEnable = false;
            _interactionController.sessionInit = false;
            _shipController.globalEnable = true;


            inEditMode = false;

        } else if (inEditMode == false && _shipInput.controlsEnable == false)
        {
            Debug.Log("Enter Edit mode");
            buttonText.text = "Exit Edit Mode";

            // Disable the ship controllers
            _shipInput.controlsEnable = false;
            _shipInput.zeroAllInputs();
            _shipController.globalEnable = false;
            _shipController.rcs_Pitch_Rate = 0;
            _shipController.rcs_Yaw_Rate  = 0;
            _shipController.rcs_Roll_Rate = 0;

            // Enable Interation controller
            _CameraManager._activeCam = CameraManager.activeCam.FreeLookCam;
            _interactionController.globalEnable = true;


            inEditMode = true;
        }

    }
}


