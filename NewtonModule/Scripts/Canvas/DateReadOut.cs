using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;
using UnityEngine.UI;


public class DateReadOut : MonoBehaviour
{

    private Planetarium _Planetarium;

    //[SerializeField]
    //private Slider timestepSlider;

    //[SerializeField]
    //private TextMeshProUGUI timestepReadout;

    [SerializeField]
    private TextMeshProUGUI dateReadout;
    [SerializeField]
    private TextMeshProUGUI timeReadout;


    private void Update()
    {
        if (_Planetarium == null)
        {
            _Planetarium = PlanetariumSingleton.Instance.GetComponent<Planetarium>();
        }

        UpdateTimeStepGUI();

    }

    private void UpdateTimeStepGUI()
    {

        //int timestepValue = (int)(timestepSlider.value - (timestepSlider.maxValue / 2f));

        //timestepReadout.text = timestepValue == 0 ? "RT" : timestepValue.ToString();

        //Debug.Log("Time Step : " + timestepReadout.text);

        //float percent = Mathf.Abs(timestepValue) / (timestepSlider.maxValue / 2f);
        //float scalar = 20000000 * percent / 20f;

        dateReadout.text = _Planetarium.currentDateTime.Year.ToString("F0") + " / " +
                           _Planetarium.currentDateTime.Month.ToString("F0") + " / " +
                           _Planetarium.currentDateTime.Day.ToString("F0");

        timeReadout.text = _Planetarium.currentDateTime.Hour.ToString("00") + " : " +
                           _Planetarium.currentDateTime.Minute.ToString("00") + " : " +
                           _Planetarium.currentDateTime.Second.ToString("00");

    }

}
