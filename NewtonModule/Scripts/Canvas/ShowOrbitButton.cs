using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ShowOrbitButton : MonoBehaviour
{
    public DrawOrbitLines _DrawOrbitLines;

    public Button showOrbitButton;

    void Awake()
    {

        if (_DrawOrbitLines)
        {
            showOrbitButton.onClick.AddListener(showOrbit);
        }
        


    }

    private void showOrbit()
    {
        //Debug.Log("Button");

        if (_DrawOrbitLines.drawLines)
        {

            _DrawOrbitLines.clearAllOrbitPoints();
        } else
        {
            _DrawOrbitLines.drawOrbits();
        }
        

        
    }


}
