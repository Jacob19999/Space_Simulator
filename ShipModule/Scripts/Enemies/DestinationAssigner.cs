using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class DestinationAssigner : MonoBehaviour
{
    public GameObject destinationContainer;
    public GameObject AIContainer;

    // Update is called once per frame
    void Update()
    {
        AssignDestinations();
    }

    private void AssignDestinations()
    {
        // Ensure containers are not null
        if (destinationContainer == null || AIContainer == null)
        {
            Debug.LogError("Destination or AI container is not set.");
            return;
        }

        int count = destinationContainer.transform.childCount;

        // Get all AI children
        foreach (Transform aiTransform in AIContainer.transform)
        {
            // Get the AIFlightComputer component
            AIFlightComputer aiFlightComputer = aiTransform.GetComponent<AIFlightComputer>();

            // Check if the AI has a destination
            if (aiFlightComputer != null && !aiFlightComputer.HasDestination())
            {
                
                // Choose a random destination
                int randomIndex = Random.Range(0, count - 1);
                Transform destinationTransform = destinationContainer.transform.GetChild(randomIndex);

                // Assign the chosen destination's position to the AI's Destination property
                Debug.Log(aiTransform.name + " set Destination " + destinationTransform.name);
                aiFlightComputer.Destination = destinationTransform.position;
            }
        }
    }
}
