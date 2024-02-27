using UnityEngine;

public class MovementController
{
    public float MaxAcceleration { get; set; } = 10f;

    private Rigidbody targetRigidbody;

    public MovementController(Rigidbody rb)
    {
        targetRigidbody = rb;
        if (targetRigidbody == null)
        {
            Debug.LogError("MovementController initialized without a Rigidbody!");
        }
        else
        {
            Debug.Log($"MovementController initialized for {targetRigidbody.gameObject.name}.");
        }
    }

    public void MoveWithThrustPercentage(Vector3 thrustVector)
    {
        if (targetRigidbody != null)
        {
            float thrustPercentage = thrustVector.magnitude;
            if (thrustPercentage < 0f || thrustPercentage > 1f)
            {
                Debug.LogError($"Thrust percentage should be between 0 and 1. Received: {thrustPercentage}, GameObject: {targetRigidbody.gameObject.name}");
                return;
            }

            float finalAcceleration = MaxAcceleration * thrustPercentage;
            Vector3 force = thrustVector.normalized * finalAcceleration;

            targetRigidbody.AddForce(force, ForceMode.Acceleration);

            //Debug.Log($"GameObject: {targetRigidbody.gameObject.name}, Final Acceleration: {finalAcceleration}, Applied Force: {force}");
        }
        else
        {
            Debug.LogError("MoveWithThrustPercentage() called on MovementController without a Rigidbody!");
        }
    }

    // Method to handle non-normalized speed
    public void MoveWithDesiredSpeed(Vector3 direction)
    {
        if (targetRigidbody != null)
        {
            float desiredAcceleration = direction.magnitude;
            if (desiredAcceleration > 0.01f) // small threshold to avoid very tiny magnitudes
            {
                float finalAcceleration = Mathf.Min(desiredAcceleration, MaxAcceleration);

                // Normalize the direction to ensure the magnitude is 1
                direction.Normalize();

                // Multiply by finalAcceleration to get the final force
                Vector3 force = direction * finalAcceleration;

                targetRigidbody.AddForce(force, ForceMode.Acceleration);

                //Debug.Log($"GameObject: {targetRigidbody.gameObject.name}, Desired Acceleration: {desiredAcceleration}, Final Acceleration: {finalAcceleration}, Applied Force: {force}");
            }
        }
        else
        {
            Debug.LogError("MoveWithDesiredSpeed() called on MovementController without a Rigidbody!");
        }
    }
}

