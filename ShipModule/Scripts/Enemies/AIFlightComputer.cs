using System.Collections.Generic;
using UnityEngine;
using static UnityEditor.Experimental.GraphView.GraphView;

[RequireComponent(typeof(Rigidbody))]
public class AIFlightComputer : MonoBehaviour
{
    [Header("Objects")]
    public InertiaTensor _inertiaTensor;

    [Header("Movement Settings")]
    public float maxAcceleration = 10f;  // This can be set/changed at runtime in the Unity Editor

    [Header("Close Avoidance Settings")]
    public float baseOverlapDiameter = 15f;
    public float velocityMultiplier = 1.1f; // Multiplier for sphere size based on velocity

    [Header("Far Avoidance Settings")]
    public List<LayerMask> detectionLayers = new();

    [Header("Pathfinding Settings")]
    [SerializeField] private float successRadius = 10f; // How close to the destination before we consider it a success
    private Vector3 destination = Vector3.zero;
    private bool pathFound = false;
    private Vector3 finalDestination = Vector3.zero;
    private bool substep = false;
    private int pathfindingStep = 0;
    [SerializeField] private int maxPathfindingSteps = 5;

    [Header("Config")]
    [SerializeField]
    private int evadeResetUpdateCount = 25; // this should be 2 seconds of fixed updates
    private int evadeResetUpdateCounter = 0;
    private int evadingCounter = 0;

    [Header("Movement Controller")]
    public SpaceShip spaceShipController;

    private MovementController movementController;
    private Rigidbody thisRigidbody;

    [Header("Resultant Movement")]
    public Vector3 accumulatedMovementDirection = Vector3.zero;
    private bool detectedSomething = false;
    private bool needToEvade = false;

    public Vector3 Destination
    {
        set
        {
            destination = value;
            pathFound = false;
        }
    }

    public bool HasDestination()
    {
        if (destination == Vector3.zero)
        {
            return false;
        }
        else
        {
            return true;
        }
    }

    public void ClearDestination()
    {
        destination = Vector3.zero;
        pathFound = false;
        substep = false;
        finalDestination = Vector3.zero;
        pathfindingStep = 0;
    }

    private void Awake()
    {
        spaceShipController = gameObject.GetComponent<SpaceShip>();

        thisRigidbody = GetComponent<Rigidbody>();
        _inertiaTensor = gameObject.GetComponent<InertiaTensor>();
        movementController = new MovementController(thisRigidbody);
        movementController.MaxAcceleration = maxAcceleration; // Set initial value

        // if the detection layers list is empty, add the default layer
        if (detectionLayers.Count == 0)
        {
            detectionLayers.Add(LayerMask.NameToLayer("Default"));
            Debug.LogWarning("No detection layers set for " + gameObject.name + ". Default layer added.");
        }
    }

    private void Update()
    {
        // Update max acceleration in case it's changed at runtime
        movementController.MaxAcceleration = maxAcceleration;

        CloseEvade();
        ShipEvade();
        FlightPathEvade();
        MovementDecider();

        if (evadeResetUpdateCounter >= evadeResetUpdateCount)
        {
            accumulatedMovementDirection = Vector3.zero;
            evadeResetUpdateCounter = 0;
        }
    }

    public void FixedUpdate()
    {
        if (!detectedSomething)
        {
            evadeResetUpdateCounter++;
            evadingCounter = 0;
        }
        if (needToEvade)
        {
            evadingCounter++;
        }

        //Debug.Log("AI Move " + accumulatedMovementDirection);

        // Apply accumulated movement in FixedUpdate

        Debug.Log("Mov Dir" + accumulatedMovementDirection + " " + maxAcceleration);

        spaceShipController.pointToTarget(accumulatedMovementDirection.normalized, 1f);

        Vector3 localAccumulatedMovement = gameObject.transform.InverseTransformDirection(accumulatedMovementDirection);
        spaceShipController.rcs_x_rate = localAccumulatedMovement.x;
        spaceShipController.rcs_y_rate = localAccumulatedMovement.y;
        spaceShipController.rcs_z_rate = localAccumulatedMovement.z;

        //movementController.MoveWithDesiredSpeed(accumulatedMovementDirection);

    }

    private void CloseEvade()
    {
        float sphereDiameter = Mathf.Max(baseOverlapDiameter, baseOverlapDiameter + velocityMultiplier * _inertiaTensor.worldVelocity.magnitude);

        Collider[] hits = Physics.OverlapSphere(transform.position, sphereDiameter/2);

        Vector3 totalEvasionVelocity = Vector3.zero;
        detectedSomething = false;

        foreach (Collider hit in hits)
        {
            if (hit.gameObject != gameObject)
            {
                 detectedSomething = true;

                Bounds hitBounds = hit.bounds;
                Vector3 closestPoint = hitBounds.ClosestPoint(transform.position);

                // note - these vector names are 'relative' to this object
                Vector3 vectorToHit = closestPoint - transform.position; // this vector points from us to the hit object
                Vector3 vectorAwayFromHit = -vectorToHit; // this vector points away from the hit object
                Vector3 relativeVelocity = (hit.attachedRigidbody?.velocity ?? Vector3.zero) - _inertiaTensor.worldVelocity;

                float angle = Vector3.Angle(vectorAwayFromHit, relativeVelocity);

                if (angle >= 90)
                {
                    // the object is moving away from us, so we don't need to worry about it
                    continue;
                }

                float angleUrgency = 1 - angle / 90f; // urgency is 0 when the object is perpendicular to us, and 1 when it's coming straight at us
                float distanceUrgency = 1 - (vectorToHit.magnitude / sphereDiameter); // urgency is 0 when the object is at the edge of the sphere, and 1 when it's at the center

                float timeToImpact = vectorToHit.magnitude / (relativeVelocity.magnitude * Mathf.Cos(Mathf.Deg2Rad * angle)); // the guard clause above prevents a divide by zero error from ever happening

                if (timeToImpact > 10)
                {
                    // the object is too far away to worry about
                    continue;
                }

                float safeDistance = (hit.bounds.extents.magnitude + GetComponent<Collider>().bounds.extents.magnitude) * 1.1f;  // Added a 10% buffer
                float distanceToMoveMagnitude = Mathf.Abs(safeDistance - vectorToHit.magnitude);
                float minVelocityMagnitude = distanceToMoveMagnitude / timeToImpact;

                Vector3 evadeVector = vectorAwayFromHit.normalized * minVelocityMagnitude;
                evadeVector *= 1 + (angleUrgency + distanceUrgency);

                if (evadingCounter > 25) // this should be about 1/2 a second
                {
                    Vector3 auxiliaryVector;

                    // Swap components of evadeVector to ensure non-collinearity
                    if (Mathf.Abs(evadeVector.x) > Mathf.Abs(evadeVector.y))
                        auxiliaryVector = new Vector3(-evadeVector.z, evadeVector.y, evadeVector.x);
                    else
                        auxiliaryVector = new Vector3(evadeVector.y, -evadeVector.x, evadeVector.z);

                    // Now get the actual perpendicular vector using the cross product
                    Vector3 lateralDirection = Vector3.Cross(evadeVector, auxiliaryVector).normalized;

                    Vector3 lateralMove = lateralDirection * evadeVector.magnitude * 2;

                    evadeVector += lateralMove;
                }

                totalEvasionVelocity += evadeVector;
            }
        }

        if (totalEvasionVelocity.magnitude > 0.01f)
        {
            needToEvade = true;
            accumulatedMovementDirection = totalEvasionVelocity + (accumulatedMovementDirection * .5f);
        }
        else
        {
            needToEvade = false;
        }
    }

    private void ShipEvade()
    {

        List<GameObject> shipList = new();
        GameObject[] allObjects = FindObjectsOfType<GameObject>();
        foreach (GameObject anObject in allObjects)
        {
            if (anObject.transform.parent == null && detectionLayers.Contains(LayerMask.GetMask(LayerMask.LayerToName(anObject.layer))))
            {
                // skip if this game object is the same as the one we're attached to
                if (anObject == gameObject)
                {
                    continue;
                }
                shipList.Add(anObject);
            }
        }

        Vector3 totalEvasionVelocity = Vector3.zero;

        foreach (GameObject ship in shipList)
        {

            Transform shipTransform = ship.transform;
            Vector3 vectorToShip = shipTransform.position - transform.position; // this vector points from us to the ship
            Vector3 vectorAwayFromShip = -vectorToShip; // this vector points away from the ship
            Vector3 relativeVelocity = ship.GetComponent<InertiaTensor>().worldVelocity - _inertiaTensor.worldVelocity;

            float angle = Vector3.Angle(vectorAwayFromShip, relativeVelocity);

            if (angle >= 90)
            {
                // the ship is moving away from us, so we don't need to worry about it
                continue;
            }

            // calculate the time to impact (assuming constant velocities)
            float closingSpeed = relativeVelocity.magnitude; // the speed at which the two objects are moving towards each other
            float distance = vectorToShip.magnitude; // the current distance between the two objects
            float timeToImpact = distance / closingSpeed;

            if (timeToImpact > 10)
            {
                // the ship is too far away to worry about
                continue;
            }

            // use the Close Avoidance Settings to get the sphere diameter but use the ship's velocity instead of our own
            float safetyDiameter = Mathf.Max(baseOverlapDiameter, baseOverlapDiameter + velocityMultiplier * ship.GetComponent<Rigidbody>().velocity.magnitude);
            float safetyRadius = safetyDiameter / 2;

            // project the positions of both objects based on their velocities
            Vector3 projectedUs = transform.position + _inertiaTensor.worldVelocity * timeToImpact;
            Vector3 projectedShip = shipTransform.position + ship.GetComponent<InertiaTensor>().worldVelocity * timeToImpact;

            // check if the projected position of the ship is within the sphere around 'us'
            if ((projectedUs - projectedShip).magnitude < safetyRadius)
            {
                // the ship will enter the sphere, take evasive action
                float safeDistance = safetyRadius * 1.1f;  // Added a 10% buffer
                float minVelocityMagnitude = safeDistance / timeToImpact;

                Vector3 evadeVector = vectorAwayFromShip.normalized * minVelocityMagnitude;

                Vector3 auxiliaryVector;
                // Swap components of evadeVector to ensure non-collinearity
                if (Mathf.Abs(evadeVector.x) > Mathf.Abs(evadeVector.y))
                    auxiliaryVector = new Vector3(-evadeVector.z, evadeVector.y, evadeVector.x);
                else
                    auxiliaryVector = new Vector3(evadeVector.y, -evadeVector.x, evadeVector.z);

                // Now get the actual perpendicular vector using the cross product
                Vector3 lateralDirection = Vector3.Cross(evadeVector, auxiliaryVector).normalized;

                // set evade vector to the lateral vector with the same magnitude
                evadeVector = lateralDirection.normalized * evadeVector.magnitude * 2;

                Debug.Log("Evasion Velocity: " + evadeVector + " for " + gameObject.name + " at " + Time.time + " with time to impact of " + timeToImpact);

                // add this to the totalEvasionVelocity
                totalEvasionVelocity += evadeVector;

            }
        }
        if (totalEvasionVelocity.magnitude > 0.01f)
        {
            needToEvade = true;
            accumulatedMovementDirection = totalEvasionVelocity + (accumulatedMovementDirection * .5f);
        }
        else
        {
            needToEvade = false;
        }
    }

    private void FlightPathEvade()
    {

        float sphereDiameter = Mathf.Max(baseOverlapDiameter, baseOverlapDiameter + velocityMultiplier * thisRigidbody.velocity.magnitude);

        Vector3 velocityDirection = _inertiaTensor.worldVelocity.normalized;
        float distance = _inertiaTensor.worldVelocity.magnitude * 10;

        Vector3 totalEvasionVelocity = Vector3.zero;

        RaycastHit hit;
        if (Physics.SphereCast(transform.position, sphereDiameter / 2, velocityDirection, out hit, distance))
        {
            Vector3 nearestBound = hit.point + hit.normal * (sphereDiameter / 2);

            Vector3 auxiliaryVector;
            if (Mathf.Abs(_inertiaTensor.worldVelocity.x) > Mathf.Abs(_inertiaTensor.worldVelocity.y))
            {
                auxiliaryVector = new Vector3(-_inertiaTensor.worldVelocity.z, _inertiaTensor.worldVelocity.y, _inertiaTensor.worldVelocity.x);
            }
            else
            {
                auxiliaryVector = new Vector3(_inertiaTensor.worldVelocity.y, -_inertiaTensor.worldVelocity.x, _inertiaTensor.worldVelocity.z);
            }

            Vector3 safestPerpendicular = Vector3.zero;
            float maxDistance = 0;

            for (int i = 0; i < 36; i++) // test 36 different perpendicular vectors
            {
                float angle = i * 10f; // each vector is 10 degrees apart
                Vector3 testVector = Quaternion.Euler(0, angle, 0) * auxiliaryVector;
                Vector3 perpendicularVector = Vector3.Cross(_inertiaTensor.worldVelocity, testVector).normalized;
                float testDistance = DistanceFromObstacles(transform.position + perpendicularVector * distance);
                if (testDistance > maxDistance)
                {
                    maxDistance = testDistance;
                    safestPerpendicular = perpendicularVector;
                }
            }

            float neededAcceleration = (nearestBound - transform.position).magnitude / (2 * Mathf.Pow(distance / _inertiaTensor.worldVelocity.magnitude, 2));
            Vector3 evasionVelocity = safestPerpendicular * neededAcceleration;

            totalEvasionVelocity = evasionVelocity;
        }

        if (totalEvasionVelocity.magnitude > 0.01f)
        {
            needToEvade = true;
            accumulatedMovementDirection = totalEvasionVelocity + (accumulatedMovementDirection * 0.5f);
        }
        else
        {
            needToEvade = false;
        }
    }

    private void MovementDecider()
    {
        if (HasDestination())
        {
            if (pathFound)
            {
                DestinationImpulse();
            }
            else
            {
                Pathfind();
            }
        }
        else
        {
            KillVelocity();
        }
    }

    private void KillVelocity()
    {
        if (!needToEvade)
        {
            spaceShipController.rcs_x_rate = 0;
            spaceShipController.rcs_y_rate = 0;
            spaceShipController.rcs_z_rate = 0;

        }
    }

    private void Pathfind()
    {
        float sphereDiameter = baseOverlapDiameter + velocityMultiplier * _inertiaTensor.worldVelocity.magnitude;
        RaycastHit hitInfo;

        // Step 1: Fire a sphere cast from us to the destination
        bool clearPath = Physics.SphereCast(transform.position, sphereDiameter / 2, destination - transform.position, out hitInfo, Vector3.Distance(transform.position, destination));

        // Step 2: Check if the path is clear or if the obstacle is within the success radius of the destination
        if (!clearPath || (hitInfo.collider != null && Vector3.Distance(hitInfo.collider.transform.position, destination) <= successRadius))
        {
            // We can fly directly to the destination
            pathFound = true;
        }
        else
        {
            // Step 3: The direct path is blocked, find the shortest path around the obstacle
            if (finalDestination == Vector3.zero)
            {
                // 3a: Copy the destination to finalDestination if it's not already set
                finalDestination = destination;
            }

            Bounds obstacleBounds = hitInfo.collider.bounds;
            Vector3 directionToObstacle = (obstacleBounds.center - transform.position).normalized;
            Vector3 detourDirection = Vector3.zero;

            // Find which axis of the bounding box is shortest to go around
            float minExtent = Mathf.Min(obstacleBounds.extents.x, obstacleBounds.extents.y, obstacleBounds.extents.z);
            if (minExtent == obstacleBounds.extents.x)
                detourDirection = transform.right * Mathf.Sign(Vector3.Dot(transform.right, directionToObstacle));
            else if (minExtent == obstacleBounds.extents.y)
                detourDirection = transform.up * Mathf.Sign(Vector3.Dot(transform.up, directionToObstacle));
            else
                detourDirection = transform.forward * Mathf.Sign(Vector3.Dot(transform.forward, directionToObstacle));

            // Set the substep to true and adjust the destination
            substep = true;
            Destination = obstacleBounds.center + detourDirection * (sphereDiameter + obstacleBounds.extents.magnitude);

            // Increment pathfindingStep and check if we've exceeded maxPathfindingSteps
            pathfindingStep++;
            if (pathfindingStep >= maxPathfindingSteps)
            {
                TangentBug();
            }
        }
    }

    private void TangentBug()
    {
        // Placeholder for the TangentBug algorithm or alternative pathfinding logic
        // if the simple pathfinding fails after maxPathfindingSteps
        // You can implement the recovery or alternative pathfinding logic here
        Debug.Log("TangentBug called - Pathfinding failed for " + gameObject.name + " at " + Time.time);
        ClearDestination();
    }

    private void DestinationImpulse()
    {
        // If path not found, exit the function
        if (!pathFound)
        {
            return;
        }

        // Update acceleration and deceleration speeds
        float accelerationSpeed = maxAcceleration;
        float decelerationSpeed = maxAcceleration;

        // Calculate the vector towards the destination
        Vector3 toDestination = destination - transform.position;
        float distanceToDestination = toDestination.magnitude;

        // Calculate the current speed in the direction towards the destination
        float currentSpeed = Vector3.Dot(_inertiaTensor.worldVelocity, toDestination.normalized);

        // If we are within the success radius, handle the substep or just clear the destination
        if (distanceToDestination <= successRadius)
        {
            if (substep)
            {
                Vector3 heldFinalDestination = finalDestination;
                ClearDestination();
                destination = heldFinalDestination;
            }
            else
            {
                ClearDestination();
            }
            return;
        }

        // Calculate the flip point based on the current relative velocity, acceleration, and deceleration speeds
        float decelerationDistance;
        if (decelerationSpeed > 0)
        {
            decelerationDistance = (currentSpeed * currentSpeed) / (2 * decelerationSpeed);
        }
        else
        {
            // Handle the case where deceleration speed is zero to avoid division by zero
            decelerationDistance = distanceToDestination; // Assume immediate deceleration is required
        }

        float flipPoint = distanceToDestination - decelerationDistance;

        // Decide whether to accelerate or decelerate based on the flip point
        if (distanceToDestination >= flipPoint)
        {
            if (currentSpeed < accelerationSpeed)
            {
                // Accelerate towards the destination
                Vector3 desiredVelocity = toDestination.normalized * accelerationSpeed;
                Vector3 accelerationVector = desiredVelocity - _inertiaTensor.worldVelocity;
                accumulatedMovementDirection += accelerationVector.normalized * accelerationSpeed;
            }
        }
        else
        {
            if (currentSpeed > 0)
            {
                // Decelerate to stop at the destination
                Vector3 decelerationVector = -_inertiaTensor.worldVelocity.normalized * decelerationSpeed;
                accumulatedMovementDirection += decelerationVector;
            }
        }
    }

    private float DistanceFromObstacles(Vector3 position)
    {
        float distance = float.MaxValue;
        Collider[] colliders = Physics.OverlapSphere(position, 1); // test a small sphere at the position
        foreach (Collider collider in colliders)
        {
            float testDistance = Vector3.Distance(position, collider.ClosestPoint(position));
            if (testDistance < distance)
            {
                distance = testDistance;
            }
        }
        return distance;
    }
}
