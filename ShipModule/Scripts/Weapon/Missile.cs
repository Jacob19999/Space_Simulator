using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using UnityEngine;
using static UnityEngine.SpatialTracking.TrackedPoseDriver;

[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(CapsuleCollider))]

public class Missile : MonoBehaviour
{

    private MissileGuidence _guidence;
    private MainEngine _mainEngine;
    public GameObject _mainEngineGO;

    [Header("General Parameters:")]
    [Tooltip("Transform of the target. Typically assigned by launcher that shot the missile, but can be manually assigned for a missile already in the scene. If null on launch, the missile will have no guidance.")]
    public Transform target;

    [Tooltip("Launching object. Typically assigned by the launcher and only needs to be assigned if manually launching a missile already in the scene. When assigned, this will prevent the missile from colliding with whatever launched it.")]
    public Transform ownShip;

    [Tooltip("Position where this missile attaches to hardpoint style launchers. If not assigned, this will automatically search for a GameObject named \"Attach\". If no such GameObject, then the missile will attach at its origin.")]
    public Transform attachPoint;

    [Tooltip("Thrust N.")]
    public float thrust = 5000f;

    [Header("Missile parameters:")]
    [Tooltip("How far off boresight the missile can see the target. Also restricts how far the missile can lead.")]
    public float seekerCone = 70.0f;

    [Tooltip("How far off boresight the missile can see the target. Also restricts how far the missile can lead.")]
    public float seekerRange = 50000f;

    [Tooltip("How long the missile will accelerate. After this, the missile maintains a constant speed.")]
    public float motorLifetime = 10f;

    [Tooltip("How many degrees per second the missile can turn.")]
    public float turnRate = 180f;

    [Tooltip("After this time, the missile will self-destruct. Timer starts on launch, not motor activation.")]
    public float timeToLive = 60f;

    [Tooltip("Proxy Fuze trigger range.")]
    public float proxy = 40f;

    [Header("Drop options:")]
    [Tooltip("If greater than 0, missile will free fall for this many seconds and then activate after this many seconds have elapsed.")]
    public float dropDelay = 2f;
    private GameObject playerShip;


    private float launchTime = 0.0f;
    private float activateTime = 0.0f;
    private float missileSpeed = 0.0f;

    public bool isLaunched = false;
    public bool missileActive = false;
    public bool motorActive = false;
    public bool targetTracking = true;

    private float ejectSpeed = 7.5f;
    // Used to prevent lead markers from getting huge when missiles are very slow.
    private const float MINIMUM_GUIDE_SPEED = 1.0f;

    public bool MissileLaunched { get { return isLaunched; } }
    public bool MotorActive { get { return motorActive; } }

    private Rigidbody missileRigidbody;
    new CapsuleCollider collider;

    private float timeToImpact = 0;
    private float distToTarget = 0f;
    private Quaternion desiredRot = Quaternion.identity;

    private float acceleration;
    private Vector3 missileVelocity = Vector3.zero;

    private void _init()
    {
        _mainEngine = _mainEngineGO.GetComponent<MainEngine>();
        missileRigidbody = transform.GetComponent<Rigidbody>();
        collider = GetComponent<CapsuleCollider>();
        missileRigidbody.isKinematic = false;

        acceleration = thrust / missileRigidbody.mass;

        // Sets it so that missile cannot collide with the thing that launched it.
        if (ownShip != null)
        {
            foreach (Collider col in ownShip.GetComponentsInChildren<Collider>())
                Physics.IgnoreCollision(collider, col);
        }

        playerShip = PlayerShipSingleton.Instance.gameObject;
    }

    private void FixedUpdate()
    {

        if (_guidence == null)
        {
            _guidence = transform.GetComponent<MissileGuidence>();
        }

        if ( TimeSince(launchTime) > dropDelay ) {
            ActivateMissile();
        }

        if (missileActive)
        {
            missileSpeed = missileSpeed + (acceleration * Time.fixedDeltaTime);

            if (missileActive && target != null)
            {
                MissileGuidance();
            }

            RunMissile();
        } 
    }

   
    public void Launch(Transform newTarget)
    {
        _init();

        if (!isLaunched)
        {
            isLaunched = true;
            launchTime = Time.time;
            transform.parent = null;
            target = newTarget;
            transform.GetComponent<Rigidbody>().detectCollisions = false;

            if (dropDelay > 0.0f)
            {

                missileVelocity = (transform.up * -ejectSpeed);
                missileRigidbody.velocity = missileVelocity;

            }
            else
            {
                ActivateMissile();
            }
        }
    }

    private void RunMissile()
    {

        if (isLaunched)
        {
            // Don't start moving under own power until drop delay has passed (if applicable).
            if (!missileActive && dropDelay > 0.0f && TimeSince(launchTime) > dropDelay)
            {
                ActivateMissile();
                transform.GetComponent<Rigidbody>().detectCollisions = true;
            }

            if (missileActive)
            {
                if (motorLifetime > 0.0f && TimeSince(activateTime) > motorLifetime)
                {
                    _mainEngine.throttle = 0f;
                    motorActive = false;
                }
                else
                {
                    _mainEngine.throttle = 1f;
                    motorActive = true;
                }

                if (targetTracking)
                {
                    

                    transform.rotation = Quaternion.RotateTowards(transform.rotation, desiredRot, turnRate * Time.fixedDeltaTime);
                    missileRigidbody.velocity = transform.forward * missileSpeed;
                    missileRigidbody.angularVelocity = Vector3.zero;
                } else
                {
                    missileActive = false;
                    _mainEngine.throttle = 0f;
                    motorActive = false;
                }
            }

            if (TimeSince(launchTime) > timeToLive || missileActive == false)
            {
                missileActive = false;
                DestroyMissile(false);
            }

        }
    }

    private void ActivateMissile()
    {
        missileActive = true;
        // If no motor lifetime is present, then the motor will just always be active.
        if (motorLifetime <= 0.0f)
        {
            motorActive = true;
        }
            
        activateTime = Time.time;
            
    }

    private void MissileGuidance()
    {
        
        Vector3 relPos = target.position - transform.position;

        float angleToTarget = Mathf.Abs(Vector3.Angle(transform.forward.normalized, relPos.normalized));
        float dist = Vector3.Distance(target.position, transform.position);

        distToTarget = dist;
        
        if (angleToTarget > seekerCone || dist > seekerRange)
        {
            targetTracking = false;
        }

        if (targetTracking)
        {
            float predictedSpeed = Mathf.Min(acceleration * motorLifetime, missileSpeed + acceleration * TimeSince(activateTime));
            timeToImpact = dist / Mathf.Max(predictedSpeed, MINIMUM_GUIDE_SPEED);
        }

        //Debug.Log("Speed = " + missileSpeed);

        desiredRot = _guidence.QuadraticPN(missileSpeed);

    }
    private void DestroyMissile(bool impact)
    {
        
        //FloatingOriginSingleton.Instance.gameObject.GetComponent<FloatingOrigin>().floatingObjectsShips.Remove(gameObject);
        //FloatingOriginSingleton.Instance.gameObject.GetComponent<FloatingOrigin>().floatingObjectsScript.Remove(gameObject.GetComponent<ThisFloatingObject>());

        Destroy(gameObject);
    }

    private float TimeSince(float since)
    {
        return Time.time - since;
    }

    private void OnCollisionEnter(Collision collision)
    {
        // Prevent missile from exploding if it hasn't activated yet.
        if (isLaunched && TimeSince(launchTime) > dropDelay)
        {
            // This is a good place to apply damage based on what was collided with.
            Debug.Log("Hit");

            DestroyMissile(true);
        }
    }


}
