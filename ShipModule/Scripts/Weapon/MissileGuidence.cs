using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MissileGuidence : MonoBehaviour
{

    public Transform target;
    public Rigidbody missileRB;

    public Vector3 targetsVelocity = Vector3.zero;
    public Vector3 missileVelocity = Vector3.zero;

    private InertiaTensor targetInertiaTensor;
    private void _Init()
    {
        target = PlayerShipSingleton.Instance.gameObject.GetComponent<TargetManager>().target;
        targetInertiaTensor = target.GetComponent<InertiaTensor>();
        missileRB = transform.GetComponent<Rigidbody>();
    }

    public Quaternion QuadraticPN(float speed)
    {
        _Init();

        targetsVelocity = targetInertiaTensor.editorVelocity;
        missileVelocity = missileRB.velocity;


        Vector3 direction;
        Quaternion target_rotation = Quaternion.identity;

        if (GetInterceptDirection(transform.position, target.position, speed, targetsVelocity, out direction))
        {
            //Debug.Log(direction);
            target_rotation = Quaternion.LookRotation(direction);
        }
        else
        {
            //well, I guess we cant intercept then
        }

        return target_rotation;
    }

    static bool GetInterceptDirection(Vector3 origin, Vector3 targetPosition, float missileSpeed, Vector3 targetVelocity, out Vector3 result)
    {

        var los = origin - targetPosition;
        var distance = los.magnitude;
        var alpha = Vector3.Angle(los, targetVelocity) * Mathf.Deg2Rad;
        var vt = targetVelocity.magnitude;
        var vRatio = vt / missileSpeed;

        //solve the triangle, using cossine law
        if (SolveQuadratic(1 - (vRatio * vRatio), 2 * vRatio * distance * Mathf.Cos(alpha), -distance * distance, out var root1, out var root2) == 0)
        {
            result = Vector3.zero;
            return false;   //no intercept solution possible!
        }

        var interceptVectorMagnitude = Mathf.Max(root1, root2);
        var time = interceptVectorMagnitude / missileSpeed;
        var estimatedPos = targetPosition + targetVelocity * time;
        result = (estimatedPos - origin).normalized;

        return true;
    }

    static int SolveQuadratic(float a, float b, float c, out float root1, out float root2)
    {

        var discriminant = b * b - 4 * a * c;

        if (discriminant < 0)
        {
            root1 = Mathf.Infinity;
            root2 = -root1;
            return 0;
        }

        root1 = (-b + Mathf.Sqrt(discriminant)) / (2 * a);
        root2 = (-b - Mathf.Sqrt(discriminant)) / (2 * a);

        return discriminant > 0 ? 2 : 1;
    }

    void OnTriggerEnter(Collider c)
    {
        Debug.Log("HIT");
    }





}
