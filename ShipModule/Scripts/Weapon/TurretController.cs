using UnityEngine;

namespace GT2.Demo
{
    public class TurretController : MonoBehaviour
    {
        [SerializeField] private TurretAim TurretAim = null;

        public Transform TargetPoint = null;
        private TargetManager playerShipTargetManager;

        //private bool isIdle = false;

        private void Awake()
        {
            TurretAim = gameObject.GetComponent<TurretAim>();

            if (TurretAim == null)
            {
                Debug.LogError(name + ": TurretController not assigned a TurretAim!");
            }
                

        }

        private void Update()
        {
            if (playerShipTargetManager == null)
            {
                playerShipTargetManager = PlayerShipSingleton.Instance.gameObject.GetComponent<TargetManager>();
            }

            TargetPoint = playerShipTargetManager.target;

            if (TurretAim == null)
                return;

            if (TargetPoint == null)
                TurretAim.IsIdle = TargetPoint == null;
            else
                TurretAim.AimPosition = TargetPoint.position;

            //if (Input.GetMouseButtonDown(0))
            //    TurretAim.IsIdle = !TurretAim.IsIdle;

            if (Input.GetKeyDown("space"))
            {
                TurretAim.IsIdle = !TurretAim.IsIdle;
            }


        }
    }
}
