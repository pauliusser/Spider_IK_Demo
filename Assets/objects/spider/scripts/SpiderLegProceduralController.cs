using UnityEngine;

[ExecuteAlways]
public class SpiderLegProceduralController : MonoBehaviour
{
    [Tooltip("Parent GameObject with multiple Circle Gizmos")]
    public GameObject parentObject;

    [Tooltip("Index of the AdvancedCircleGizmo to follow")]
    public int gizmoIndex = 0;

    [Tooltip("Index of the tick to follow")]
    public int tickIndex = 0;
    [Tooltip("Leg controller settings")]
    public Transform target;
    public float driftRadius = 0.16f;
    public int legSet = 0;
    public int legIndexInSet = 0;

    // Debug
    public bool enableDebug = true;

    // Step interpolation variables
    [Header("Step Interpolation")]
    public float baseStepDuration = 2f; // Base duration when stationary
    public float stepHeight = 0.1f; // Height to lift leg during step
    public AnimationCurve stepCurve = AnimationCurve.EaseInOut(0, 0, 1, 1); // For timing
    public float speedMultiplier = 1f; // How much speed affects step speed
    public float distanceInfluenceMultiplier = 2f;
    public bool overrideStepTarget = false;
    public float stepSpeedMultiplier = 1f; // Controlled by SpiderLocomotionController
    
    private AdvancedCircleGizmo parentGizmo;
    private Vector3 previousPosition;
    private float legDrift = 0f;
    public float alowedLegDrift = 0.16f;
    private Vector3 nexStepPositionXZ;
    public AdvancedCircleGizmo radiusGizmo;
    public AdvancedCircleGizmo directionGizmo;
    public Vector3 stepTargetLocalOverride = Vector3.zero;

    public float reductionIncrement = 0.01f;
    private SpiderLocomotionController locomotionController;
    private Vector3 prevPos;
    private Vector3 deltaPos;
    
    // Interpolation state
    public bool isMoving = false;
    public float stepProgress = 0f; // 0 to 1, driven by time+distance
    private Vector3 startTargetPos;
    private Vector3 controlTargetPos;
    private Vector3 endTargetPos;
    
    // Speed tracking
    private Vector3 bodyVelocity = Vector3.zero;
    private float currentSpeed = 0f;
    private float stepDistance = 0f;
    private Vector3 positionAtStepStart;

    void OnEnable()
    {
        locomotionController = GetComponentInParent<SpiderLocomotionController>();
        AdvancedCircleGizmo[] circleGizmos = gameObject.GetComponents<AdvancedCircleGizmo>();
        if (circleGizmos.Length >= 2)
        {
            radiusGizmo = circleGizmos[0];
            directionGizmo = circleGizmos[1];
            radiusGizmo.radius = driftRadius;
        }
    }
    
    void Start()
    {
        previousPosition = transform.position;
        prevPos = transform.position;
        legDrift = GetLegDrift();
        SetInitialTargetPosition();
    }    

    private void Update()
    {
        deltaPos = transform.position - prevPos;
        
        // Calculate body velocity (how fast the spider is moving)
        if (Application.isPlaying && Time.deltaTime > 0f)
        {
            // Smooth velocity calculation
            Vector3 instantVelocity = deltaPos / Time.deltaTime;
            bodyVelocity = Vector3.Lerp(bodyVelocity, instantVelocity, Time.deltaTime * 10f);
            currentSpeed = bodyVelocity.magnitude;
        }
        
        if (Application.isPlaying)
        {
            AjustGizmoDownDir();

            legDrift = GetLegDrift();
            SetLegMotionDirection();
            
            // Check if we need to start a new step
            if (!isMoving && legDrift > alowedLegDrift)
            {
                // Check if this leg's set is allowed to step by the locomotion controller
                if (locomotionController == null || locomotionController.CanLegSetStep(legSet))
                {
                    StartStep();
                }
                // else: wait for its turn
            }
            
            // Handle ongoing step interpolation
            if (isMoving)
            {
                
                // SetLegMotionDirection();
                UpdateStep();
            }
        }
        // else
        // {
        //     PositionOnParentGizmo_Editor();
        // }
        prevPos = transform.position;
    }
    // void AjustGizmoDownDir()
    // {
    //     int layerMask = LayerMask.GetMask("Ground");
    //     Vector3 avgNormal = FibonachiSphere.AvgNormalInSphere(transform.position, -transform.parent.up, layerMask, 32, 1.5f, 90f);
    //     transform.localEulerAngles = Vector3.zero;
    //     Quaternion rotation = Quaternion.FromToRotation(transform.up, avgNormal);
    //     transform.rotation = rotation * transform.rotation;
    // }

    void AjustGizmoDownDir()
{
    int layerMask = LayerMask.GetMask("Ground");
    Vector3 avgNormal = FibonachiSphere.AvgNormalInSphere(transform.position, -transform.parent.up, layerMask, 32, 1.5f, 90f);
    Quaternion targetRotation = Quaternion.FromToRotation(transform.up, avgNormal) * transform.rotation;
    float blendFactor = 0.4f; // 0 = no rotation, 1 = full rotation
    transform.rotation = Quaternion.Slerp(transform.rotation, targetRotation, blendFactor);
}

    void StartStep()
    {
        if (directionGizmo == null || directionGizmo.tickLocalPositions == null || directionGizmo.tickLocalPositions.Length == 0)
            return;

        // Determine target local position
        Vector3 targetLocalPos = overrideStepTarget ? stepTargetLocalOverride : directionGizmo.tickLocalPositions[0];
        Vector3 rayStart = transform.TransformPoint(targetLocalPos);
        Vector3 down = transform.TransformDirection(Vector3.down);

        RaycastHit hit;
        float rayLength = 0.65f;
        int layerMask = LayerMask.GetMask("Ground");
        float radius = 0.15f;

        // if (Physics.Raycast(rayStart, down, out hit, rayLength, layerMask))
        // {
        //     startTargetPos = target.position;
        //     endTargetPos = hit.point;
        //     stepDistance = Vector3.Distance(startTargetPos, endTargetPos);

        //     // Midpoint and lift
        //     Vector3 midPoint = Vector3.Lerp(startTargetPos, endTargetPos, 0.5f);
        //     Vector3 liftOffset = transform.TransformDirection(Vector3.up) * stepHeight;
        //     RaycastHit midHit;
        //     if (Physics.Raycast(midPoint + liftOffset * 2f, down, out midHit, rayLength * 2f, layerMask))
        //         midPoint = midHit.point;
        //     controlTargetPos = midPoint + liftOffset;

        //     stepProgress = 0f;
        //     positionAtStepStart = transform.position;
        //     isMoving = true;

        //     if (enableDebug)
        //         DrawBezierPath(startTargetPos, controlTargetPos, endTargetPos, Color.cyan);
        // }
        // else 
        if (Physics.SphereCast(rayStart + transform.up * radius, radius, down, out hit, 1.5f, layerMask))
        {
            startTargetPos = target.position;
            endTargetPos = hit.point;
            stepDistance = Vector3.Distance(startTargetPos, endTargetPos);

            // Midpoint and lift
            Vector3 midPoint = Vector3.Lerp(startTargetPos, endTargetPos, 0.5f);
            Vector3 liftOffset = transform.TransformDirection(Vector3.up) * stepHeight;
            RaycastHit midHit;
            if (Physics.Raycast(midPoint + liftOffset * 2f, down, out midHit, rayLength * 2f, layerMask))
                midPoint = midHit.point;
            controlTargetPos = midPoint + liftOffset;

            stepProgress = 0f;
            positionAtStepStart = transform.position;
            isMoving = true;
        }

        alowedLegDrift = driftRadius;
        radiusGizmo.radius = driftRadius;
    }

    void UpdateStep()
    {
        if (!isMoving) return;

        float timeProgress = Time.deltaTime / baseStepDuration;
        float distanceThisFrame = deltaPos.magnitude;
        float distanceProgress = (distanceThisFrame / driftRadius) * distanceInfluenceMultiplier;

        // Apply speed multiplier (e.g., 2x for emergency)
        float totalProgress = (timeProgress + distanceProgress) * stepSpeedMultiplier;

        stepProgress += totalProgress;

        if (stepProgress >= 1f)
        {
            stepProgress = 1f;
            isMoving = false;
            target.position = endTargetPos;
        }

        float curveT = stepCurve.Evaluate(stepProgress);
        Vector3 newPos = CalculateBezierPoint(startTargetPos, controlTargetPos, endTargetPos, curveT);
        target.position = newPos;

        if (enableDebug)
            Debug.DrawLine(target.position, target.position + Vector3.up * 0.05f, Color.magenta);
    }
    
    Vector3 CalculateBezierPoint(Vector3 p0, Vector3 p1, Vector3 p2, float t)
    {
        float u = 1 - t;
        float tt = t * t;
        float uu = u * u;
        
        Vector3 point = uu * p0;
        point += 2 * u * t * p1;
        point += tt * p2;
        
        return point;
    }
    
    void DrawBezierPath(Vector3 p0, Vector3 p1, Vector3 p2, Color color)
    {
        // Draw control lines
        Debug.DrawLine(p0, p1, new Color(color.r, color.g, color.b, 0.3f), baseStepDuration);
        Debug.DrawLine(p1, p2, new Color(color.r, color.g, color.b, 0.3f), baseStepDuration);
        
        // Draw curve
        int segments = 10;
        Vector3 prevPoint = p0;
        for (int i = 1; i <= segments; i++)
        {
            float t = i / (float)segments;
            Vector3 point = CalculateBezierPoint(p0, p1, p2, t);
            Debug.DrawLine(prevPoint, point, color, baseStepDuration);
            prevPoint = point;
        }
        
        // Draw control points
        Debug.DrawLine(p0, p0 + Vector3.up * 0.05f, Color.white, baseStepDuration);
        Debug.DrawLine(p1, p1 + Vector3.up * 0.05f, Color.yellow, baseStepDuration);
        Debug.DrawLine(p2, p2 + Vector3.up * 0.05f, Color.green, baseStepDuration);
    }

    void SetInitialTargetPosition()
    {
        if (directionGizmo == null || directionGizmo.tickLocalPositions == null || directionGizmo.tickLocalPositions.Length == 0)
            return;
            
        nexStepPositionXZ = directionGizmo.tickLocalPositions[0];
        Vector3 halfstepForwardPos = nexStepPositionXZ / 2;
        Vector3 halfstepBackwardsPos = -halfstepForwardPos;

        Vector3 rayPos = transform.TransformPoint(legSet == 0 ? halfstepBackwardsPos : halfstepForwardPos);
        Vector3 down = transform.TransformDirection(Vector3.down);
        
        if (enableDebug)
            Debug.DrawRay(rayPos, down, Color.yellow);
        
        int layerMask = LayerMask.GetMask("Ground");
        RaycastHit hit;
        float rayLength = 2f;
        
        if (Physics.Raycast(rayPos, down, out hit, rayLength, layerMask))
        {
            target.position = hit.point;
            if (enableDebug)
                Debug.DrawLine(hit.point, hit.point + Vector3.up * 0.04f, Color.green, 0.2f);
        }
    }

    float GetLegDrift()
    {
        if (target == null) return 0f;
        
        Vector3 targetOnLocalYplane = transform.InverseTransformPoint(target.position);
        targetOnLocalYplane.y = 0;
        Vector3 targetProjected = transform.TransformPoint(targetOnLocalYplane);
        
        if (enableDebug)
            Debug.DrawLine(transform.position, targetProjected, Color.red);
        
        return targetOnLocalYplane.magnitude;
    }

    // void SetLegMotionDirection()
    // {
    //     Vector3 movementDirection = transform.position - previousPosition;
        
    //     if (movementDirection.magnitude > 0.003f)
    //     {
    //         Vector3 horizontalDirection = new Vector3(movementDirection.x, 0, movementDirection.z);
            
    //         if (horizontalDirection.magnitude > 0.001f)
    //         {
    //             if (transform.parent != null)
    //             {
    //                 horizontalDirection = transform.parent.InverseTransformDirection(horizontalDirection);
    //             }
                
    //             Quaternion targetLocalRotation = Quaternion.LookRotation(horizontalDirection, Vector3.up);
    //             Vector3 targetEuler = targetLocalRotation.eulerAngles;
    //             Vector3 currentEuler = transform.localEulerAngles;
                
    //             float newY = Mathf.LerpAngle(currentEuler.y, targetEuler.y, Time.deltaTime * 10f);
    //             transform.localEulerAngles = new Vector3(currentEuler.x, newY, currentEuler.z);
    //         }
    //     }
        
    //     previousPosition = transform.position;
    // }

    void SetLegMotionDirection()
    {
        Debug.DrawLine(transform.position, transform.position + transform.up.normalized * 0.1f, Color.red);
        Vector3 movementDirection = transform.position - previousPosition;
        
        if (movementDirection.magnitude > 0.003f)
        {      
            Vector3 motionVectorOnLocalPlane = Vector3.ProjectOnPlane(movementDirection, transform.up).normalized;
            float angle = Vector3.SignedAngle(transform.forward, motionVectorOnLocalPlane, transform.up);
            Quaternion deltaRotation = Quaternion.AngleAxis(angle, transform.up);
            transform.rotation = deltaRotation * transform.rotation;
        }
        
        previousPosition = transform.position;
    }

    void PositionOnParentGizmo_Editor()
    {
        if (parentObject == null) return;
        AdvancedCircleGizmo[] gizmos = parentObject.GetComponents<AdvancedCircleGizmo>();
        if (gizmos == null || gizmoIndex < 0 || gizmoIndex >= gizmos.Length) return;
        parentGizmo = gizmos[gizmoIndex];
        if (parentGizmo.tickLocalPositions == null || tickIndex < 0 || tickIndex >= parentGizmo.tickLocalPositions.Length)
            return;
        transform.localPosition = parentGizmo.tickLocalPositions[tickIndex];
    }
}