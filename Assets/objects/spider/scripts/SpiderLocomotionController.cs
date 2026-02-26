using UnityEngine;

public class SpiderLocomotionController : MonoBehaviour
{
    public SpiderLegProceduralController[] legControllers;

    [Header("Debug")]
    public bool enableDebug = true; // Master switch for all debug visualizations

    [Header("Gait Coordination")]
    [Range(0, 1)] public float targetPhaseOffset = 0.5f;
    public float phaseStrength = 0.3f;
    public int minGroundLegs = 4;

    [Header("Stationary Behavior")]
    public float stationaryThreshold = 0.01f;
    public float stationaryDelay = 2f;
    public float minStationaryScale = 0.5f;
    public float transitionSpeed = 2f;
    
    [Header("Step Alternation")]
    public float stepSetSwitchInterval = 0.2f;
    private int currentStepSet = 0;
    private float stepSetTimer = 0f;

    [Header("Emergency")]
    public float emergencyDriftMultiplier = 2f;
    public float emergencySpeedMultiplier = 3f;

    private Vector3 lastPos;
    private float stationaryTimer = 0f;
    private bool isStationary = false;
    private float[] setAverageProgress;

    void OnEnable()
    {
        legControllers = GetComponentsInChildren<SpiderLegProceduralController>();
        setAverageProgress = new float[2];
        lastPos = transform.position;
        stepSetTimer = stepSetSwitchInterval;
        
        // Propagate debug setting to all legs
        UpdateLegDebugSettings();
    }

    void Update()
    {
        if (!Application.isPlaying || legControllers == null || legControllers.Length == 0)
            return;

        // Movement detection
        Vector3 velocity = (transform.position - lastPos) / Time.deltaTime;
        float speed = velocity.magnitude;

        if (speed < stationaryThreshold)
            stationaryTimer += Time.deltaTime;
        else
            stationaryTimer = 0f;

        bool nowStationary = stationaryTimer >= stationaryDelay;
        if (nowStationary != isStationary)
            isStationary = nowStationary;

        // Update alternating step set
        stepSetTimer -= Time.deltaTime;
        if (stepSetTimer <= 0f)
        {
            stepSetTimer = stepSetSwitchInterval;
            currentStepSet = (currentStepSet == 0) ? 1 : 0;
        }

        // Phase error calculation
        int set0Count = 0, set1Count = 0;
        float set0Progress = 0f, set1Progress = 0f;
        foreach (var leg in legControllers)
        {
            if (leg.legSet == 0) { set0Progress += leg.stepProgress; set0Count++; }
            else { set1Progress += leg.stepProgress; set1Count++; }
        }
        if (set0Count > 0) setAverageProgress[0] = set0Progress / set0Count;
        if (set1Count > 0) setAverageProgress[1] = set1Progress / set1Count;

        float phaseDiff = Mathf.Repeat(setAverageProgress[1] - setAverageProgress[0], 1f);
        float error = targetPhaseOffset - phaseDiff;

        // Grounded legs count
        int groundedCount = 0;
        foreach (var leg in legControllers)
            if (!leg.isMoving) groundedCount++;

        int airborneCount = legControllers.Length - groundedCount;
        bool emergency = airborneCount > (legControllers.Length - minGroundLegs);

        // Apply adjustments per leg
        foreach (var leg in legControllers)
        {
            // Propagate debug setting to leg
            leg.enableDebug = this.enableDebug;

            // Set speed multiplier for airborne legs in emergency
            if (emergency && leg.isMoving)
                leg.stepSpeedMultiplier = emergencySpeedMultiplier;
            else
                leg.stepSpeedMultiplier = 1f;

            // Base target (before smoothing)
            float baseTarget = leg.driftRadius;

            // Stationary scaling
            if (isStationary)
                baseTarget *= minStationaryScale;

            // Phase adjustment
            float phaseAdj = (leg.legSet == 0 ? -error : error) * phaseStrength;
            baseTarget *= (1f + phaseAdj);

            // Smoothly move toward baseTarget
            leg.alowedLegDrift = Mathf.Lerp(leg.alowedLegDrift, baseTarget, Time.deltaTime * transitionSpeed);

            // EMERGENCY: If too few grounded, instantly expand allowed drift for grounded legs
            if (groundedCount < minGroundLegs && !leg.isMoving)
            {
                leg.alowedLegDrift = leg.driftRadius * emergencyDriftMultiplier;
            }

            // Clamp to safe range
            leg.alowedLegDrift = Mathf.Clamp(leg.alowedLegDrift, leg.driftRadius * 0.3f, leg.driftRadius * 2f);

            // Update visual gizmos (only update if debug is enabled)
            if (enableDebug)
            {
                if (leg.radiusGizmo != null)
                    leg.radiusGizmo.radius = leg.alowedLegDrift;
                if (leg.directionGizmo != null)
                    leg.directionGizmo.radius = leg.alowedLegDrift * 0.95f;
            }

            // Stationary override
            leg.overrideStepTarget = isStationary;
            if (isStationary)
                leg.stepTargetLocalOverride = Vector3.zero;
        }

        lastPos = transform.position;
    }

    void UpdateLegDebugSettings()
    {
        if (legControllers == null) return;
        foreach (var leg in legControllers)
        {
            if (leg != null)
                leg.enableDebug = this.enableDebug;
        }
    }

    void OnValidate()
    {
        // When debug toggle changes in editor, propagate to children
        if (!Application.isPlaying)
        {
            UpdateLegDebugSettings();
        }
    }

    public bool CanLegSetStep(int legSet)
    {
        int groundedCount = 0;
        foreach (var leg in legControllers)
            if (!leg.isMoving) groundedCount++;
        
        bool emergency = (legControllers.Length - groundedCount) > (legControllers.Length - minGroundLegs);
        
        if (emergency)
            return true;
        
        return legSet == currentStepSet;
    }
}