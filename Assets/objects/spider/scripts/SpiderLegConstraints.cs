using UnityEngine;

public class SpiderLegConstraints : MonoBehaviour
{
    [Header("Constraints")]
    [Range(-180, 180)] public float minXReachAngle = 0f;
    [Range(-180, 180)] public float maxXReachAngle = 0f;
    [Range(-180, 180)] public float minYReachAngle = 0f;
    [Range(-180, 180)] public float maxYReachAngle = 0f;
    [Range(-180, 180)] public float minXRotAngle = 0f;
    [Range(-180, 180)] public float maxXRotAngle = 0f;
    [Range(-180, 180)] public float minYRotAngle = 0f;
    [Range(-180, 180)] public float maxYRotAngle = 0f;
    [Range(-180, 180)] public float minZRotAngle = 0f;
    [Range(-180, 180)] public float maxZRotAngle = 0f;
    [Range(0, 1)] public float stiffness = 1f;
    
    [Header("Configuration")]
    public Vector3 initialPose;
    public bool isRightSide = false;

    void Awake()
    {
        // Store initial rotation
        initialPose = transform.localEulerAngles;
    }
    
    void Reset()
    {
        // Set initial pose when component is added
        initialPose = transform.localEulerAngles;
    }
}