using UnityEngine;

public class SpiderLegControls : MonoBehaviour
{
    [Header("Chain Configuration")]
    public int links = 7;
    public int legIndex = 0; // 0-3 for right legs, 4-7 for left legs is set manually in the inspector
    public int legPairIndex = 0; // 0-3 for each pair of legs, set manually in the inspector to determine which legs are paired together for symmetrical control
    public Transform target; // The target the leg should reach towards, set manually in the inspector
    [Header("Leg Control Weights")]
    [Range(-1f, 1f)] public float poseX = 0f; // extend leg
    [Range(-1f, 1f)] public float poseY = 0f; // point leg up - down
    [Range(-1f, 1f)] public float poseZ = 0f; // point leg left - right
    [Range(-1f, 1f)] private float poseW = 0f;
    public Vector3 localRootPosition;
    public Vector3 endEffectorPosition; // The end effector of the leg, set manually in the inspector

    // Internal state
    private Transform[] jointChain;
    private SpiderLegConstraints[] jointConstraints;

    void Awake()
    {
        InitialiseChain();
        InitialiseConstraints();
        localRootPosition = transform.localPosition;
        endEffectorPosition = jointChain[links - 1].position;
    }

    void InitialiseChain()
    {
        // Initialize joint chain based on hierarchy
        jointChain = new Transform[links];
        Transform current = transform;
        for (int i = 0; i < links && current != null; i++)
        {
            jointChain[i] = current;
            if (current.childCount > 0)
                current = current.GetChild(0);
            else if (i < links - 1)
                Debug.LogError($"Not enough child joints for the specified link count at {current.name}");
        }
    }

    void InitialiseConstraints()
    {
        jointConstraints = new SpiderLegConstraints[links - 1];
        for (int i = 0; i < links - 1; i++)
        {
            if (jointChain[i] != null)
            {
                jointConstraints[i] = jointChain[i].GetComponent<SpiderLegConstraints>();
                if (jointConstraints[i] == null)
                {
                    Debug.LogWarning($"Joint {jointChain[i].name} is missing a SpiderLegConstraints component.");
                }
            }
        }
    }

    void Update()
    {
        setPose();
    }
    public Vector3 setPose()
    {
        if (jointChain == null || jointChain[0] == null) return endEffectorPosition;
        
        for (int i = 0; i < links - 1; i++)
        {
            if (jointChain[i] != null && jointConstraints[i] != null)
            {
                Quaternion targetRotation = Quaternion.Euler(jointConstraints[i].initialPose);
                        
                float xReachAmount = Mathf.Lerp(
                    jointConstraints[i].minXReachAngle, 
                    jointConstraints[i].maxXReachAngle, 
                    (poseX + 1f) * 0.5f);
                targetRotation *= Quaternion.Euler(xReachAmount, 0, 0);
                
                float xRotAmount = Mathf.Lerp(
                    jointConstraints[i].minXRotAngle, 
                    jointConstraints[i].maxXRotAngle, 
                    (poseY + 1f) * 0.5f);
                targetRotation *= Quaternion.Euler(xRotAmount, 0, 0);
                
                float yReachAmount = Mathf.Lerp(
                    jointConstraints[i].minYReachAngle, 
                    jointConstraints[i].maxYReachAngle, 
                    (poseX + 1f) * 0.5f);
                
                float yRotAmount = Mathf.Lerp(
                    jointConstraints[i].minYRotAngle, 
                    jointConstraints[i].maxYRotAngle, 
                    (poseZ + 1f) * 0.5f);
                
                poseW = MapValue(poseZ, 0.75f);
                float zRotAmount = Mathf.Lerp(
                    jointConstraints[i].minZRotAngle, 
                    jointConstraints[i].maxZRotAngle, 
                    (poseW + 1f) * 0.5f);
                
                if (jointConstraints[i].isRightSide)
                {
                    yReachAmount = -yReachAmount;
                    yRotAmount = -yRotAmount;
                    zRotAmount = -zRotAmount;
                }
                
                // Apply Y reach rotation
                targetRotation *= Quaternion.Euler(0, yReachAmount, 0);
                // Apply Y and Z rotations
                targetRotation *= Quaternion.Euler(0, yRotAmount, 0);
                targetRotation *= Quaternion.Euler(0, 0, zRotAmount);
                
                jointChain[i].localRotation = targetRotation;
                endEffectorPosition = jointChain[links - 1].position;
            }
        }
        return endEffectorPosition;
    }
        float MapValue(float a, float strength = 0.5f)
    {

        float multiplier = strength + (1 - strength) * a * a;
        return a * multiplier;
    }
}