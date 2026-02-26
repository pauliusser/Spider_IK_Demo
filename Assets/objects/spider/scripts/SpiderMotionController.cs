
using Unity.VisualScripting;
using UnityEngine;

public class SpiderMotionController : MonoBehaviour
{
    // MARK: Public
    [Header("Raycasting")]
    public LayerMask groundLayerMask;
    public float rayLength = 1f;
    public int avgNormalResolution = 64;
    public float coveredNormalAngle = 90f;
    public bool debug = false;

    [Header("Movement")]
    public float step = 0.3f;                 // distance between nodes
    public float height = 0.35f;              // above ground
    public float spiderSpeed = 0.5f;          // mulriplier
    public float rotationSpeed = 90f;         // degrees per second

    [Header("Input")]
    [Range(-1f, 1f)] public float inputX = 0f;
    [Range(-1f, 1f)] public float inputY = 0f;
    [Range(-1f, 1f)] public float inputZ = 0f;

    // MARK: Private
    private Vector3[] rayDirections;
    private Vector3 inputsXYZ;
    private Vector3 prevXYZ;
    private bool isStationary;
    private bool isTurning;
    private bool inputChanged;
    private bool isStoped = false;
    private float stepDuration;
    private float totalStepYaw;
    private float curveProgress;
    private int validNodeCount;
    private Vector3 previousDebugPos;

    // MARK: PathNode
    private struct PathNode
    {
        public Vector3 position;
        public Vector3 forward;
        public Vector3 up;
        public Vector3 tangent;
    }
    private PathNode[] nodes = new PathNode[3];

    // MARK: Start
    private void Start()
    {
        InitializeFirstNode();
        prevXYZ = Vector3.zero;
        previousDebugPos = transform.position;
    }

    // MARK: Update
    private void Update()
    {
        inputsXYZ.x = inputX;
        inputsXYZ.y = inputY;
        inputsXYZ.z = inputZ;

        isStationary = (inputX == 0f && inputY == 0f);
        isTurning = (inputZ != 0f);
        inputChanged = inputsXYZ != prevXYZ;

        // MARK: ifStationary
        if (isStationary)
        {
            isStoped = false;
            if (isTurning)
            {
                float yawThisFrame = inputZ * rotationSpeed * Time.deltaTime;
                transform.rotation = Quaternion.AngleAxis(yawThisFrame, transform.up) * transform.rotation;
                nodes[0].forward = transform.forward;
                nodes[0].up = transform.up;
                UpdateNodeTangent(ref nodes[0]);
            }
            prevXYZ = inputsXYZ;
            return;
        }

        // MARK: inputChanged
        if (inputChanged)
        {
            isStoped = false;
            // Drift correction
            Vector3 desiredUp = SampleGroundNormal(transform.position, -transform.up);
            if (desiredUp != Vector3.zero)
            {
                transform.rotation = Quaternion.FromToRotation(transform.up, desiredUp) * transform.rotation;
            }

            // Reset path to current transform
            nodes[0].position = transform.position;
            nodes[0].forward = transform.forward;
            nodes[0].up = transform.up;
            UpdateNodeTangent(ref nodes[0]);
            validNodeCount = 1;
            curveProgress = 0f;
        }

        // MARK: step params
        float inputMag = new Vector3(inputX, inputY, 0).magnitude;
        if (inputMag < 0.01f) inputMag = 0.1f; // safety
        stepDuration = 1f / (spiderSpeed * inputMag);
        totalStepYaw = inputZ * rotationSpeed * stepDuration;

        // MARK: node count handle
        while (validNodeCount < 2)
        {
            nodes[validNodeCount] = GenerateNextNode(nodes[validNodeCount - 1], totalStepYaw);
            if (isStoped) return;
            validNodeCount++;
        }
        while (validNodeCount < 3)
        {
            nodes[validNodeCount] = GenerateNextNode(nodes[validNodeCount - 1], totalStepYaw);
            if (isStoped) return;
            validNodeCount++;
        }

        // MARK: Advance
        curveProgress += Time.deltaTime / stepDuration;
        if (curveProgress >= 1f)
        {
            nodes[0] = nodes[1];
            nodes[1] = nodes[2];
            validNodeCount = 2;
            curveProgress = 0f;
        }
        curveProgress = Mathf.Clamp01(curveProgress);

        // MARK: Interpolate
        PathNode start = nodes[0];
        PathNode end = nodes[1];
        float L = Vector3.Distance(start.position, end.position);
        float oneThirdL = L / 3f;
        Vector3 p1 = start.position + start.tangent * oneThirdL;
        Vector3 p2 = end.position - end.tangent * oneThirdL;
        Vector3 targetPos = CubicBezier(start.position, p1, p2, end.position, curveProgress);
        Quaternion targetRot = Quaternion.Slerp(
            Quaternion.LookRotation(start.forward, start.up),
            Quaternion.LookRotation(end.forward, end.up),
            curveProgress
        );

        transform.position = targetPos;
        transform.rotation = targetRot;

        prevXYZ = inputsXYZ;

        // MARK: Debug
        if (debug)
        {
            Debug.DrawLine(previousDebugPos, transform.position, Color.red, 5f);
            previousDebugPos = transform.position;

            Debug.DrawLine(start.position, p1, Color.cyan);
            Debug.DrawLine(end.position, p2, Color.cyan);
            Debug.DrawRay(start.position, start.tangent * 0.5f, Color.magenta);
            Debug.DrawRay(end.position, end.tangent * 0.5f, Color.magenta);
        }
    }

    // MARK: First Node init
    private void InitializeFirstNode()
    {
        nodes[0].position = transform.position;
        nodes[0].forward = transform.forward;
        nodes[0].up = transform.up;
        UpdateNodeTangent(ref nodes[0]);
        validNodeCount = 1;
    }
    // MARK: update target
    private void UpdateNodeTangent(ref PathNode node)
    {
        Vector3 right = Vector3.Cross(node.up, node.forward).normalized;
        Vector3 moveDir = (node.forward * inputY + right * inputX).normalized;
        node.tangent = moveDir;
    }
    // MARK: GENERATE NEXT
    private PathNode GenerateNextNode(PathNode from, float stepYaw)
    {
        PathNode newNode = new PathNode();
        float halfYaw = stepYaw * 0.5f;

        // 1. Half yaw around current up
        Quaternion halfYawRot1 = Quaternion.AngleAxis(halfYaw, from.up);
        Vector3 midForward = halfYawRot1 * from.forward;

        // 2. Movement direction from mid orientation
        Vector3 right = Vector3.Cross(from.up, midForward).normalized;
        Vector3 moveDir = (midForward * inputY + right * inputX).normalized;

        // 3. Intermediate position
        Vector3 intermediatePos = from.position + moveDir * step;

        // 4. Sample new up
        Vector3 newUp = SampleGroundNormal(intermediatePos, -from.up);
        if (newUp == Vector3.zero)
        {
            // Can't find ground - don't move this frame
            Debug.Log("Spider stopped - no ground detected");
            isStoped = true;

            // Return a safe node at current position with same orientation
            newNode.position = from.position;
            newNode.forward = from.forward;
            newNode.up = from.up;
            UpdateNodeTangent(ref newNode);
            return newNode;
        }

        // 5. Remaining half yaw around new up
        Quaternion halfYawRot2 = Quaternion.AngleAxis(halfYaw, newUp);
        Vector3 forwardAfterYaw = halfYawRot2 * midForward;

        // 6. Align with new surface
        Quaternion surfaceRot = Quaternion.FromToRotation(from.up, newUp);
        newNode.forward = surfaceRot * forwardAfterYaw;
        newNode.up = newUp;

        // 7. Adjust height
        newNode.position = AdjustPositionToHeight(intermediatePos, newUp);

        // 8. Compute tangent
        UpdateNodeTangent(ref newNode);

        if (debug)
        {
            Debug.DrawLine(from.position, newNode.position, Color.yellow, 3f);
            Debug.DrawLine(newNode.position, newNode.position + newNode.up * 0.2f, Color.green, 3f);
            Debug.DrawLine(newNode.position, newNode.position + newNode.forward * 0.2f, Color.blue, 3f);
        }

        return newNode;
    }

    // MARK: avg normal
    private Vector3 SampleGroundNormal(Vector3 position, Vector3 referenceDown)
    {
        Vector3 normal = FibonachiSphere.AvgNormalInSphere(
            position,
            referenceDown,
            groundLayerMask,
            avgNormalResolution,
            rayLength,
            coveredNormalAngle,
            debug
        );

        if (normal == Vector3.zero)
        {
            normal = FibonachiSphere.AvgNormalInSphere(
                position,
                referenceDown,
                groundLayerMask,
                avgNormalResolution * 2,
                rayLength,
                coveredNormalAngle / 2,
                debug
            );
        }
        if (normal == Vector3.zero)
        {
            RaycastHit hit;
            
            if (Physics.SphereCast(
                position + referenceDown.normalized * rayLength * step, // step size up offset
                step, // step size radius = almost guaranteed hit
                referenceDown,
                out hit,
                rayLength,
                groundLayerMask))
            {
                normal = hit.normal;
                Debug.Log("Sphere cast fallback succeeded on thin edge");
                
                if (debug)
                    Debug.DrawLine(hit.point, hit.point + hit.normal * 0.3f, Color.magenta, 2f);
            }
        }
        return normal;
    }

    private Vector3 AdjustPositionToHeight(Vector3 position, Vector3 up)
    {
        RaycastHit hit;
        if (Physics.Raycast(position, -up, out hit, height * 1.25f, groundLayerMask))
        {
            float error = height - (hit.point - position).magnitude;
            return position + up * error;
        }
        else if (Physics.SphereCast(position + up * height, height, -up, out hit, rayLength, groundLayerMask))
        {
            return position - up * (hit.distance - height);
        }
        return position; // fallback
    }

    // MARK: Bezier
    private Vector3 CubicBezier(Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3, float t)
    {
        float u = 1f - t;
        float u2 = u * u;
        float u3 = u2 * u;
        float t2 = t * t;
        float t3 = t2 * t;
        return u3 * p0 + 3f * u2 * t * p1 + 3f * u * t2 * p2 + t3 * p3;
    }
}