using UnityEngine;
using UnityEngine.InputSystem;

public class CameraOrbit : MonoBehaviour
{
    [Header("Target")]
    public Transform target;           // The object to orbit around

    [Header("Orbit Settings")]
    public float rotationSpeed = 5f;    // Mouse sensitivity
    public float zoomSpeed = 2f;        // Scroll sensitivity
    public float minZoom = 2f;           // Minimum distance from target
    public float maxZoom = 10f;          // Maximum distance from target
    public bool invertY = false;         // Invert vertical rotation

    [Header("Smoothing")]
    public bool smoothZoom = true;
    public float zoomSmoothTime = 0.1f;

    [Header("Input Actions")]
    public InputAction pan;              // Right mouse button hold
    public InputAction look;              // Mouse delta
    public InputAction zoom;              // Mouse scroll wheel

    // Private
    private float currentRotationX;
    private float currentRotationY;
    private float currentZoom;
    private float targetZoom;
    private float zoomVelocity;
    private bool isDragging;

    private void Awake()
    {
        // Set up input bindings programmatically
        pan = new InputAction("Pan", binding: "<Mouse>/rightButton");
        look = new InputAction("Look", binding: "<Mouse>/delta");
        zoom = new InputAction("Zoom", binding: "<Mouse>/scroll/y");
    }

    private void OnEnable()
    {
        pan.Enable();
        look.Enable();
        zoom.Enable();
        
        pan.performed += OnPanPerformed;
        pan.canceled += OnPanCanceled;
    }

    private void OnDisable()
    {
        pan.performed -= OnPanPerformed;
        pan.canceled -= OnPanCanceled;
        
        pan.Disable();
        look.Disable();
        zoom.Disable();
    }

    private void OnPanPerformed(InputAction.CallbackContext ctx)
    {
        isDragging = true;
    }

    private void OnPanCanceled(InputAction.CallbackContext ctx)
    {
        isDragging = false;
    }

    private void Start()
    {
        if (target == null) return;

        // Capture initial distance as zoom
        Vector3 directionToTarget = target.position - transform.position;
        currentZoom = directionToTarget.magnitude;
        targetZoom = currentZoom;
        
        // Set min/max zoom relative to initial distance
        minZoom = Mathf.Min(minZoom, currentZoom * 0.5f);
        maxZoom = Mathf.Max(maxZoom, currentZoom * 2f);
        
        // Initialize rotation from current transform
        Vector3 angles = transform.eulerAngles;
        currentRotationX = angles.y;
        currentRotationY = angles.x;
    }

    private void Update()
    {
        if (target == null) return;
        
        HandleRotation();
        HandleZoom();
    }

    private void LateUpdate()
    {
        if (target == null) return;
        
        ApplyCameraTransform();
    }

    private void HandleRotation()
    {
        if (!isDragging) return;

        // Get mouse delta from input action
        Vector2 delta = look.ReadValue<Vector2>();
        
        // Apply rotation speed
        currentRotationX += delta.x * rotationSpeed * 0.1f;
        currentRotationY += delta.y * rotationSpeed * 0.1f * (invertY ? 1 : -1);
        
        // Clamp vertical rotation to prevent flipping
        currentRotationY = Mathf.Clamp(currentRotationY, -89f, 89f);
    }

    private void HandleZoom()
    {
        // Get scroll value from input action
        float scroll = zoom.ReadValue<float>();
        
        if (scroll != 0)
        {
            targetZoom = Mathf.Clamp(targetZoom - scroll * zoomSpeed, minZoom, maxZoom);
        }

        if (smoothZoom)
        {
            currentZoom = Mathf.SmoothDamp(currentZoom, targetZoom, ref zoomVelocity, zoomSmoothTime);
        }
        else
        {
            currentZoom = targetZoom;
        }
    }

    private void ApplyCameraTransform()
    {
        // Calculate rotation
        Quaternion rotation = Quaternion.Euler(currentRotationY, currentRotationX, 0);
        
        // Calculate position: start at target, move back by zoom distance
        Vector3 positionOffset = rotation * new Vector3(0, 0, -currentZoom);
        
        // Apply position and rotation
        transform.position = target.position + positionOffset;
        transform.LookAt(target.position);
    }

    // Optional: Public methods
    public void SetTarget(Transform newTarget)
    {
        target = newTarget;
        
        if (target != null && isActiveAndEnabled)
        {
            // Recalculate distance for new target
            Vector3 directionToTarget = target.position - transform.position;
            currentZoom = directionToTarget.magnitude;
            targetZoom = currentZoom;
        }
    }

    public void SetTarget(GameObject newTarget)
    {
        SetTarget(newTarget?.transform);
    }

    public void ResetRotation()
    {
        Vector3 angles = transform.eulerAngles;
        currentRotationX = angles.y;
        currentRotationY = angles.x;
    }
}