using UnityEngine;

using UnityEngine.InputSystem;

public class SpiderPlayerInputs : MonoBehaviour
{
    [Header("Input Action")]
    public InputAction moveAction;
    private SpiderMotionController motionControll;
    Vector3 playerInput;
    private void OnEnable() => moveAction.Enable();
    private void OnDisable() => moveAction.Disable();
    void Start()
    {
        motionControll = GetComponent<SpiderMotionController>();
    }

    // Update is called once per frame
    void Update()
    {
        playerInput = moveAction.ReadValue<Vector3>();
        motionControll.inputX = playerInput.x;
        motionControll.inputY = playerInput.y;
        motionControll.inputZ = playerInput.z;
    }
}
