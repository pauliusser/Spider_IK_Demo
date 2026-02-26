using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
using UnityEngine.Rendering;
#endif

[ExecuteAlways]
public class AdvancedCircleGizmo : MonoBehaviour
{
    [Header("Circle")]
    [Min(0.0001f)] public float radius = 5f;
    [Min(3)] public int segments = 64;
    public Color color = Color.green;

    [Header("Arc Settings")]
    public bool arcMode = false;
    [Tooltip("Degrees from +Z axis.")]
    public float arcStart = 0f;
    public float arcEnd = 360f;
    public bool clockwise = true;

    [Header("Ticks")]
    public bool enableTicks = false;
    public bool ticksOnly = false;
    [Min(1)] public int tickCount = 8;
    public float tickSize = 0.5f;

    [Header("Rendering")]
    public bool drawOnTop = false;

    [Header("Debug Output")]
    public Vector3[] tickLocalPositions;

    private void OnDrawGizmos()
    {
        if (radius <= 0f) return;

#if UNITY_EDITOR
        if (drawOnTop)
        {
            Handles.zTest = CompareFunction.Always;
            Handles.color = color;
            Handles.matrix = transform.localToWorldMatrix;

            DrawAll(true);

            Handles.zTest = CompareFunction.LessEqual;
            return;
        }
#endif

        Matrix4x4 oldMatrix = Gizmos.matrix;
        Gizmos.matrix = transform.localToWorldMatrix;
        Gizmos.color = color;

        DrawAll(false);

        Gizmos.matrix = oldMatrix;
    }

    private void DrawAll(bool useHandles)
    {
        float startRad;
        float arcRange;

        if (!arcMode)
        {
            startRad = 0f;
            arcRange = clockwise ? Mathf.PI * 2f : -Mathf.PI * 2f;
        }
        else
        {
            float startDeg = Mathf.Repeat(arcStart, 360f);
            float endDeg = Mathf.Repeat(arcEnd, 360f);

            float directionSign = clockwise ? 1f : -1f;

            startRad = Mathf.Deg2Rad * startDeg * directionSign;

            float spanDeg = Mathf.Repeat(endDeg - startDeg, 360f);
            arcRange = Mathf.Deg2Rad * spanDeg * directionSign;
        }

        if (!ticksOnly)
            DrawArc(startRad, arcRange, useHandles);

        if (enableTicks)
            DrawTicks(startRad, arcRange, useHandles);
    }

    // ---------------- ARC ----------------

    private void DrawArc(float startRad, float arcRange, bool useHandles)
    {
        int stepCount = Mathf.Max(1, segments);
        float step = arcRange / stepCount;

        Vector3 prevPoint = GetPoint(startRad);

        for (int i = 1; i <= stepCount; i++)
        {
            float angle = startRad + step * i;
            Vector3 newPoint = GetPoint(angle);
            DrawLine(prevPoint, newPoint, useHandles);
            prevPoint = newPoint;
        }
    }

    // ---------------- TICKS ----------------

    private void DrawTicks(float startRad, float arcRange, bool useHandles)
    {
        tickLocalPositions = new Vector3[tickCount];
        if (tickCount <= 0) return;

        float step;

        if (arcMode)
            step = tickCount > 1 ? arcRange / (tickCount - 1) : 0f;
        else
            step = arcRange / tickCount;

        for (int i = 0; i < tickCount; i++)
        {
            float angle = startRad + step * i;
            Vector3 pos = GetPoint(angle);
            tickLocalPositions[i] = pos;

            if (ticksOnly)
                DrawRotatedCross(pos, angle, useHandles);
            else
                DrawCenteredTick(pos, angle, useHandles);
        }
    }

    // ---------------- HELPERS ----------------

    private Vector3 GetPoint(float angleRad)
    {
        return new Vector3(
            Mathf.Sin(angleRad) * radius,
            0f,
            Mathf.Cos(angleRad) * radius
        );
    }

    private void DrawCenteredTick(Vector3 center, float angleRad, bool useHandles)
    {
        Vector3 radial = new Vector3(
            Mathf.Sin(angleRad),
            0f,
            Mathf.Cos(angleRad)
        ).normalized;

        float half = tickSize * 0.5f;

        Vector3 a = center - radial * half;
        Vector3 b = center + radial * half;

        DrawLine(a, b, useHandles);
    }

    private void DrawRotatedCross(Vector3 center, float angleRad, bool useHandles)
    {
        float half = tickSize * 0.5f;

        Vector3 radial = new Vector3(
            Mathf.Sin(angleRad),
            0f,
            Mathf.Cos(angleRad)
        ).normalized;

        Vector3 tangent = new Vector3(
            radial.z,
            0f,
            -radial.x
        );

        DrawLine(center - radial * half, center + radial * half, useHandles);
        DrawLine(center - tangent * half, center + tangent * half, useHandles);
    }

    private void DrawLine(Vector3 a, Vector3 b, bool useHandles)
    {
#if UNITY_EDITOR
        if (useHandles)
            Handles.DrawLine(a, b);
        else
#endif
            Gizmos.DrawLine(a, b);
    }
}
