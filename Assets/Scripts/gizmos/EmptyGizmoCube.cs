using UnityEngine;

public class EmptyGizmoCube : MonoBehaviour
{
    public Color gizmoColor = Color.white;
    public float size = 0.1f;

    private void OnDrawGizmos()
    {
        Gizmos.color = gizmoColor;
        Gizmos.DrawWireCube(transform.position, Vector3.one * size);
    }
}
