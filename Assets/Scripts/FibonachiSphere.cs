using UnityEngine;

public static class FibonachiSphere
{

    public static Vector3[] GetRayDirections(Vector3 forwardDir, int rayCount, float maxAngle = 180f)
    {
        if (rayCount <= 0) return new Vector3[0];
        Vector3[] directions = new Vector3[rayCount];
        float phi = (1f + Mathf.Sqrt(5f)) / 2f;
        float cosMaxAngle = Mathf.Cos(maxAngle * Mathf.Deg2Rad);
        for (int i = 0; i < rayCount; i++)
        {
            float t = (rayCount == 1) ? 0f : 1f - (2f * i) / (rayCount - 1);
            float z = Mathf.Lerp(cosMaxAngle, 1f, (t + 1f) * 0.5f);
            float radius = Mathf.Sqrt(Mathf.Max(0f, 1f - z * z));
            float theta = 2f * Mathf.PI * i * phi;
            float x = Mathf.Cos(theta) * radius;
            float y = Mathf.Sin(theta) * radius;
            Vector3 direction = new Vector3(x, y, z).normalized;
            if (forwardDir != Vector3.forward)
            {
                Quaternion rotation = Quaternion.FromToRotation(Vector3.forward, forwardDir);
                direction = rotation * direction;
            }
            directions[i] = direction;
        }
        return directions;
    }
    public static Vector3 AvgNormalInSphere(Vector3 center, Vector3 poleDir,  LayerMask layerMask, int rayCount = 64 ,float rayLength = 2f,float maxAngle = 180f, bool debug = false)
    {
        Vector3[] rayDirections = GetRayDirections(poleDir, rayCount, maxAngle);
        Vector3 avgNormal = Vector3.zero;
        int hitCount = 0;
        foreach (var dir in rayDirections)
        {
            RaycastHit hit;
            if (Physics.Raycast(center, dir, out hit, rayLength, layerMask))
            {
                avgNormal += hit.normal * (1 - (hit.distance / rayLength));
                hitCount++;
                if (debug) Debug.DrawLine(hit.point, hit.point + hit.normal * 0.1f, Color.yellow);
            }
        }
        if (hitCount > 0)
        {
            if (debug) Debug.DrawRay(center, avgNormal.normalized * 0.2f, Color.blue);
            return avgNormal.normalized;
        }
        return Vector3.zero;
    }
}
