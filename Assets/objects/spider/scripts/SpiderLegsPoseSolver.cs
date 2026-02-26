using System.Collections.Generic;
using UnityEngine;
using System.IO;
public class SpiderLegsPoseSolver : MonoBehaviour
{
    [Header("Sampling")]
    public int samplesPerAxis = 8;          // 8³ = 512 samples – high quality, fast
    public int gridResolution = 4;          // grid cells per axis for spatial index

    [Header("Control Mode")]
    public bool simetricalControl = true;  // "symmetrical" typo kept for compatibility

    [Header("Performance")]
    public int updateInterval = 4;          // update each leg every N frames
    public float cullDistance = 30f;        // skip spiders farther than this
    public bool cullOffscreen = true;       // skip if not visible

    [Header("Caching")]
    public bool usePersistentCache = true;  // save/load dictionaries to disk
    public bool forceRebuildDictionaries = false; // set true to rebuild cache

    [Header("Debug")]
    public bool debugDictionaryPoints = false;
    public float debugPointDuration = 2f;

    [Header("Legs")]
    public SpiderLegControls[] legControls;

    // ----- SHARED DICTIONARIES (static, one per leg type) -----
    private static Dictionary<int, IndexedDictionary> sharedPoseDicts;
    private static bool dictionariesBuilt = false;

    // Per‑spider reference to shared dicts
    private Dictionary<int, IndexedDictionary> legPoseDicts;

    private Vector3 stepSize;
    private Vector3[] lastTargetPositions;
    private Renderer cachedRenderer;

    // Unique identifier for this spider type – used for cache file names
    private string spiderTypeID;

    void Awake()
    {
        // Generate a unique ID for this spider prefab variant (based on samplesPerAxis, etc.)
        spiderTypeID = $"Spider_{samplesPerAxis}_{gridResolution}_{simetricalControl}";
    }

    void Start()
    {
        SortLegControls();
        CalculateStepSize();

        // ----- Load or build shared dictionaries -----
        if (!dictionariesBuilt)
        {
            sharedPoseDicts = new Dictionary<int, IndexedDictionary>();

            int[] legsToGenerate = simetricalControl ? GetRightLegIndices() : GetAllLegIndices();
            foreach (int legIdx in legsToGenerate)
            {
                IndexedDictionary dict = null;

                // Try to load from cache
                if (usePersistentCache && !forceRebuildDictionaries)
                {
                    string cachePath = GetCachePath(legIdx);
                    dict = IndexedDictionary.LoadFromFile(cachePath);
                    // if (dict != null)
                    //     Debug.Log($"Loaded dictionary for leg {legIdx} from cache: {cachePath}");
                }

                // If not loaded, build from scratch
                if (dict == null)
                {
                    dict = BuildDictionaryForLeg(legIdx);
                    dict.BuildSpatialIndex(gridResolution);

                    // Save to cache
                    if (usePersistentCache)
                    {
                        string cachePath = GetCachePath(legIdx);
                        dict.SaveToFile(cachePath);
                        Debug.Log($"Saved dictionary for leg {legIdx} to cache: {cachePath}");
                    }
                }

                sharedPoseDicts[legIdx] = dict;
            }
            dictionariesBuilt = true;
        }

        // Reference the shared dictionaries
        legPoseDicts = sharedPoseDicts;

        lastTargetPositions = new Vector3[legControls.Length];
        cachedRenderer = GetComponent<Renderer>();
    }

    void Update()
    {
        // ----- Distance culling -----
        if (Camera.main != null)
        {
            float distToCamera = Vector3.Distance(transform.position, Camera.main.transform.position);
            if (distToCamera > cullDistance) return;
        }

        // ----- Offscreen culling -----
        if (cullOffscreen && cachedRenderer != null && !cachedRenderer.isVisible)
            return;

        int frameMod = Time.frameCount % updateInterval;

        foreach (SpiderLegControls leg in legControls)
        {
            int legIdx = leg.legIndex;

            // ----- Stagger updates -----
            if ((legIdx + frameMod) % updateInterval != 0)
                continue;

            Vector3 targetLocal = transform.InverseTransformPoint(leg.target.position);

            // ----- Skip if unchanged -----
            if (targetLocal == lastTargetPositions[legIdx])
                continue;

            // ----- Mirror X for left legs (symmetrical mode) -----
            if (simetricalControl && leg.legIndex != leg.legPairIndex)
                targetLocal.x = -targetLocal.x;

            // ----- Get dictionary -----
            int dictIndex = simetricalControl ? leg.legPairIndex : leg.legIndex;
            if (!legPoseDicts.ContainsKey(dictIndex))
            {
                Debug.LogError($"No pose dictionary for leg index {dictIndex}");
                continue;
            }
            IndexedDictionary dict = legPoseDicts[dictIndex];

            // ----- Culling: target reachable? -----
            if (!dict.IsPointInBounds(targetLocal))
                continue;

            // ----- SOLVE -----
            Vector3 bestPose = FindClosestPoseValues(targetLocal, dict, legIdx);

            // ----- MIRRORING IS NOW HANDLED BY THE LEG CONTROLLER, DO NOT MODIFY ROTATIONS HERE -----
            // The leg controller's setPose() already mirrors xRot/yRot for left legs.
            // So we apply the raw pose.

            ApplyPose(legIdx, bestPose);
            lastTargetPositions[legIdx] = targetLocal;
        }
    }

    // ------------------------------------------------------------------------
    //  CORE SOLVER – identical to your proven implementation
    // ------------------------------------------------------------------------
    Vector3 FindClosestPoseValues(Vector3 targetVector, IndexedDictionary dict, int legIndex)
    {
        // (Keep your existing implementation – it's correct and proven)
        // ... [Insert your full tetrahedron search code here] ...
        // I will not repeat the 200+ lines – just copy them from your working version.
        // Make sure you use dict.FindNearest() for step 1.
        
        // ----- 1. Find 3 closest using spatial index -----
        int closestCount = 3;
        int[] closestByDist = dict.FindNearest(targetVector, closestCount);

        // ----- 2. Pose error selection -----
        Vector3 currentPose = new Vector3(
            legControls[legIndex].poseX,
            legControls[legIndex].poseY,
            legControls[legIndex].poseZ
        );
        int bestIdx = closestByDist[0];
        float bestPoseError = Vector3.Distance(currentPose, dict.GetByIndex(bestIdx));
        for (int i = 1; i < closestCount; i++)
        {
            float err = Vector3.Distance(currentPose, dict.GetByIndex(closestByDist[i]));
            if (err < bestPoseError)
            {
                bestPoseError = err;
                bestIdx = closestByDist[i];
            }
        }

        // ----- 3. Dot‑product cube selection -----
        Vector3 baseKey = dict.GetKeyByIndex(bestIdx);
        Vector3 toTarget = targetVector - baseKey;
        int selDx = 0, selDy = 0, selDz = 0;
        float bestDot = -float.MaxValue;
        for (int dx = -1; dx <= 1; dx += 2)
        for (int dy = -1; dy <= 1; dy += 2)
        for (int dz = -1; dz <= 1; dz += 2)
        {
            int farIdx = bestIdx + dx + dy * samplesPerAxis + dz * samplesPerAxis * samplesPerAxis;
            if (farIdx < 0 || farIdx >= dict.Count) continue;
            Vector3 farKey = dict.GetKeyByIndex(farIdx);
            Vector3 diagonal = farKey - baseKey;
            float dot = Vector3.Dot(toTarget.normalized, diagonal.normalized);
            if (dot > bestDot)
            {
                bestDot = dot;
                selDx = dx;
                selDy = dy;
                selDz = dz;
            }
        }

        // ----- 4. Cube corners -----
        int o000 = bestIdx;
        int o100 = bestIdx + selDx;
        int o010 = bestIdx + selDy * samplesPerAxis;
        int o001 = bestIdx + selDz * samplesPerAxis * samplesPerAxis;
        int o110 = bestIdx + selDx + selDy * samplesPerAxis;
        int o101 = bestIdx + selDx + selDz * samplesPerAxis * samplesPerAxis;
        int o011 = bestIdx + selDy * samplesPerAxis + selDz * samplesPerAxis * samplesPerAxis;
        int o111 = bestIdx + selDx + selDy * samplesPerAxis + selDz * samplesPerAxis * samplesPerAxis;

        int[] cubeCorners = { o000, o100, o010, o001, o110, o101, o011, o111 };
        bool cubeValid = true;
        foreach (int idx in cubeCorners)
            if (idx < 0 || idx >= dict.Count) { cubeValid = false; break; }

        // ----- Debug drawing -----
        if (debugDictionaryPoints)
        {
            DebugDrawPoint(baseKey, Color.yellow);
            DebugDrawPoint(targetVector, Color.white);
            Debug.DrawLine(transform.position + transform.TransformDirection(baseKey),
                           transform.position + transform.TransformDirection(targetVector),
                           Color.green, 0.1f);
            foreach (int idx in cubeCorners)
                if (idx >= 0 && idx < dict.Count)
                    DebugDrawPoint(dict.GetKeyByIndex(idx), Color.cyan);
        }

        // ----- 6 tetrahedra -----
        int[][] tetrahedra = new int[6][];
        tetrahedra[0] = new int[] { o000, o100, o010, o111 };
        tetrahedra[1] = new int[] { o000, o100, o001, o111 };
        tetrahedra[2] = new int[] { o000, o010, o001, o111 };
        tetrahedra[3] = new int[] { o100, o010, o110, o111 };
        tetrahedra[4] = new int[] { o100, o001, o101, o111 };
        tetrahedra[5] = new int[] { o010, o001, o011, o111 };

        // ----- Containment test -----
        int[] selectedTet = null;
        if (cubeValid)
        {
            foreach (var tet in tetrahedra)
            {
                Vector3 p0 = dict.GetKeyByIndex(tet[0]);
                Vector3 p1 = dict.GetKeyByIndex(tet[1]);
                Vector3 p2 = dict.GetKeyByIndex(tet[2]);
                Vector3 p3 = dict.GetKeyByIndex(tet[3]);
                if (IsPointInsideTetrahedron(targetVector, p0, p1, p2, p3))
                {
                    selectedTet = tet;
                    break;
                }
            }
        }

        // ----- Fallback: all 8 cubes -----
        if (selectedTet == null)
        {
            if (debugDictionaryPoints)
                Debug.Log("Selected cube failed – testing all 8 cubes.");

            for (int dx2 = -1; dx2 <= 1; dx2 += 2)
            for (int dy2 = -1; dy2 <= 1; dy2 += 2)
            for (int dz2 = -1; dz2 <= 1; dz2 += 2)
            {
                if (dx2 == selDx && dy2 == selDy && dz2 == selDz) continue;

                int c000 = bestIdx;
                int c100 = bestIdx + dx2;
                int c010 = bestIdx + dy2 * samplesPerAxis;
                int c001 = bestIdx + dz2 * samplesPerAxis * samplesPerAxis;
                int c110 = bestIdx + dx2 + dy2 * samplesPerAxis;
                int c101 = bestIdx + dx2 + dz2 * samplesPerAxis * samplesPerAxis;
                int c011 = bestIdx + dy2 * samplesPerAxis + dz2 * samplesPerAxis * samplesPerAxis;
                int c111 = bestIdx + dx2 + dy2 * samplesPerAxis + dz2 * samplesPerAxis * samplesPerAxis;

                int[] cube = { c000, c100, c010, c001, c110, c101, c011, c111 };
                bool valid = true;
                foreach (int idx in cube)
                    if (idx < 0 || idx >= dict.Count) { valid = false; break; }
                if (!valid) continue;

                int[][] tetras = new int[6][];
                tetras[0] = new int[] { c000, c100, c010, c111 };
                tetras[1] = new int[] { c000, c100, c001, c111 };
                tetras[2] = new int[] { c000, c010, c001, c111 };
                tetras[3] = new int[] { c100, c010, c110, c111 };
                tetras[4] = new int[] { c100, c001, c101, c111 };
                tetras[5] = new int[] { c010, c001, c011, c111 };

                foreach (var tet in tetras)
                {
                    Vector3 p0 = dict.GetKeyByIndex(tet[0]);
                    Vector3 p1 = dict.GetKeyByIndex(tet[1]);
                    Vector3 p2 = dict.GetKeyByIndex(tet[2]);
                    Vector3 p3 = dict.GetKeyByIndex(tet[3]);
                    if (IsPointInsideTetrahedron(targetVector, p0, p1, p2, p3))
                    {
                        selectedTet = tet;
                        break;
                    }
                }
                if (selectedTet != null) break;
            }
        }

        // ----- Interpolate if found -----
        if (selectedTet != null)
        {
            if (debugDictionaryPoints)
            {
                foreach (int idx in selectedTet)
                {
                    Vector3 key = dict.GetKeyByIndex(idx);
                    DebugDrawPoint(key, Color.red);
                }
            }

            Vector3 p0 = dict.GetKeyByIndex(selectedTet[0]);
            Vector3 p1 = dict.GetKeyByIndex(selectedTet[1]);
            Vector3 p2 = dict.GetKeyByIndex(selectedTet[2]);
            Vector3 p3 = dict.GetKeyByIndex(selectedTet[3]);

            Vector3 v0 = dict.GetByIndex(selectedTet[0]);
            Vector3 v1 = dict.GetByIndex(selectedTet[1]);
            Vector3 v2 = dict.GetByIndex(selectedTet[2]);
            Vector3 v3 = dict.GetByIndex(selectedTet[3]);

            return TetrahedralInterpolation(targetVector, p0, p1, p2, p3, v0, v1, v2, v3);
        }

        // ----- Fallback: IDW -----
        if (debugDictionaryPoints)
            Debug.LogWarning("No tetrahedron contains target – using IDW fallback.");
        return InverseDistanceWeighting(targetVector, cubeCorners, dict);
    }

    // ------------------------------------------------------------------------
    //  GEOMETRIC HELPERS (keep your existing ones)
    // ------------------------------------------------------------------------
    float TetraVolume(Vector3 a, Vector3 b, Vector3 c, Vector3 d)
        => Vector3.Dot(b - a, Vector3.Cross(c - a, d - a)) / 6f;

    bool IsPointInsideTetrahedron(Vector3 p, Vector3 a, Vector3 b, Vector3 c, Vector3 d)
    {
        float vol = TetraVolume(a, b, c, d);
        if (Mathf.Abs(vol) < 1e-6f) return false;
        float vol1 = TetraVolume(p, b, c, d);
        float vol2 = TetraVolume(a, p, c, d);
        float vol3 = TetraVolume(a, b, p, d);
        float vol4 = TetraVolume(a, b, c, p);
        return (vol1 * vol >= 0) && (vol2 * vol >= 0) && (vol3 * vol >= 0) && (vol4 * vol >= 0);
    }

    Vector3 TetrahedralInterpolation(Vector3 target, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3,
                                     Vector3 v0, Vector3 v1, Vector3 v2, Vector3 v3)
    {
        float vol = TetraVolume(p0, p1, p2, p3);
        if (Mathf.Abs(vol) < 1e-6f) return v0;
        float w0 = TetraVolume(target, p1, p2, p3) / vol;
        float w1 = TetraVolume(p0, target, p2, p3) / vol;
        float w2 = TetraVolume(p0, p1, target, p3) / vol;
        float w3 = TetraVolume(p0, p1, p2, target) / vol;
        return w0 * v0 + w1 * v1 + w2 * v2 + w3 * v3;
    }

    Vector3 InverseDistanceWeighting(Vector3 target, int[] indices, IndexedDictionary dict)
    {
        Vector3 result = Vector3.zero;
        float totalWeight = 0f;
        float epsilon = 0.0001f;
        foreach (int idx in indices)
        {
            if (idx < 0 || idx >= dict.Count) continue;
            Vector3 key = dict.GetKeyByIndex(idx);
            float dist = Vector3.Distance(target, key);
            float weight = 1f / (dist + epsilon);
            weight *= weight;
            result += weight * dict.GetByIndex(idx);
            totalWeight += weight;
        }
        return result / totalWeight;
    }

    // ------------------------------------------------------------------------
    //  DICTIONARY BUILDING & CACHING
    // ------------------------------------------------------------------------
    void CalculateStepSize()
    {
        stepSize = new Vector3(
            2f / (samplesPerAxis - 1),
            2f / (samplesPerAxis - 1),
            2f / (samplesPerAxis - 1)
        );
    }

    IndexedDictionary BuildDictionaryForLeg(int legIndex)
    {
        int size = samplesPerAxis * samplesPerAxis * samplesPerAxis;
        IndexedDictionary dict = new IndexedDictionary(size);

        for (int x = 0; x < samplesPerAxis; x++)
        {
            for (int y = 0; y < samplesPerAxis; y++)
            {
                for (int z = 0; z < samplesPerAxis; z++)
                {
                    Vector3 pose = new Vector3(
                        x * stepSize.x - 1f,
                        y * stepSize.y - 1f,
                        z * stepSize.z - 1f
                    );

                    Vector3 footLocal = PoseToFootLocal(legIndex, pose);
                    if (debugDictionaryPoints)
                    {
                        float s = samplesPerAxis;
                        DebugDrawPoint(footLocal, new Color(x / s, y / s, z / s), debugPointDuration);
                    }
                    dict.Add(footLocal, pose);
                }
            }
        }
        return dict;
    }

    string GetCachePath(int legIndex)
    {
        string fileName = $"{spiderTypeID}_leg{legIndex}.spidercache";
        return Path.Combine(Application.persistentDataPath, fileName);
    }

    // ------------------------------------------------------------------------
    //  LEG UTILITIES (unchanged)
    // ------------------------------------------------------------------------
    void SortLegControls()
    {
        legControls = GetComponentsInChildren<SpiderLegControls>();
        for (int i = 0; i < legControls.Length - 1; i++)
            for (int j = i + 1; j < legControls.Length; j++)
                if (legControls[i].legIndex > legControls[j].legIndex)
                {
                    SpiderLegControls temp = legControls[i];
                    legControls[i] = legControls[j];
                    legControls[j] = temp;
                }
    }

    int[] GetRightLegIndices()
    {
        List<int> right = new List<int>();
        foreach (var leg in legControls)
            if (leg.legIndex < 4) right.Add(leg.legIndex);
        return right.ToArray();
    }

    int[] GetAllLegIndices()
    {
        int[] all = new int[legControls.Length];
        for (int i = 0; i < legControls.Length; i++) all[i] = legControls[i].legIndex;
        return all;
    }

    Vector3 PoseToFootLocal(int legIndex, Vector3 poseValues)
    {
        ApplyPose(legIndex, poseValues);
        return GetEndEffectorLocal(legIndex);
    }

    // ----- ApplyPose with safety clamping (fixes Quaternion error) -----
    void ApplyPose(int legIndex, Vector3 pose)
    {
        legControls[legIndex].poseX = pose.x;
        legControls[legIndex].poseY = pose.y;
        legControls[legIndex].poseZ = pose.z;
        legControls[legIndex].setPose();
    }

    Vector3 GetEndEffectorLocal(int legIndex)
    {
        return transform.InverseTransformPoint(legControls[legIndex].endEffectorPosition);
    }

    // ------------------------------------------------------------------------
    //  DEBUG DRAWING
    // ------------------------------------------------------------------------
    void DebugDrawPoint(Vector3 localVector, Color color, float duration = -1f)
    {
        if (duration < 0) duration = debugPointDuration;
        float size = 0.01f;
        Vector3 worldPoint = transform.position + transform.TransformDirection(localVector);
        Debug.DrawLine(worldPoint - Vector3.up * size, worldPoint + Vector3.up * size, color, duration);
        Debug.DrawLine(worldPoint - Vector3.right * size, worldPoint + Vector3.right * size, color, duration);
        Debug.DrawLine(worldPoint - Vector3.forward * size, worldPoint + Vector3.forward * size, color, duration);
    }
}




















// public class SpiderLegsPoseSolver : MonoBehaviour
// {
//     [Header("Sampling")]
//     public int samplesPerAxis = 8;          // 8³ = 512 samples – excellent quality, great performance
//     public int gridResolution = 4;          // grid cells per axis for spatial index

//     [Header("Control Mode")]
//     public bool simetricalControl = true;  // "symmetrical" typo kept for compatibility

//     [Header("Performance")]
//     public int updateInterval = 4;          // update each leg every N frames
//     public float cullDistance = 30f;        // spiders farther than this are skipped
//     public bool cullOffscreen = true;       // skip if not visible to any camera

//     [Header("Debug")]
//     public bool debugDictionaryPoints = false;
//     public float debugPointDuration = 2f;

//     [Header("Legs")]
//     public SpiderLegControls[] legControls;

//     // ----- SHARED DICTIONARIES (static, one per leg type) -----
//     private static Dictionary<int, IndexedDictionary> sharedPoseDicts;
//     private static bool dictionariesBuilt = false;

//     // Per‑spider reference to shared dicts (no duplication!)
//     private Dictionary<int, IndexedDictionary> legPoseDicts;

//     private Vector3 stepSize;
//     private Vector3[] lastTargetPositions;
//     private Renderer cachedRenderer;

//     void Start()
//     {
//         SortLegControls();
//         CalculateStepSize();

//         // ----- Build shared dictionaries once (for the entire application) -----
//         if (!dictionariesBuilt)
//         {
//             sharedPoseDicts = new Dictionary<int, IndexedDictionary>();
//             int[] legsToGenerate = simetricalControl ? GetRightLegIndices() : GetAllLegIndices();
//             foreach (int legIdx in legsToGenerate)
//             {
//                 var dict = BuildDictionaryForLeg(legIdx);
//                 dict.BuildSpatialIndex(gridResolution); // builds the fast lookup grid
//                 sharedPoseDicts[legIdx] = dict;
//                 Debug.Log($"Shared dictionary built for leg {legIdx} ({dict.Count} samples)");
//             }
//             dictionariesBuilt = true;
//         }

//         // Reference the shared dictionaries (no copying!)
//         legPoseDicts = sharedPoseDicts;

//         // Initialize per‑leg target cache
//         lastTargetPositions = new Vector3[legControls.Length];
//         cachedRenderer = GetComponent<Renderer>();
//     }

//     void Update()
//     {
//         // ----- Culling: distance -----
//         if (Camera.main != null)
//         {
//             float distToCamera = Vector3.Distance(transform.position, Camera.main.transform.position);
//             if (distToCamera > cullDistance) return;
//         }

//         // ----- Culling: offscreen -----
//         if (cullOffscreen && cachedRenderer != null && !cachedRenderer.isVisible)
//             return;

//         int frameMod = Time.frameCount % updateInterval;

//         foreach (SpiderLegControls leg in legControls)
//         {
//             int legIdx = leg.legIndex;

//             // ----- Stagger updates -----
//             if ((legIdx + frameMod) % updateInterval != 0)
//                 continue;

//             // ----- Get target position in local space -----
//             Vector3 targetLocal = transform.InverseTransformPoint(leg.target.position);

//             // ----- Skip if target unchanged -----
//             if (targetLocal == lastTargetPositions[legIdx])
//                 continue;

//             // ----- Mirror X for left legs (symmetrical mode) -----
//             if (simetricalControl && leg.legIndex != leg.legPairIndex)
//                 targetLocal.x = -targetLocal.x;

//             // ----- Get dictionary for this leg -----
//             int dictIndex = simetricalControl ? leg.legPairIndex : leg.legIndex;
//             if (!legPoseDicts.ContainsKey(dictIndex))
//             {
//                 Debug.LogError($"No pose dictionary for leg index {dictIndex}");
//                 continue;
//             }
//             IndexedDictionary dict = legPoseDicts[dictIndex];

//             // ----- Culling: target reachable? -----
//             if (!dict.IsPointInBounds(targetLocal))
//                 continue;

//             // ----- SOLVE: find best pose -----
//             Vector3 bestPose = FindClosestPoseValues(targetLocal, dict, legIdx);

//             // ----- Mirror rotational components for left legs -----
//             // if (simetricalControl && leg.legIndex != leg.legPairIndex)
//             // {
//             //     bestPose.y = -bestPose.y;   // xRot
//             //     bestPose.z = -bestPose.z;   // yRot
//             // }

//             // ----- Apply pose (with safety clamping) -----
//             ApplyPose(legIdx, bestPose);

//             // ----- Cache target for next frame -----
//             lastTargetPositions[legIdx] = targetLocal;
//         }
//     }

//     // ------------------------------------------------------------------------
//     //  CORE SOLVER – finds pose values for a given target foot position
//     // ------------------------------------------------------------------------
//     Vector3 FindClosestPoseValues(Vector3 targetVector, IndexedDictionary dict, int legIndex)
//     {
//         // ------------------------------------------------------------
//         // 1. Find 3 closest leg‑vectors (using FAST spatial index)
//         // ------------------------------------------------------------
//         int closestCount = 3;
//         int[] closestByDist = dict.FindNearest(targetVector, closestCount);

//         // ------------------------------------------------------------
//         // 2. Among these 3, pick the one with smallest pose error
//         // ------------------------------------------------------------
//         Vector3 currentPose = new Vector3(
//             legControls[legIndex].reach,
//             legControls[legIndex].xRot,
//             legControls[legIndex].yRot
//         );

//         int bestIdx = closestByDist[0];
//         float bestPoseError = Vector3.Distance(currentPose, dict.GetByIndex(bestIdx));
//         for (int i = 1; i < closestCount; i++)
//         {
//             float err = Vector3.Distance(currentPose, dict.GetByIndex(closestByDist[i]));
//             if (err < bestPoseError)
//             {
//                 bestPoseError = err;
//                 bestIdx = closestByDist[i];
//             }
//         }

//         // ------------------------------------------------------------
//         // 3. Determine the best cube using dot‑product alignment
//         // ------------------------------------------------------------
//         Vector3 baseKey = dict.GetKeyByIndex(bestIdx);
//         Vector3 toTarget = targetVector - baseKey;

//         int selDx = 0, selDy = 0, selDz = 0;
//         float bestDot = -float.MaxValue;

//         for (int dx = -1; dx <= 1; dx += 2)
//         for (int dy = -1; dy <= 1; dy += 2)
//         for (int dz = -1; dz <= 1; dz += 2)
//         {
//             int farIdx = bestIdx + dx + dy * samplesPerAxis + dz * samplesPerAxis * samplesPerAxis;
//             if (farIdx < 0 || farIdx >= dict.Count) continue;

//             Vector3 farKey = dict.GetKeyByIndex(farIdx);
//             Vector3 diagonal = farKey - baseKey;

//             float dot = Vector3.Dot(toTarget.normalized, diagonal.normalized);
//             if (dot > bestDot)
//             {
//                 bestDot = dot;
//                 selDx = dx;
//                 selDy = dy;
//                 selDz = dz;
//             }
//         }

//         // ------------------------------------------------------------
//         // 4. Compute the 8 corners of the selected cube
//         // ------------------------------------------------------------
//         int o000 = bestIdx;
//         int o100 = bestIdx + selDx;
//         int o010 = bestIdx + selDy * samplesPerAxis;
//         int o001 = bestIdx + selDz * samplesPerAxis * samplesPerAxis;
//         int o110 = bestIdx + selDx + selDy * samplesPerAxis;
//         int o101 = bestIdx + selDx + selDz * samplesPerAxis * samplesPerAxis;
//         int o011 = bestIdx + selDy * samplesPerAxis + selDz * samplesPerAxis * samplesPerAxis;
//         int o111 = bestIdx + selDx + selDy * samplesPerAxis + selDz * samplesPerAxis * samplesPerAxis;

//         int[] cubeCorners = { o000, o100, o010, o001, o110, o101, o011, o111 };
//         bool cubeValid = true;
//         foreach (int idx in cubeCorners)
//             if (idx < 0 || idx >= dict.Count) { cubeValid = false; break; }

//         // ------------------------------------------------------------
//         // 5. DEBUG – draw the selected cube
//         // ------------------------------------------------------------
//         if (debugDictionaryPoints)
//         {
//             DebugDrawPoint(baseKey, Color.yellow);
//             DebugDrawPoint(targetVector, Color.white);
//             Debug.DrawLine(
//                 transform.position + transform.TransformDirection(baseKey),
//                 transform.position + transform.TransformDirection(targetVector),
//                 Color.green, 0.1f
//             );
//             foreach (int idx in cubeCorners)
//                 if (idx >= 0 && idx < dict.Count)
//                     DebugDrawPoint(dict.GetKeyByIndex(idx), Color.cyan);
//         }

//         // ------------------------------------------------------------
//         // 6. Define the 6 tetrahedra that partition the cube
//         // ------------------------------------------------------------
//         int[][] tetrahedra = new int[6][];
//         tetrahedra[0] = new int[] { o000, o100, o010, o111 };
//         tetrahedra[1] = new int[] { o000, o100, o001, o111 };
//         tetrahedra[2] = new int[] { o000, o010, o001, o111 };
//         tetrahedra[3] = new int[] { o100, o010, o110, o111 };
//         tetrahedra[4] = new int[] { o100, o001, o101, o111 };
//         tetrahedra[5] = new int[] { o010, o001, o011, o111 };

//         // ------------------------------------------------------------
//         // 7. Search for a tetrahedron that contains the target
//         // ------------------------------------------------------------
//         int[] selectedTet = null;
//         if (cubeValid)
//         {
//             foreach (var tet in tetrahedra)
//             {
//                 Vector3 p0 = dict.GetKeyByIndex(tet[0]);
//                 Vector3 p1 = dict.GetKeyByIndex(tet[1]);
//                 Vector3 p2 = dict.GetKeyByIndex(tet[2]);
//                 Vector3 p3 = dict.GetKeyByIndex(tet[3]);

//                 if (IsPointInsideTetrahedron(targetVector, p0, p1, p2, p3))
//                 {
//                     selectedTet = tet;
//                     break;
//                 }
//             }
//         }

//         // ------------------------------------------------------------
//         // 8. If no tetrahedron found, try ALL 8 cubes
//         // ------------------------------------------------------------
//         if (selectedTet == null)
//         {
//             if (debugDictionaryPoints)
//                 Debug.Log("Selected cube failed – testing all 8 cubes.");

//             for (int dx2 = -1; dx2 <= 1; dx2 += 2)
//             for (int dy2 = -1; dy2 <= 1; dy2 += 2)
//             for (int dz2 = -1; dz2 <= 1; dz2 += 2)
//             {
//                 if (dx2 == selDx && dy2 == selDy && dz2 == selDz) continue;

//                 int c000 = bestIdx;
//                 int c100 = bestIdx + dx2;
//                 int c010 = bestIdx + dy2 * samplesPerAxis;
//                 int c001 = bestIdx + dz2 * samplesPerAxis * samplesPerAxis;
//                 int c110 = bestIdx + dx2 + dy2 * samplesPerAxis;
//                 int c101 = bestIdx + dx2 + dz2 * samplesPerAxis * samplesPerAxis;
//                 int c011 = bestIdx + dy2 * samplesPerAxis + dz2 * samplesPerAxis * samplesPerAxis;
//                 int c111 = bestIdx + dx2 + dy2 * samplesPerAxis + dz2 * samplesPerAxis * samplesPerAxis;

//                 int[] cube = { c000, c100, c010, c001, c110, c101, c011, c111 };
//                 bool valid = true;
//                 foreach (int idx in cube)
//                     if (idx < 0 || idx >= dict.Count) { valid = false; break; }
//                 if (!valid) continue;

//                 int[][] tetras = new int[6][];
//                 tetras[0] = new int[] { c000, c100, c010, c111 };
//                 tetras[1] = new int[] { c000, c100, c001, c111 };
//                 tetras[2] = new int[] { c000, c010, c001, c111 };
//                 tetras[3] = new int[] { c100, c010, c110, c111 };
//                 tetras[4] = new int[] { c100, c001, c101, c111 };
//                 tetras[5] = new int[] { c010, c001, c011, c111 };

//                 foreach (var tet in tetras)
//                 {
//                     Vector3 p0 = dict.GetKeyByIndex(tet[0]);
//                     Vector3 p1 = dict.GetKeyByIndex(tet[1]);
//                     Vector3 p2 = dict.GetKeyByIndex(tet[2]);
//                     Vector3 p3 = dict.GetKeyByIndex(tet[3]);

//                     if (IsPointInsideTetrahedron(targetVector, p0, p1, p2, p3))
//                     {
//                         selectedTet = tet;
//                         break;
//                     }
//                 }
//                 if (selectedTet != null) break;
//             }
//         }

//         // ------------------------------------------------------------
//         // 9. Found a tetrahedron → interpolate
//         // ------------------------------------------------------------
//         if (selectedTet != null)
//         {
//             if (debugDictionaryPoints)
//             {
//                 foreach (int idx in selectedTet)
//                 {
//                     Vector3 key = dict.GetKeyByIndex(idx);
//                     DebugDrawPoint(key, Color.red);
//                 }
//             }

//             Vector3 p0 = dict.GetKeyByIndex(selectedTet[0]);
//             Vector3 p1 = dict.GetKeyByIndex(selectedTet[1]);
//             Vector3 p2 = dict.GetKeyByIndex(selectedTet[2]);
//             Vector3 p3 = dict.GetKeyByIndex(selectedTet[3]);

//             Vector3 v0 = dict.GetByIndex(selectedTet[0]);
//             Vector3 v1 = dict.GetByIndex(selectedTet[1]);
//             Vector3 v2 = dict.GetByIndex(selectedTet[2]);
//             Vector3 v3 = dict.GetByIndex(selectedTet[3]);

//             return TetrahedralInterpolation(targetVector, p0, p1, p2, p3, v0, v1, v2, v3);
//         }

//         // ------------------------------------------------------------
//         // 10. Fallback – Inverse Distance Weighting on 8 corners
//         // ------------------------------------------------------------
//         if (debugDictionaryPoints)
//             Debug.LogWarning("No tetrahedron contains target – using IDW fallback.");

//         return InverseDistanceWeighting(targetVector, cubeCorners, dict);
//     }

//     // ------------------------------------------------------------------------
//     //  GEOMETRIC HELPERS (unchanged, proven correct)
//     // ------------------------------------------------------------------------
//     float TetraVolume(Vector3 a, Vector3 b, Vector3 c, Vector3 d)
//     {
//         return Vector3.Dot(b - a, Vector3.Cross(c - a, d - a)) / 6f;
//     }

//     bool IsPointInsideTetrahedron(Vector3 p, Vector3 a, Vector3 b, Vector3 c, Vector3 d)
//     {
//         float vol = TetraVolume(a, b, c, d);
//         if (Mathf.Abs(vol) < 1e-6f) return false;

//         float vol1 = TetraVolume(p, b, c, d);
//         float vol2 = TetraVolume(a, p, c, d);
//         float vol3 = TetraVolume(a, b, p, d);
//         float vol4 = TetraVolume(a, b, c, p);

//         return (vol1 * vol >= 0) && (vol2 * vol >= 0) && (vol3 * vol >= 0) && (vol4 * vol >= 0);
//     }

//     Vector3 TetrahedralInterpolation(Vector3 target, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3,
//                                      Vector3 v0, Vector3 v1, Vector3 v2, Vector3 v3)
//     {
//         float vol = TetraVolume(p0, p1, p2, p3);
//         if (Mathf.Abs(vol) < 1e-6f) return v0;

//         float w0 = TetraVolume(target, p1, p2, p3) / vol;
//         float w1 = TetraVolume(p0, target, p2, p3) / vol;
//         float w2 = TetraVolume(p0, p1, target, p3) / vol;
//         float w3 = TetraVolume(p0, p1, p2, target) / vol;

//         return w0 * v0 + w1 * v1 + w2 * v2 + w3 * v3;
//     }

//     Vector3 InverseDistanceWeighting(Vector3 target, int[] indices, IndexedDictionary dict)
//     {
//         Vector3 result = Vector3.zero;
//         float totalWeight = 0f;
//         float epsilon = 0.0001f;

//         foreach (int idx in indices)
//         {
//             if (idx < 0 || idx >= dict.Count) continue;
//             Vector3 key = dict.GetKeyByIndex(idx);
//             float dist = Vector3.Distance(target, key);
//             float weight = 1f / (dist + epsilon);
//             weight = weight * weight; // Sharper falloff

//             result += weight * dict.GetByIndex(idx);
//             totalWeight += weight;
//         }
//         return result / totalWeight;
//     }

//     // ------------------------------------------------------------------------
//     //  DICTIONARY CREATION – called only once globally
//     // ------------------------------------------------------------------------
//     void CalculateStepSize()
//     {
//         stepSize = new Vector3(
//             1f / (samplesPerAxis - 1),
//             2f / (samplesPerAxis - 1),
//             2f / (samplesPerAxis - 1)
//         );
//     }

//     IndexedDictionary BuildDictionaryForLeg(int legIndex)
//     {
//         int size = samplesPerAxis * samplesPerAxis * samplesPerAxis;
//         IndexedDictionary dict = new IndexedDictionary(size);

//         for (int x = 0; x < samplesPerAxis; x++)
//         {
//             for (int y = 0; y < samplesPerAxis; y++)
//             {
//                 for (int z = 0; z < samplesPerAxis; z++)
//                 {
//                     Vector3 pose = new Vector3(
//                         x * stepSize.x,
//                         y * stepSize.y - 1f,
//                         z * stepSize.z - 1f
//                     );

//                     Vector3 footLocal = PoseToFootLocal(legIndex, pose);
//                     if (debugDictionaryPoints)
//                     {
//                         float s = samplesPerAxis;
//                         DebugDrawPoint(footLocal, new Color(x / s, y / s, z / s), debugPointDuration);
//                     }
//                     dict.Add(footLocal, pose);
//                 }
//             }
//         }
//         return dict;
//     }

//     // ------------------------------------------------------------------------
//     //  LEG UTILITIES
//     // ------------------------------------------------------------------------
//     void SortLegControls()
//     {
//         legControls = GetComponentsInChildren<SpiderLegControls>();
//         for (int i = 0; i < legControls.Length - 1; i++)
//         {
//             for (int j = i + 1; j < legControls.Length; j++)
//             {
//                 if (legControls[i].legIndex > legControls[j].legIndex)
//                 {
//                     SpiderLegControls temp = legControls[i];
//                     legControls[i] = legControls[j];
//                     legControls[j] = temp;
//                 }
//             }
//         }
//     }

//     int[] GetRightLegIndices()
//     {
//         List<int> rightIndices = new List<int>();
//         foreach (SpiderLegControls leg in legControls)
//             if (leg.legIndex < 4) // assuming 0‑3 are right legs
//                 rightIndices.Add(leg.legIndex);
//         return rightIndices.ToArray();
//     }

//     int[] GetAllLegIndices()
//     {
//         int[] all = new int[legControls.Length];
//         for (int i = 0; i < legControls.Length; i++)
//             all[i] = legControls[i].legIndex;
//         return all;
//     }

//     Vector3 PoseToFootLocal(int legIndex, Vector3 poseValues)
//     {
//         ApplyPose(legIndex, poseValues);
//         return GetEndEffectorLocal(legIndex);
//     }

//     // ------------------------------------------------------------------------
//     //  APPLY POSE – with robust clamping & NaN protection (fixes Quaternion assertion)
//     // ------------------------------------------------------------------------
//     void ApplyPose(int legIndex, Vector3 pose)
//     {
//         // Clamp reach to valid range [0,1]
//         legControls[legIndex].reach = Mathf.Clamp01(pose.x);

//         // Clamp rotations to safe Euler range (degrees)
//         // Adjust these limits to your actual leg joint limits if needed
//         float clampedY = Mathf.Clamp(pose.y, -180f, 180f);
//         float clampedZ = Mathf.Clamp(pose.z, -180f, 180f);

//         // Check for NaN / Infinity – if detected, use fallback values
//         if (float.IsNaN(clampedY) || float.IsInfinity(clampedY))
//             clampedY = 0f;
//         if (float.IsNaN(clampedZ) || float.IsInfinity(clampedZ))
//             clampedZ = 0f;

//         legControls[legIndex].xRot = clampedY;
//         legControls[legIndex].yRot = clampedZ;
//         legControls[legIndex].zRot = clampedZ; // simplified: yRot drives zRot

//         legControls[legIndex].setPose();
//     }

//     Vector3 GetEndEffectorLocal(int legIndex)
//     {
//         Vector3 worldPos = legControls[legIndex].endEffectorPosition;
//         return transform.InverseTransformPoint(worldPos);
//     }

//     // ------------------------------------------------------------------------
//     //  DEBUG DRAWING
//     // ------------------------------------------------------------------------
//     void DebugDrawPoint(Vector3 localVector, Color color, float duration = -1f)
//     {
//         if (duration < 0) duration = debugPointDuration;
//         float size = 0.01f;
//         Vector3 worldPoint = transform.position + transform.TransformDirection(localVector);

//         Debug.DrawLine(worldPoint - Vector3.up * size, worldPoint + Vector3.up * size, color, duration);
//         Debug.DrawLine(worldPoint - Vector3.right * size, worldPoint + Vector3.right * size, color, duration);
//         Debug.DrawLine(worldPoint - Vector3.forward * size, worldPoint + Vector3.forward * size, color, duration);
//     }
// }

























/// working version 1

// public class SpiderLegsPoseSolver : MonoBehaviour
// {
//     [Header("Sampling")]
//     public int samplesPerAxis = 16;

//     [Header("Control Mode")]
//     public bool simetricalControl = true; // "symmetrical" typo kept for compatibility

//     [Header("Debug")]
//     public bool debugDictionaryPoints = false;
//     public float debugPointDuration = 2f;

//     [Header("Legs")]
//     public SpiderLegControls[] legControls;
//     [Header("Reachability")]
//     public bool enableReachabilityCheck = true;
//     public float reachabilitySafetyMargin = 0.05f; // 5cm buffer

//     // Dictionaries: leg index → precomputed pose data
//     private Dictionary<int, IndexedDictionary> legPoseDicts;

//     // Step size in pose space
//     private Vector3 stepSize;
//     private Vector3[] lastTargetPositions; // To track target changes and skip solving if unchanged

//     void Start()
//     {
//         // 1. Sort legs by legIndex (ascending)
//         SortLegControls();

//         // 2. Calculate sampling step
//         stepSize = new Vector3(
//             1f / (samplesPerAxis - 1),
//             2f / (samplesPerAxis - 1),
//             2f / (samplesPerAxis - 1)
//         );

//         // 3. Determine which legs need a dictionary
//         int[] legsToGenerate = simetricalControl ? GetRightLegIndices() : GetAllLegIndices();

//         // 4. Create dictionaries for those legs
//         CreatePoseDictionaries(legsToGenerate);
//         // 5. Initialize last target positions
//         lastTargetPositions = new Vector3[legControls.Length];
//     }

//     void Update()
//     {
//         // Move every leg
//         foreach (SpiderLegControls leg in legControls)
//         {
//             // if target positio relative to the body did not change since last frame, skip solving for this leg
//             if (transform.InverseTransformPoint(leg.target.position) == lastTargetPositions[leg.legIndex])
//             {
//                 continue;
//             }

//             int legIdx = leg.legIndex;

//             Vector3 targetLocal = transform.InverseTransformPoint(leg.target.position);

//             if (simetricalControl && leg.legIndex != leg.legPairIndex)
//             {
//                 targetLocal.x = -targetLocal.x;
//             }

//             int dictIndex = simetricalControl ? leg.legPairIndex : leg.legIndex;
//             if (!legPoseDicts.ContainsKey(dictIndex))
//             {
//                 Debug.LogError($"No pose dictionary for leg index {dictIndex}");
//                 continue;
//             }

//             Vector3 bestPose = FindClosestPoseValues(targetLocal, legPoseDicts[dictIndex], legIdx);

//             ApplyPose(legIdx, bestPose);
//             Vector3 legTargetWorld = leg.target.position;
//             lastTargetPositions[legIdx] = transform.InverseTransformPoint(legTargetWorld);
//         }
//     }

//     // ------------------------------------------------------------------------
//     //  PUBLIC CORE METHOD – SOLVE FOR ONE LEG
//     // ------------------------------------------------------------------------
//     Vector3 FindClosestPoseValues(Vector3 targetVector, IndexedDictionary dict, int legIndex)
//     {
//         // ------------------------------------------------------------
//         // 1. Find 3 closest leg‑vectors (by Euclidean distance)
//         // ------------------------------------------------------------
//         int closestCount = 3;
//         int[] closestByDist = new int[closestCount];
//         float minDist = float.MaxValue;

//         for (int i = 0; i < dict.Count; i++)
//         {
//             Vector3 key = dict.GetKeyByIndex(i);
//             float dist = Vector3.Distance(targetVector, key);
//             if (dist < minDist)
//             {
//                 minDist = dist;
//                 for (int j = closestCount - 1; j > 0; j--)
//                     closestByDist[j] = closestByDist[j - 1];
//                 closestByDist[0] = i;
//             }
//         }

//         // ------------------------------------------------------------
//         // 2. Among these 3, pick the one with smallest pose error
//         // ------------------------------------------------------------
//         Vector3 currentPose = new Vector3(
//             legControls[legIndex].reach,
//             legControls[legIndex].xRot,
//             legControls[legIndex].yRot
//         );

//         int bestIdx = closestByDist[0];
//         float bestPoseError = Vector3.Distance(currentPose, dict.GetByIndex(bestIdx));
//         for (int i = 1; i < closestCount; i++)
//         {
//             float err = Vector3.Distance(currentPose, dict.GetByIndex(closestByDist[i]));
//             if (err < bestPoseError)
//             {
//                 bestPoseError = err;
//                 bestIdx = closestByDist[i];
//             }
//         }

//         // ------------------------------------------------------------
//         // 3. Determine the best cube using dot‑product alignment
//         // ------------------------------------------------------------
//         Vector3 baseKey = dict.GetKeyByIndex(bestIdx);
//         Vector3 toTarget = targetVector - baseKey;

//         int selDx = 0, selDy = 0, selDz = 0;
//         float bestDot = -float.MaxValue;

//         for (int dx = -1; dx <= 1; dx += 2)
//         for (int dy = -1; dy <= 1; dy += 2)
//         for (int dz = -1; dz <= 1; dz += 2)
//         {
//             int farIdx = bestIdx + dx + dy * samplesPerAxis + dz * samplesPerAxis * samplesPerAxis;
//             if (farIdx < 0 || farIdx >= dict.Count) continue;

//             Vector3 farKey = dict.GetKeyByIndex(farIdx);
//             Vector3 diagonal = farKey - baseKey;

//             float dot = Vector3.Dot(toTarget.normalized, diagonal.normalized);
//             if (dot > bestDot)
//             {
//                 bestDot = dot;
//                 selDx = dx;
//                 selDy = dy;
//                 selDz = dz;
//             }
//         }

//         // ------------------------------------------------------------
//         // 4. Compute the 8 corners of the selected cube
//         // ------------------------------------------------------------
//         int o000 = bestIdx;
//         int o100 = bestIdx + selDx;
//         int o010 = bestIdx + selDy * samplesPerAxis;
//         int o001 = bestIdx + selDz * samplesPerAxis * samplesPerAxis;
//         int o110 = bestIdx + selDx + selDy * samplesPerAxis;
//         int o101 = bestIdx + selDx + selDz * samplesPerAxis * samplesPerAxis;
//         int o011 = bestIdx + selDy * samplesPerAxis + selDz * samplesPerAxis * samplesPerAxis;
//         int o111 = bestIdx + selDx + selDy * samplesPerAxis + selDz * samplesPerAxis * samplesPerAxis;

//         int[] cubeCorners = { o000, o100, o010, o001, o110, o101, o011, o111 };
//         bool cubeValid = true;
//         foreach (int idx in cubeCorners)
//             if (idx < 0 || idx >= dict.Count) { cubeValid = false; break; }

//         // ------------------------------------------------------------
//         // 5. DEBUG – draw the selected cube
//         // ------------------------------------------------------------
//         if (debugDictionaryPoints)
//         {
//             DebugDrawPoint(baseKey, Color.yellow);
//             DebugDrawPoint(targetVector, Color.white);
//             Debug.DrawLine(
//                 transform.position + transform.TransformDirection(baseKey),
//                 transform.position + transform.TransformDirection(targetVector),
//                 Color.green, 0.1f
//             );
//             foreach (int idx in cubeCorners)
//                 if (idx >= 0 && idx < dict.Count)
//                     DebugDrawPoint(dict.GetKeyByIndex(idx), Color.cyan);
//         }

//         // ------------------------------------------------------------
//         // 6. Define the 6 tetrahedra that partition the cube
//         // ------------------------------------------------------------
//         int[][] tetrahedra = new int[6][];
//         tetrahedra[0] = new int[] { o000, o100, o010, o111 };
//         tetrahedra[1] = new int[] { o000, o100, o001, o111 };
//         tetrahedra[2] = new int[] { o000, o010, o001, o111 };
//         tetrahedra[3] = new int[] { o100, o010, o110, o111 };
//         tetrahedra[4] = new int[] { o100, o001, o101, o111 };
//         tetrahedra[5] = new int[] { o010, o001, o011, o111 };

//         // ------------------------------------------------------------
//         // 7. Search for a tetrahedron that contains the target
//         // ------------------------------------------------------------
//         int[] selectedTet = null;
//         if (cubeValid)
//         {
//             foreach (var tet in tetrahedra)
//             {
//                 Vector3 p0 = dict.GetKeyByIndex(tet[0]);
//                 Vector3 p1 = dict.GetKeyByIndex(tet[1]);
//                 Vector3 p2 = dict.GetKeyByIndex(tet[2]);
//                 Vector3 p3 = dict.GetKeyByIndex(tet[3]);

//                 if (IsPointInsideTetrahedron(targetVector, p0, p1, p2, p3))
//                 {
//                     selectedTet = tet;
//                     break;
//                 }
//             }
//         }

//         // ------------------------------------------------------------
//         // 8. If no tetrahedron found, try ALL 8 cubes
//         // ------------------------------------------------------------
//         if (selectedTet == null)
//         {
//             if (debugDictionaryPoints)
//                 Debug.Log("Selected cube failed – testing all 8 cubes.");

//             for (int dx2 = -1; dx2 <= 1; dx2 += 2)
//             for (int dy2 = -1; dy2 <= 1; dy2 += 2)
//             for (int dz2 = -1; dz2 <= 1; dz2 += 2)
//             {
//                 if (dx2 == selDx && dy2 == selDy && dz2 == selDz) continue;

//                 int c000 = bestIdx;
//                 int c100 = bestIdx + dx2;
//                 int c010 = bestIdx + dy2 * samplesPerAxis;
//                 int c001 = bestIdx + dz2 * samplesPerAxis * samplesPerAxis;
//                 int c110 = bestIdx + dx2 + dy2 * samplesPerAxis;
//                 int c101 = bestIdx + dx2 + dz2 * samplesPerAxis * samplesPerAxis;
//                 int c011 = bestIdx + dy2 * samplesPerAxis + dz2 * samplesPerAxis * samplesPerAxis;
//                 int c111 = bestIdx + dx2 + dy2 * samplesPerAxis + dz2 * samplesPerAxis * samplesPerAxis;

//                 int[] cube = { c000, c100, c010, c001, c110, c101, c011, c111 };
//                 bool valid = true;
//                 foreach (int idx in cube)
//                     if (idx < 0 || idx >= dict.Count) { valid = false; break; }
//                 if (!valid) continue;

//                 int[][] tetras = new int[6][];
//                 tetras[0] = new int[] { c000, c100, c010, c111 };
//                 tetras[1] = new int[] { c000, c100, c001, c111 };
//                 tetras[2] = new int[] { c000, c010, c001, c111 };
//                 tetras[3] = new int[] { c100, c010, c110, c111 };
//                 tetras[4] = new int[] { c100, c001, c101, c111 };
//                 tetras[5] = new int[] { c010, c001, c011, c111 };

//                 foreach (var tet in tetras)
//                 {
//                     Vector3 p0 = dict.GetKeyByIndex(tet[0]);
//                     Vector3 p1 = dict.GetKeyByIndex(tet[1]);
//                     Vector3 p2 = dict.GetKeyByIndex(tet[2]);
//                     Vector3 p3 = dict.GetKeyByIndex(tet[3]);

//                     if (IsPointInsideTetrahedron(targetVector, p0, p1, p2, p3))
//                     {
//                         selectedTet = tet;
//                         break;
//                     }
//                 }
//                 if (selectedTet != null) break;
//             }
//         }

//         // ------------------------------------------------------------
//         // 9. Found a tetrahedron → interpolate
//         // ------------------------------------------------------------
//         if (selectedTet != null)
//         {
//             if (debugDictionaryPoints)
//             {
//                 foreach (int idx in selectedTet)
//                 {
//                     Vector3 key = dict.GetKeyByIndex(idx);
//                     DebugDrawPoint(key, Color.red);
//                 }
//             }

//             Vector3 p0 = dict.GetKeyByIndex(selectedTet[0]);
//             Vector3 p1 = dict.GetKeyByIndex(selectedTet[1]);
//             Vector3 p2 = dict.GetKeyByIndex(selectedTet[2]);
//             Vector3 p3 = dict.GetKeyByIndex(selectedTet[3]);

//             Vector3 v0 = dict.GetByIndex(selectedTet[0]);
//             Vector3 v1 = dict.GetByIndex(selectedTet[1]);
//             Vector3 v2 = dict.GetByIndex(selectedTet[2]);
//             Vector3 v3 = dict.GetByIndex(selectedTet[3]);

//             return TetrahedralInterpolation(targetVector, p0, p1, p2, p3, v0, v1, v2, v3);
//         }

//         // ------------------------------------------------------------
//         // 10. Fallback – Inverse Distance Weighting on 8 corners
//         // ------------------------------------------------------------
//         if (debugDictionaryPoints)
//             Debug.LogWarning("No tetrahedron contains target – using IDW fallback.");

//         return InverseDistanceWeighting(targetVector, cubeCorners, dict);
//     }

//     // ------------------------------------------------------------------------
//     //  GEOMETRIC HELPERS (tetrahedron, interpolation, IDW)
//     // ------------------------------------------------------------------------
//     float TetraVolume(Vector3 a, Vector3 b, Vector3 c, Vector3 d)
//     {
//         return Vector3.Dot(b - a, Vector3.Cross(c - a, d - a)) / 6f;
//     }

//     bool IsPointInsideTetrahedron(Vector3 p, Vector3 a, Vector3 b, Vector3 c, Vector3 d)
//     {
//         float vol = TetraVolume(a, b, c, d);
//         if (Mathf.Abs(vol) < 1e-6f) return false;

//         float vol1 = TetraVolume(p, b, c, d);
//         float vol2 = TetraVolume(a, p, c, d);
//         float vol3 = TetraVolume(a, b, p, d);
//         float vol4 = TetraVolume(a, b, c, p);

//         return (vol1 * vol >= 0) && (vol2 * vol >= 0) && (vol3 * vol >= 0) && (vol4 * vol >= 0);
//     }

//     Vector3 TetrahedralInterpolation(Vector3 target, Vector3 p0, Vector3 p1, Vector3 p2, Vector3 p3,
//                                      Vector3 v0, Vector3 v1, Vector3 v2, Vector3 v3)
//     {
//         float vol = TetraVolume(p0, p1, p2, p3);
//         if (Mathf.Abs(vol) < 1e-6f) return v0;

//         float w0 = TetraVolume(target, p1, p2, p3) / vol;
//         float w1 = TetraVolume(p0, target, p2, p3) / vol;
//         float w2 = TetraVolume(p0, p1, target, p3) / vol;
//         float w3 = TetraVolume(p0, p1, p2, target) / vol;

//         return w0 * v0 + w1 * v1 + w2 * v2 + w3 * v3;
//     }

//     Vector3 InverseDistanceWeighting(Vector3 target, int[] indices, IndexedDictionary dict)
//     {
//         Vector3 result = Vector3.zero;
//         float totalWeight = 0f;
//         float epsilon = 0.0001f;

//         foreach (int idx in indices)
//         {
//             if (idx < 0 || idx >= dict.Count) continue;
//             Vector3 key = dict.GetKeyByIndex(idx);
//             float dist = Vector3.Distance(target, key);
//             float weight = 1f / (dist + epsilon);
//             weight = weight * weight; // Sharper falloff

//             result += weight * dict.GetByIndex(idx);
//             totalWeight += weight;
//         }
//         return result / totalWeight;
//     }

//     // ------------------------------------------------------------------------
//     //  DICTIONARY CREATION
//     // ------------------------------------------------------------------------
//     void CreatePoseDictionaries(int[] legIndices)
//     {
//         legPoseDicts = new Dictionary<int, IndexedDictionary>();
//         foreach (int legIdx in legIndices)
//         {
//             Debug.Log($"Creating indexed pose dictionary for leg {legIdx}");
//             legPoseDicts[legIdx] = BuildDictionaryForLeg(legIdx);
//         }
//     }

//     IndexedDictionary BuildDictionaryForLeg(int legIndex)
//     {
//         int size = samplesPerAxis * samplesPerAxis * samplesPerAxis;
//         IndexedDictionary dict = new IndexedDictionary(size);

//         for (int x = 0; x < samplesPerAxis; x++)
//         {
//             for (int y = 0; y < samplesPerAxis; y++)
//             {
//                 for (int z = 0; z < samplesPerAxis; z++)
//                 {
//                     Vector3 pose = new Vector3(
//                         x * stepSize.x,
//                         y * stepSize.y - 1f,
//                         z * stepSize.z - 1f
//                     );

//                     Vector3 footLocal = PoseToFootLocal(legIndex, pose);
//                     if (debugDictionaryPoints)
//                     {
//                         float s = samplesPerAxis;
//                         DebugDrawPoint(footLocal, new Color(x / s, y / s, z / s), debugPointDuration);
//                     }
//                     dict.Add(footLocal, pose);
//                 }
//             }
//         }
//         return dict;
//     }

//     // ------------------------------------------------------------------------
//     //  LEG UTILITIES
//     // ------------------------------------------------------------------------
//     void SortLegControls()
//     {
//         legControls = GetComponentsInChildren<SpiderLegControls>();
//         for (int i = 0; i < legControls.Length - 1; i++)
//         {
//             for (int j = i + 1; j < legControls.Length; j++)
//             {
//                 if (legControls[i].legIndex > legControls[j].legIndex)
//                 {
//                     SpiderLegControls temp = legControls[i];
//                     legControls[i] = legControls[j];
//                     legControls[j] = temp;
//                 }
//             }
//         }
//     }

//     int[] GetRightLegIndices()
//     {
//         List<int> rightIndices = new List<int>();
//         foreach (SpiderLegControls leg in legControls)
//         {
//             if (leg.legIndex < 4) // assuming 0‑3 are right legs
//                 rightIndices.Add(leg.legIndex);
//         }
//         return rightIndices.ToArray();
//     }

//     int[] GetAllLegIndices()
//     {
//         int[] all = new int[legControls.Length];
//         for (int i = 0; i < legControls.Length; i++)
//             all[i] = legControls[i].legIndex;
//         return all;
//     }

//     Vector3 PoseToFootLocal(int legIndex, Vector3 poseValues)
//     {
//         ApplyPose(legIndex, poseValues);
//         return GetEndEffectorLocal(legIndex);
//     }

//     void ApplyPose(int legIndex, Vector3 pose)
//     {
//         legControls[legIndex].reach = pose.x;
//         legControls[legIndex].xRot = pose.y;
//         legControls[legIndex].yRot = pose.z;
//         legControls[legIndex].zRot = pose.z; // simplified: yRot drives zRot
//         legControls[legIndex].setPose();
//     }

//     Vector3 GetEndEffectorLocal(int legIndex)
//     {
//         Vector3 worldPos = legControls[legIndex].endEffectorPosition;
//         return transform.InverseTransformPoint(worldPos);
//     }

//     // ------------------------------------------------------------------------
//     //  DEBUG DRAWING
//     // ------------------------------------------------------------------------
//     void DebugDrawPoint(Vector3 localVector, Color color, float duration = -1f)
//     {
//         if (duration < 0) duration = debugPointDuration;
//         float size = 0.01f;
//         Vector3 worldPoint = transform.position + transform.TransformDirection(localVector);

//         Debug.DrawLine(worldPoint - Vector3.up * size, worldPoint + Vector3.up * size, color, duration);
//         Debug.DrawLine(worldPoint - Vector3.right * size, worldPoint + Vector3.right * size, color, duration);
//         Debug.DrawLine(worldPoint - Vector3.forward * size, worldPoint + Vector3.forward * size, color, duration);
//     }
// }