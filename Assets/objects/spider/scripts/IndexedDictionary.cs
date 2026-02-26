using System.Collections.Generic;
// using System;
using UnityEngine;
using System.IO;
// using System.Runtime.Serialization.Formatters.Binary;

public class IndexedDictionary
{
    private Vector3[] _keys;
    private Vector3[] _values;
    private Dictionary<Vector3, int> _keyToIndex;
    private int _count;

    // Spatial grid
    private struct GridCell { public List<int> indices; }
    private GridCell[,,] _grid;
    private Vector3 _gridMin, _gridCellSize;
    private int _gridRes = 4;
    private bool _gridBuilt = false;

    public int Count => _count;
    public int Capacity => _keys.Length;
    public bool HasSpatialIndex => _gridBuilt;

    // File format version
    private const int FILE_VERSION = 1;

    // ----- Constructors ----------------------------------------------------
    public IndexedDictionary(int size)
    {
        _keys = new Vector3[size];
        _values = new Vector3[size];
        _keyToIndex = new Dictionary<Vector3, int>(size);
        _count = 0;
    }
    private IndexedDictionary() { }

    // ----- Add / Get -------------------------------------------------------
    public void Add(Vector3 key, Vector3 value)
    {
        if (_count >= Capacity)
            throw new System.InvalidOperationException("IndexedDictionary is full.");
        if (_keyToIndex.ContainsKey(key))
            throw new System.ArgumentException("Key already exists.");

        _keys[_count] = key;
        _values[_count] = value;
        _keyToIndex[key] = _count;
        _count++;
    }
    public Vector3 GetByIndex(int index) => _values[index];
    public Vector3 GetKeyByIndex(int index) => _keys[index];

    // ----- Spatial Index ---------------------------------------------------
    public void BuildSpatialIndex(int gridResolution = 4)
    {
        if (_count == 0) return;
        _gridRes = Mathf.Max(2, gridResolution);

        Vector3 min = _keys[0], max = _keys[0];
        for (int i = 1; i < _count; i++)
        {
            min = Vector3.Min(min, _keys[i]);
            max = Vector3.Max(max, _keys[i]);
        }
        min -= Vector3.one * 0.01f;
        max += Vector3.one * 0.01f;

        _gridMin = min;
        _gridCellSize = (max - min) / _gridRes;
        _grid = new GridCell[_gridRes, _gridRes, _gridRes];

        for (int i = 0; i < _count; i++)
        {
            Vector3 key = _keys[i];
            int gx = Mathf.Clamp(Mathf.FloorToInt((key.x - _gridMin.x) / _gridCellSize.x), 0, _gridRes - 1);
            int gy = Mathf.Clamp(Mathf.FloorToInt((key.y - _gridMin.y) / _gridCellSize.y), 0, _gridRes - 1);
            int gz = Mathf.Clamp(Mathf.FloorToInt((key.z - _gridMin.z) / _gridCellSize.z), 0, _gridRes - 1);
            if (_grid[gx, gy, gz].indices == null)
                _grid[gx, gy, gz].indices = new List<int>();
            _grid[gx, gy, gz].indices.Add(i);
        }
        _gridBuilt = true;
    }

    public int[] FindNearest(Vector3 target, int k)
    {
        if (!_gridBuilt) return FindNearestLinear(target, k);

        int gx = Mathf.Clamp(Mathf.FloorToInt((target.x - _gridMin.x) / _gridCellSize.x), 0, _gridRes - 1);
        int gy = Mathf.Clamp(Mathf.FloorToInt((target.y - _gridMin.y) / _gridCellSize.y), 0, _gridRes - 1);
        int gz = Mathf.Clamp(Mathf.FloorToInt((target.z - _gridMin.z) / _gridCellSize.z), 0, _gridRes - 1);

        List<int> candidates = new List<int>();
        for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++)
        for (int dz = -1; dz <= 1; dz++)
        {
            int nx = gx + dx, ny = gy + dy, nz = gz + dz;
            if (nx < 0 || nx >= _gridRes || ny < 0 || ny >= _gridRes || nz < 0 || nz >= _gridRes)
                continue;
            var cell = _grid[nx, ny, nz];
            if (cell.indices != null)
                candidates.AddRange(cell.indices);
        }

        if (candidates.Count < k)
            return FindNearestLinear(target, k);

        candidates.Sort((a, b) =>
            Vector3.SqrMagnitude(_keys[a] - target).CompareTo(Vector3.SqrMagnitude(_keys[b] - target)));

        int[] result = new int[k];
        for (int i = 0; i < k; i++)
            result[i] = candidates[i];
        return result;
    }

    private int[] FindNearestLinear(Vector3 target, int k)
    {
        var list = new List<(int idx, float sqDist)>();
        for (int i = 0; i < _count; i++)
        {
            float sqDist = Vector3.SqrMagnitude(_keys[i] - target);
            list.Add((i, sqDist));
        }
        list.Sort((a, b) => a.sqDist.CompareTo(b.sqDist));
        int[] result = new int[k];
        for (int i = 0; i < k && i < list.Count; i++)
            result[i] = list[i].idx;
        return result;
    }

    public bool IsPointInBounds(Vector3 point)
    {
        if (!_gridBuilt) return true;
        return point.x >= _gridMin.x && point.x <= _gridMin.x + _gridRes * _gridCellSize.x &&
               point.y >= _gridMin.y && point.y <= _gridMin.y + _gridRes * _gridCellSize.y &&
               point.z >= _gridMin.z && point.z <= _gridMin.z + _gridRes * _gridCellSize.z;
    }

    // ------------------------------------------------------------------------
    //  ROBUST BINARY SERIALIZATION with version header & error handling
    // ------------------------------------------------------------------------
    public void SaveToFile(string filePath)
    {
        // Ensure directory exists
        string directory = Path.GetDirectoryName(filePath);
        if (!string.IsNullOrEmpty(directory) && !Directory.Exists(directory))
            Directory.CreateDirectory(directory);

        using (BinaryWriter writer = new BinaryWriter(File.Open(filePath, FileMode.Create)))
        {
            // Version header
            writer.Write(FILE_VERSION);
            // Write count
            writer.Write(_count);
            // Write grid parameters
            writer.Write(_gridRes);
            writer.Write(_gridMin.x);
            writer.Write(_gridMin.y);
            writer.Write(_gridMin.z);
            writer.Write(_gridCellSize.x);
            writer.Write(_gridCellSize.y);
            writer.Write(_gridCellSize.z);

            // Write all keys
            for (int i = 0; i < _count; i++)
            {
                writer.Write(_keys[i].x);
                writer.Write(_keys[i].y);
                writer.Write(_keys[i].z);
            }

            // Write all values
            for (int i = 0; i < _count; i++)
            {
                writer.Write(_values[i].x);
                writer.Write(_values[i].y);
                writer.Write(_values[i].z);
            }
        }
    }

    public static IndexedDictionary LoadFromFile(string filePath)
    {
        if (!File.Exists(filePath))
            return null;

        try
        {
            using (BinaryReader reader = new BinaryReader(File.Open(filePath, FileMode.Open, FileAccess.Read)))
            {
                // ----- Minimal length check -----
                if (reader.BaseStream.Length < 4)
                    return null;

                // ----- Read version header -----
                int fileVersion = reader.ReadInt32();
                if (fileVersion != FILE_VERSION)
                {
                    Debug.LogWarning($"Cache file {filePath} has unsupported version {fileVersion}. Expected {FILE_VERSION}. Rebuilding.");
                    return null;
                }

                // ----- Read count -----
                int count = reader.ReadInt32();
                if (count <= 0 || count > 1000000) // sanity
                    return null;

                // ----- Verify remaining length (rough estimate) -----
                long expectedMinSize = reader.BaseStream.Position + 4 + 24 + (count * 3 * 4) + (count * 3 * 4);
                if (reader.BaseStream.Length < expectedMinSize)
                    return null;

                // ----- Read grid parameters -----
                int gridRes = reader.ReadInt32();
                Vector3 gridMin = new Vector3(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());
                Vector3 gridCellSize = new Vector3(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());

                // ----- Read keys -----
                Vector3[] keys = new Vector3[count];
                for (int i = 0; i < count; i++)
                    keys[i] = new Vector3(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());

                // ----- Read values -----
                Vector3[] values = new Vector3[count];
                for (int i = 0; i < count; i++)
                    values[i] = new Vector3(reader.ReadSingle(), reader.ReadSingle(), reader.ReadSingle());

                // ----- Reconstruct dictionary -----
                IndexedDictionary dict = new IndexedDictionary
                {
                    _keys = keys,
                    _values = values,
                    _count = count,
                    _gridRes = gridRes,
                    _gridMin = gridMin,
                    _gridCellSize = gridCellSize,
                    _gridBuilt = false
                };

                // ----- Rebuild key->index map -----
                dict._keyToIndex = new Dictionary<Vector3, int>(count);
                for (int i = 0; i < count; i++)
                    dict._keyToIndex[dict._keys[i]] = i;

                // ----- Rebuild spatial grid -----
                if (gridRes > 0)
                    dict.BuildSpatialIndex(gridRes);

                return dict;
            }
        }
        catch (EndOfStreamException)
        {
            Debug.LogWarning($"Cache file {filePath} is incomplete or corrupt. Rebuilding.");
            return null;
        }
        catch (IOException)
        {
            Debug.LogWarning($"Could not read cache file {filePath}. Rebuilding.");
            return null;
        }
    }
}




















/// working version 1

// public class IndexedDictionary
// {
//     private Vector3[] _keys;
//     private Vector3[] _values;
//     private Dictionary<Vector3, int> _keyToIndex;

//     private int _count;

//     public int Count => _count;
//     public int Capacity => _keys.Length;

//     public IndexedDictionary(int size)
//     {
//         _keys = new Vector3[size];
//         _values = new Vector3[size];
//         _keyToIndex = new Dictionary<Vector3, int>(size);
//         _count = 0;
//     }

//     // Add new entry
//     public void Add(Vector3 key, Vector3 value)
//     {
//         if (_count >= Capacity)
//             throw new InvalidOperationException("IndexedDictionary is full.");

//         if (_keyToIndex.ContainsKey(key))
//             throw new ArgumentException("Key already exists.");

//         _keys[_count] = key;
//         _values[_count] = value;
//         _keyToIndex[key] = _count;

//         _count++;
//     }

//     // Set value (add if not exists, update if exists)
//     public void Set(Vector3 key, Vector3 value)
//     {
//         if (_keyToIndex.TryGetValue(key, out int index))
//         {
//             _values[index] = value;
//         }
//         else
//         {
//             Add(key, value);
//         }
//     }

//     // Get by key
//     public Vector3 Get(Vector3 key)
//     {
//         if (_keyToIndex.TryGetValue(key, out int index))
//         {
//             return _values[index];
//         }

//         throw new KeyNotFoundException();
//     }

//     // TryGet by key (safe)
//     public bool TryGet(Vector3 key, out Vector3 value)
//     {
//         if (_keyToIndex.TryGetValue(key, out int index))
//         {
//             value = _values[index];
//             return true;
//         }

//         value = default;
//         return false;
//     }

//     // Get by index
//     public Vector3 GetByIndex(int index)
//     {
//         if (index < 0 || index >= _count)
//             throw new IndexOutOfRangeException();

//         return _values[index];
//     }

//     // Get key by index
//     public Vector3 GetKeyByIndex(int index)
//     {
//         if (index < 0 || index >= _count)
//             throw new IndexOutOfRangeException();

//         return _keys[index];
//     }

//     // Get index by key
//     public int GetIndex(Vector3 key)
//     {
//         if (_keyToIndex.TryGetValue(key, out int index))
//             return index;

//         throw new KeyNotFoundException();
//     }

//     // Check existence
//     public bool ContainsKey(Vector3 key)
//     {
//         return _keyToIndex.ContainsKey(key);
//     }
// }




















/// working version 2 with spatial index and serialization

// public class IndexedDictionary
// {
//     private Vector3[] _keys;
//     private Vector3[] _values;
//     private Dictionary<Vector3, int> _keyToIndex;
//     private int _count;

//     // Spatial grid for fast nearest neighbour search
//     private struct GridCell
//     {
//         public List<int> indices;
//     }
//     private GridCell[,,] _grid;
//     private Vector3 _gridMin, _gridCellSize;
//     private int _gridRes = 4;               // 4×4×4 cells – tune this (memory vs speed)
//     private bool _gridBuilt = false;

//     public int Count => _count;
//     public int Capacity => _keys.Length;
//     public bool HasSpatialIndex => _gridBuilt;

//     public IndexedDictionary(int size)
//     {
//         _keys = new Vector3[size];
//         _values = new Vector3[size];
//         _keyToIndex = new Dictionary<Vector3, int>(size);
//         _count = 0;
//     }

//     // ----- Add / Set / Get -------------------------------------------------
//     public void Add(Vector3 key, Vector3 value)
//     {
//         if (_count >= Capacity)
//             throw new System.InvalidOperationException("IndexedDictionary is full.");
//         if (_keyToIndex.ContainsKey(key))
//             throw new System.ArgumentException("Key already exists.");

//         _keys[_count] = key;
//         _values[_count] = value;
//         _keyToIndex[key] = _count;
//         _count++;
//     }

//     public void Set(Vector3 key, Vector3 value)
//     {
//         if (_keyToIndex.TryGetValue(key, out int index))
//             _values[index] = value;
//         else
//             Add(key, value);
//     }

//     public Vector3 Get(Vector3 key) => _values[_keyToIndex[key]];
//     public bool TryGet(Vector3 key, out Vector3 value)
//     {
//         if (_keyToIndex.TryGetValue(key, out int index))
//         {
//             value = _values[index];
//             return true;
//         }
//         value = default;
//         return false;
//     }

//     public Vector3 GetByIndex(int index) => _values[index];
//     public Vector3 GetKeyByIndex(int index) => _keys[index];
//     public int GetIndex(Vector3 key) => _keyToIndex[key];
//     public bool ContainsKey(Vector3 key) => _keyToIndex.ContainsKey(key);

//     // ----- Spatial Index ---------------------------------------------------
//     public void BuildSpatialIndex(int gridResolution = 4)
//     {
//         if (_count == 0) return;
//         _gridRes = Mathf.Max(2, gridResolution);

//         // Find axis‑aligned bounds of all keys
//         Vector3 min = _keys[0], max = _keys[0];
//         for (int i = 1; i < _count; i++)
//         {
//             min = Vector3.Min(min, _keys[i]);
//             max = Vector3.Max(max, _keys[i]);
//         }
//         // Add small padding
//         min -= Vector3.one * 0.01f;
//         max += Vector3.one * 0.01f;

//         _gridMin = min;
//         _gridCellSize = (max - min) / _gridRes;
//         _grid = new GridCell[_gridRes, _gridRes, _gridRes];

//         // Insert each key into its cell
//         for (int i = 0; i < _count; i++)
//         {
//             Vector3 key = _keys[i];
//             int gx = Mathf.Clamp(Mathf.FloorToInt((key.x - _gridMin.x) / _gridCellSize.x), 0, _gridRes - 1);
//             int gy = Mathf.Clamp(Mathf.FloorToInt((key.y - _gridMin.y) / _gridCellSize.y), 0, _gridRes - 1);
//             int gz = Mathf.Clamp(Mathf.FloorToInt((key.z - _gridMin.z) / _gridCellSize.z), 0, _gridRes - 1);
//             if (_grid[gx, gy, gz].indices == null)
//                 _grid[gx, gy, gz].indices = new List<int>();
//             _grid[gx, gy, gz].indices.Add(i);
//         }
//         _gridBuilt = true;
//     }

//     // Fast nearest neighbour search using grid
//     public int[] FindNearest(Vector3 target, int k)
//     {
//         if (!_gridBuilt) return FindNearestLinear(target, k);

//         // Find cell containing target
//         int gx = Mathf.Clamp(Mathf.FloorToInt((target.x - _gridMin.x) / _gridCellSize.x), 0, _gridRes - 1);
//         int gy = Mathf.Clamp(Mathf.FloorToInt((target.y - _gridMin.y) / _gridCellSize.y), 0, _gridRes - 1);
//         int gz = Mathf.Clamp(Mathf.FloorToInt((target.z - _gridMin.z) / _gridCellSize.z), 0, _gridRes - 1);

//         // Collect candidates from 3×3×3 neighbourhood
//         List<int> candidates = new List<int>();
//         for (int dx = -1; dx <= 1; dx++)
//         for (int dy = -1; dy <= 1; dy++)
//         for (int dz = -1; dz <= 1; dz++)
//         {
//             int nx = gx + dx, ny = gy + dy, nz = gz + dz;
//             if (nx < 0 || nx >= _gridRes || ny < 0 || ny >= _gridRes || nz < 0 || nz >= _gridRes)
//                 continue;
//             var cell = _grid[nx, ny, nz];
//             if (cell.indices != null)
//                 candidates.AddRange(cell.indices);
//         }

//         // Fallback to linear search if not enough candidates
//         if (candidates.Count < k)
//             return FindNearestLinear(target, k);

//         // Sort by distance squared and return first k
//         candidates.Sort((a, b) =>
//             Vector3.SqrMagnitude(_keys[a] - target)
//                 .CompareTo(Vector3.SqrMagnitude(_keys[b] - target)));

//         int[] result = new int[k];
//         for (int i = 0; i < k; i++)
//             result[i] = candidates[i];
//         return result;
//     }

//     // Linear search fallback (your original method, but using squared distance)
//     private int[] FindNearestLinear(Vector3 target, int k)
//     {
//         var list = new List<(int idx, float sqDist)>();
//         for (int i = 0; i < _count; i++)
//         {
//             float sqDist = Vector3.SqrMagnitude(_keys[i] - target);
//             list.Add((i, sqDist));
//         }
//         list.Sort((a, b) => a.sqDist.CompareTo(b.sqDist));
//         int[] result = new int[k];
//         for (int i = 0; i < k && i < list.Count; i++)
//             result[i] = list[i].idx;
//         return result;
//     }

//     // ----- Bounds check for culling ----------------------------------------
//     public bool IsPointInBounds(Vector3 point)
//     {
//         if (!_gridBuilt) return true; // cannot check, assume true
//         return point.x >= _gridMin.x && point.x <= _gridMin.x + _gridRes * _gridCellSize.x &&
//                point.y >= _gridMin.y && point.y <= _gridMin.y + _gridRes * _gridCellSize.y &&
//                point.z >= _gridMin.z && point.z <= _gridMin.z + _gridRes * _gridCellSize.z;
//     }
// }


















