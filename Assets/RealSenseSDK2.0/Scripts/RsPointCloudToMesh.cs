using System;
using UnityEngine;
using Intel.RealSense;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Assertions;
using System.Runtime.InteropServices;
using System.Threading;
using System.Collections.Generic;
using System.Linq;

using CGALDotNet;
using CGALDotNetGeometry.Numerics;
using CGALDotNet.Polyhedra;
using CGALDotNet.Hulls;
using System.Diagnostics;
using static CGALDotNet.CGALGlobal;
using CGALDotNet.Polygons;
using System.Data.Common;
using System.Security.Cryptography;
using System.Runtime.ExceptionServices;
using System.Runtime.Remoting.Messaging;
using System.Linq.Expressions;
using UnityEngine.AI;
using System.Net.NetworkInformation;
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class RsPointCloudToMesh : MonoBehaviour
{
    private const float BOX_DIST_THRESHOLD = 0.3f;
    private const float PLANE_DIST_THRESHOLD = 2;
    private const int MIN_POINTS_FOR_SHAPE = 100; // Adjust as needed
    private int key_count = 0;
    public Material hullMaterial;
    private Mesh mesh;
    private Texture2D uvmap;
    private FrameQueue q;
    private Dictionary<int, GameObject> GameObjReference = new Dictionary<int, GameObject>();
    private Dictionary<int, Vector3[]> VerticesReference = new Dictionary<int, Vector3[]>();
    private Dictionary<int, Vector3> PlaneNormalReference = new Dictionary<int, Vector3>();
    // Create a dictionary to store voxel counts
    private Dictionary<Vector3Int, int> voxelCounts = new Dictionary<Vector3Int, int>();
        private Dictionary<Vector3Int, GameObject> voxelRef = new Dictionary<Vector3Int, GameObject>();
    private readonly int[] cubeTriangles = new int[] 
    {
        // Front face
        0, 1, 2,
        0, 2, 3,
        // Back face
        4, 6, 5,
        4, 7, 6,
        // Top face
        5, 6, 2,
        5, 2, 1,
        // Bottom face
        0, 3, 7,
        0, 7, 4,
        // Left face
        0, 4, 5,
        0, 5, 1,
        // Right face
        3, 2, 6,
        3, 6, 7
    };

    public RsFrameProvider Source;

    [NonSerialized]
    private Vector3[] vertices;
    public int n = 3;

//     // private int totalTimes;
//     // private float totalSum;
//     // private float minSum = float.MaxValue;
//     // public float maxSum = float.MinValue;

    void Start()
    {
        Source.OnStart += OnStartStreaming;
        Source.OnStop += Dispose;
    }

    private void OnStartStreaming(PipelineProfile obj)
    {
        // Changes <
        Initialize();
        // Changes >
        q = new FrameQueue(1);

        using (var depth = obj.Streams.FirstOrDefault(s => s.Stream == Stream.Depth && s.Format == Format.Z16).As<VideoStreamProfile>())
            ResetMesh(depth.Width, depth.Height);

        Source.OnNewSample += OnNewSample;
    }

    private void ResetMesh(int width, int height)
    {
        Assert.IsTrue(SystemInfo.SupportsTextureFormat(TextureFormat.RGFloat));
        uvmap = new Texture2D(width, height, TextureFormat.RGFloat, false, true)
        {
            wrapMode = TextureWrapMode.Clamp,
            filterMode = FilterMode.Point,
        };
        GetComponent<MeshRenderer>().sharedMaterial.SetTexture("_UVMap", uvmap);

        if (mesh != null)
            mesh.Clear();
        else
            mesh = new Mesh()
            {
                indexFormat = IndexFormat.UInt32,
            };

        vertices = new Vector3[width * height];

        var indices = new int[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
            indices[i] = i;

        mesh.MarkDynamic();
        mesh.vertices = vertices;

        var uvs = new Vector2[width * height];
        Array.Clear(uvs, 0, uvs.Length);
        for (int j = 0; j < height; j++)
        {
            for (int i = 0; i < width; i++)
            {
                uvs[i + j * width].x = i / (float)width;
                uvs[i + j * width].y = j / (float)height;
            }
        }

        mesh.uv = uvs;

        mesh.SetIndices(indices, MeshTopology.Points, 0, false);
        mesh.bounds = new Bounds(Vector3.zero, Vector3.one * 10f);

        GetComponent<MeshFilter>().sharedMesh = mesh;
    }

    void OnDestroy()
    {
        if (q != null)
        {
            q.Dispose();
            q = null;
        }

        if (mesh != null)
            Destroy(null);
    }

    private void Dispose()
    {
        Source.OnNewSample -= OnNewSample;

        if (q != null)
        {
            q.Dispose();
            q = null;
        }
    }

    private void OnNewSample(Frame frame)
    {
        if (q == null)
            return;
        try
        {
            if (frame.IsComposite)
            {
                using (var fs = frame.As<FrameSet>())
                using (var points = fs.FirstOrDefault<Points>(Stream.Depth, Format.Xyz32f))
                {
                    if (points != null)
                    {
                        q.Enqueue(points);
                    }
                }
                return;
            }

            if (frame.Is(Extension.Points))
            {
                q.Enqueue(frame);
            }
        }
        catch (Exception e)
        {
            UnityEngine.Debug.LogException(e);
        }
    }


    protected void LateUpdate()
    {
        if (q != null)
        {
            Points points;
            if (q.PollForFrame<Points>(out points))
                using (points)
                {
                    if (points.Count != mesh.vertexCount)
                    {
                        using (var p = points.GetProfile<VideoStreamProfile>())
                            ResetMesh(p.Width, p.Height);
                    }

                    if (points.TextureData != IntPtr.Zero)
                    {
                        uvmap.LoadRawTextureData(points.TextureData, points.Count * sizeof(float) * 2);
                        uvmap.Apply();
                    }

                    if (points.VertexData != IntPtr.Zero)
                    {
                        points.CopyVertices(vertices);

                        mesh.vertices = vertices;
                        mesh.UploadMeshData(false);

                        // Stopwatch sw = Stopwatch.StartNew();
                        Helper(n);
                        n++;
                        // sw.Stop();
                        // Get the elapsed time
                        // TimeSpan elapsedTime = sw.Elapsed;
                        // Print the time elapsed
                        // UnityEngine.Debug.Log("Time elapsed: " + elapsedTime.TotalMilliseconds);

                        // totalSum += elapsedTime.TotalMilliseconds;
                        // totalTimes++;

                        // minSum = minSum < elapsedTime.TotalMilliseconds ? minSum : elapsedTime.TotalMilliseconds;
                        // maxSum = maxSum > elapsedTime.TotalMilliseconds ? maxSum : elapsedTime.TotalMilliseconds;

                        // UnityEngine.Debug.Log("Average Time elapsed: " + totalSum/totalTimes + " min Time elapsed: " + minSum + " max Time elapsed: " + maxSum);
                        // UnityEngine.Debug.Log("N of vertices: " + vertices.Length + " Iteration: " + totalTimes + " boxes " + GameObjReference.Values.Count);
                        
                    }
                }
        }
    }

    private void Helper(int value)
    {
        // Define your voxel size
        float voxelSize = 0.2f; // Example size, adjust as needed

        // Iterate over all points and count points in each voxel
        if (value % 3 == 0) {
            foreach (var point in vertices)
            {
                Vector3Int voxelPosition = new Vector3Int(
                    Mathf.FloorToInt(point.x / voxelSize),
                    Mathf.FloorToInt(point.y / voxelSize),
                    Mathf.FloorToInt(point.z / voxelSize)
                );

                // Increment the count for this voxel
                // if (!InsideOfVoxel(voxelPosition, point, voxelSize)) {
                if (!voxelCounts.ContainsKey(voxelPosition))
                {
                    voxelCounts[voxelPosition] = 1;
                }
                else
                {
                    voxelCounts[voxelPosition]++;
                }
                // }
            }
        }

        // Iterate over voxel counts and create shapes where necessary
        foreach (var kvp in voxelCounts)
        {
            Vector3Int voxelPosition = kvp.Key;
            int pointCount = kvp.Value;

            Vector3 voxelMin = new Vector3(
                    (voxelPosition.x) * voxelSize,
                    (voxelPosition.y) * voxelSize,
                    (voxelPosition.z) * voxelSize
                );
            Vector3 voxelMax = new Vector3(
                    (voxelPosition.x + 1) * voxelSize,
                    (voxelPosition.y + 1) * voxelSize,
                    (voxelPosition.z + 1) * voxelSize
                );

            Vector3[] points = CreateNewBox(voxelMin, voxelMax);

            if (pointCount >= MIN_POINTS_FOR_SHAPE)
            {
                // Calculate the center of the voxel
                // Vector3 voxelCenter = new Vector3(
                //     (voxelPosition.x + 0.5f) * voxelSize,
                //     (voxelPosition.y + 0.5f) * voxelSize,
                //     (voxelPosition.z + 0.5f) * voxelSize
                // );
                if (!voxelRef.ContainsKey(voxelPosition)) {            
                    // You can now create a shape (e.g., box) at voxelCenter
                    // Example: CreateNewBox(voxelCenter, ...);
                    SetPointsReference(key_count, points);
                    PlaneNormalReference[key_count] = new Vector3(0, 0, -1);
                    GameObject gOref = CreateGameObjs(key_count, points);
                    key_count++;
                    voxelRef[voxelPosition] = gOref;
                }
            } else {
                // if (VerticesReference.ContainsValue(points)) {            
                //     // You can now create a shape (e.g., box) at voxelCenter
                //     // Example: CreateNewBox(voxelCenter, ...);
                //     // VerticesReference.Remove(VerticesReference.);
                //     // PlaneNormalReference[key_count] = new Vector3(0, 0, -1);
                //     // CreateGameObjs(key_count, points);
                //     // key_count++;
                // }
            }
        }
    }

    // use the voxels

    private void HelperHelper()
    {
        float MaxVal = 3;
        float MinVal = 0.0f;
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 point = vertices[i];
            if (point != Vector3.zero &&
                Math.Abs(point.x) < MaxVal && Math.Abs(point.y) < MaxVal && Math.Abs(point.z) < MaxVal)
                // Math.Abs(point.x) < MaxVal && Math.Abs(point.y) < MaxVal && Math.Abs(point.z) < MaxVal &&
                // Math.Abs(point.x) > MinVal && Math.Abs(point.y) > MinVal && Math.Abs(point.z) > MinVal)
            {
                int key_to_update = -2; // -2 means do not do anything
                if (usePointDecider(point, ref key_to_update)) { // update set
                    UpdateSet(key_to_update, point);

                } else if (key_to_update == -1) { // create a new set
                    Vector3[] points = CreateNewBox(point, point);
                    SetPointsReference(key_count, points);
                    PlaneNormalReference[key_count] = new Vector3(0, 0, -1);
                    CreateGameObjs(key_count, points);
                    key_count++;
                }
            }
        }
    }

    public bool usePointDecider(Vector3 point, ref int boxToUpdateKey) {
        boxToUpdateKey = -2;
        if (VerticesReference.Keys.Count == 0)
        {
            boxToUpdateKey = -1;
            return false;
        }
        float minDist = float.MaxValue;
        int minBox = 0;

        foreach (int i in VerticesReference.Keys)
        {
            if (InsideOfBox(GetPointsReference(i), point)) { return false; }
            else
            {
                float dist = ClosestPointDistance(point, GetPointsReference(i));
                if (dist <= minDist)
                {
                    minDist = dist;
                    minBox = i;
                }
            }
        }
        if (minDist <= BOX_DIST_THRESHOLD)
        {
            if (DistanceToPlane(minBox, point) < PLANE_DIST_THRESHOLD) {
                boxToUpdateKey = minBox;
                return true;
            } 
            // else {
            //     boxToUpdateKey = -1;
            // }
        }
        return false;
    }

    private float DistanceToPlane(int key, Vector3 point)
    {
        Vector3 planeNormal = PlaneNormalReference[key];
        Vector3[] points = VerticesReference[key];
        Vector3 pointToPlane = point - points[6];
        float dotProduct = Vector3.Dot(pointToPlane, planeNormal);
        return Mathf.Abs(dotProduct) / planeNormal.magnitude;
    }

    private float ClosestPointDistance(Vector3 p, Vector3[] bpx)
    {
        float minDist = float.MaxValue;
        foreach (Vector3 vertex in bpx)
        {
            float dist = Vector3.Distance(vertex, p);
            minDist = Mathf.Min(dist, minDist);
        }
        return minDist;
    }

    private bool InsideOfBox(Vector3[] box, Vector3 p)
    {
        Vector3 min = box[0];
        Vector3 max = box[6];
        return !(
            Math.Abs(p.x) < Math.Abs(min.x) || 
            Math.Abs(p.x) > Math.Abs(max.x) || 
            Math.Abs(p.y) < Math.Abs(min.y) || 
            Math.Abs(p.y) > Math.Abs(max.y) || 
            Math.Abs(p.z) < Math.Abs(min.z) || 
            Math.Abs(p.z) > Math.Abs(max.z)
        );
    }

    // private bool InsideOfVoxel(Vector3Int v, Vector3 p, float s)
    // {
    //     Vector3 min = new Vector3(
    //                 (v.x) * s,
    //                 (v.y) * s,
    //                 (v.z) * s
    //             );
    //     Vector3 max = new Vector3(
    //                 (v.x + 0.5f) * s,
    //                 (v.y + 0.5f) * s,
    //                 (v.z + 1) * s
    //             );
    //     return !(
    //         Math.Abs(p.x) < Math.Abs(min.x) || 
    //         Math.Abs(p.x) > Math.Abs(max.x) || 
    //         Math.Abs(p.y) < Math.Abs(min.y) || 
    //         Math.Abs(p.y) > Math.Abs(max.y) || 
    //         Math.Abs(p.z) < Math.Abs(min.z) || 
    //         Math.Abs(p.z) > Math.Abs(max.z)
    //     );
    // }
    bool InsideOfVoxel(Vector3Int voxelPosition, Vector3 point, float voxelSize)
    {
        Vector3 minBound = new Vector3(voxelPosition.x * voxelSize, voxelPosition.y * voxelSize, voxelPosition.z * voxelSize);
        Vector3 maxBound = new Vector3((voxelPosition.x + 1) * voxelSize, (voxelPosition.y + 1) * voxelSize, (voxelPosition.z + 1) * voxelSize);

        return point.x >= minBound.x && point.x <= maxBound.x &&
            point.y >= minBound.y && point.y <= maxBound.y &&
            point.z >= minBound.z && point.z <= maxBound.z;
    }

    private void UpdateSet(int key, Vector3 p)
    {
        Vector3[] points = VerticesReference[key];
        if (points == null)
            return;

        Vector3 min = points[0];
        Vector3 max = points[6];

        if (p.x < min.x || p.x > max.x || p.y < min.y || p.y > max.y || p.z < min.z || p.z > max.z)
        {
            min = Vector3.Min(min, p);
            max = Vector3.Max(max, p);
            Vector3[] newBox = CreateNewBox(max, min);

            // if (Math.Abs(newBox[6].z - newBox[0].z) >= 0.1)
            // {
            //     max.z = points[6].z;
            //     min.z = points[0].z;
            //     RotationMethod(key, newBox, GetGameObjectReference(key), p);
            // }
            // else
            // {
                UpdateMesh(newBox, GetGameObjectReference(key));
                SetPointsReference(key, newBox);
                UpdatePlane(key, Quaternion.identity);
            // }
        }
    }

    private void RotationMethod(int key, Vector3[] points, GameObject obj, Vector3 p)
    {
        Vector3[] OG = GetPointsReference(key);
        // If the distance between the plane and the point is too big do nothing
        // Vector3 minPoint = points[0];
        // Vector3 maxPoint = points[0];

        // foreach (Vector3 point in points)
        // {
        //     minPoint = Vector3.Min(minPoint, point);
        //     maxPoint = Vector3.Max(maxPoint, point);
        // we first update the plane based on points beyond threshold
        // if the box is too big we apply a rotation and make it slimer
        // }
        Vector3 minPoint = OG[0];
        Vector3 maxPoint = OG[6];

        // Get the parent GameObject of the childObject
        GameObject parentObject = obj.transform.parent.gameObject;

        Vector3 center = boxCenter(OG);
        // Vector3 direction = maxPoint - minPoint;
        Vector3 direction = p - center;

        // Quaternion rotation = Quaternion.LookRotation(direction, Vector3.up);

        Quaternion rotation = Quaternion.LookRotation(new Vector3(0.1f, 0.1f, 0), Vector3.up);

        // position - center after rot
        parentObject.transform.rotation = rotation;

        UpdatePlane(key, rotation);
    }

    private void UpdatePlane(int key, Quaternion q)
    {
        Vector3[] points = VerticesReference[key];
        if (q == Quaternion.identity)
        {
            float width = Mathf.Abs(points[6].x - points[0].x);
            float height = Mathf.Abs(points[6].y - points[0].y);
            float depth = Mathf.Abs(points[6].z - points[0].z);

            if (width > height && width > depth)
                PlaneNormalReference[key] = new Vector3(1, 0, 0);
            else if (depth > width)
                PlaneNormalReference[key] = new Vector3(0, 0, 1);
            else
                PlaneNormalReference[key] = new Vector3(0, 1, 0);
        }
        else
        {
            Vector3 normal = PlaneNormalReference[key];
            Quaternion normalQuaternion = new Quaternion(normal.x, normal.y, normal.z, 0f);
            Quaternion rotatedNormalQuaternion = q * normalQuaternion;
            Vector3 rotatedNormal = new Vector3(rotatedNormalQuaternion.x, rotatedNormalQuaternion.y, rotatedNormalQuaternion.z);
            PlaneNormalReference[key] = rotatedNormal;
        }
    }

    private Vector3[] CreateNewBox(Vector3 max, Vector3 min)
    {
        Vector3[] points = new Vector3[8];
        points[0] = min;
        points[1] = new Vector3(max.x, min.y, min.z);
        points[2] = new Vector3(max.x, min.y, max.z);
        points[3] = new Vector3(min.x, min.y, max.z);
        points[4] = new Vector3(min.x, max.y, min.z);
        points[5] = new Vector3(max.x, max.y, min.z);
        points[6] = max;
        points[7] = new Vector3(min.x, max.y, max.z);
        return points;
    }

    private GameObject CreateGameObjs(int key, Vector3[] points)
    {
        GameObject newGameObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
        Mesh mesh = new Mesh();
        mesh.vertices = points;
        mesh.triangles = cubeTriangles;
        newGameObj.GetComponent<MeshFilter>().mesh = mesh;
        GameObjReference[key] = newGameObj;
        // Create a new material with the desired color
        Material boxMaterial = new Material(Shader.Find("Standard"));
        // boxMaterial.color = Color.blue; // Change the color to whatever you want
        newGameObj.GetComponent<MeshRenderer>().material = hullMaterial;
        // Create parent GameObject for rotation
        GameObject parentObject = new GameObject("ParentObject");
        // Translate its position
        parentObject.transform.position = boxCenter(points);
        // Set the parent of childObject to parentObject
        newGameObj.transform.SetParent(parentObject.transform);
        return newGameObj;
    }

    public void UpdateMesh(Vector3[] vertices, GameObject gameObject) 
    {
        MeshFilter meshFilter = gameObject.GetComponent<MeshFilter>();
        if (meshFilter == null) { UnityEngine.Debug.LogError("MeshFilter component not found on the GameObject."); return; }

        Mesh mesh = meshFilter.mesh;

        mesh.Clear();
        mesh.vertices = vertices;
        mesh.triangles = cubeTriangles;
        mesh.RecalculateNormals();
    }

    private void Initialize()
    {
        PlaneNormalReference.Clear();
        VerticesReference.Clear();
        GameObjReference.Clear();
    }

    // Method to set the GameObj for a given key
    public void SetGameObjectReference(int key, GameObject value)
    {
        if (GameObjReference.ContainsKey(key)) { 
            // Key already exists, update the value 
            GameObjReference[key] = value;
        }
        else
        {   // Key doesn't exist, add a new entry
            GameObjReference.Add(key, value);
        }
    }
    // Method to get the gameObject reference for a given key
    public GameObject GetGameObjectReference(int key)
    {
        GameObject gamObj = null;
        GameObjReference.TryGetValue(key, out gamObj);
        return gamObj;
    }
    // Method to set the points array  for a given key
    public void SetPointsReference(int key, Vector3[] value)
    {
        if (VerticesReference.ContainsKey(key)) { 
            // Key already exists, update the value 
            VerticesReference[key] = value;
            // Store the current position of the child relative to the parent
            GameObject thisOne = GetGameObjectReference(key);
            Vector3 childLocalPosition = thisOne.transform.position;
            // Update the position of the parent without moving the child
            GameObject parentObject = thisOne.transform.parent.gameObject;
            parentObject.transform.position = boxCenter(value);
            // Reapply the offset to the child's position to maintain its position relative to the parent
            thisOne.transform.position = childLocalPosition;
        }
        else
        {   // Key doesn't exist, add a new entry
            VerticesReference.Add(key, value);
        }
    }
    // Method to get the points array reference for a given key
    public Vector3[] GetPointsReference(int key)
    {
        Vector3[] points = null;
        VerticesReference.TryGetValue(key, out points);
        return points;
    }

    public void SetPlaneNormalReference(int key, Vector3 normal)
    {
        if (PlaneNormalReference.ContainsKey(key)) { 
            // Key already exists, update the value 
            PlaneNormalReference[key] = normal;
        }
        else
        {   // Key doesn't exist, add a new entry
            PlaneNormalReference.Add(key, normal);
        }
    }
    // Method to get the normal reference for a given key
    public Vector3 GetPlaneNormalReference(int key)
    {
        Vector3 normal;
        PlaneNormalReference.TryGetValue(key, out normal);
        return normal;
    }

    public Vector3 boxCenter(Vector3[] box) {
        return (box[0] + box[6]) * 0.5f;
    }
}
