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
    private const int MIN_POINTS_FOR_SHAPE = 50; // Adjust as needed
    public Material hullMaterial;
    private Mesh mesh;
    private Texture2D uvmap;
    private FrameQueue q;
    // Create a dictionary to store voxel counts
    private Dictionary<Vector3Int, int> voxelCounts = new Dictionary<Vector3Int, int>();
    // Create a dictionary to store voxel objects
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
        // voxel size
        float voxelSize = 0.13f;

        // Iterate over all points and count points in each voxel
        if (value % 3 == 0) {
            foreach (var point in vertices)
            {
                Vector3Int voxelPosition = new Vector3Int(
                    Mathf.FloorToInt(point.x / voxelSize),
                    Mathf.FloorToInt(point.y / voxelSize),
                    Mathf.FloorToInt(point.z / voxelSize)
                );

                if (!voxelCounts.ContainsKey(voxelPosition))
                {
                    voxelCounts[voxelPosition] = 1;
                }
                else
                {
                    voxelCounts[voxelPosition]++;
                }
            }
        }

        // Iterate over voxel counts and create/delete boxes where necessary
        foreach (var vox_pos_key in new List<Vector3Int>(voxelCounts.Keys))
        {
            if (voxelCounts[vox_pos_key] > 1) {
                voxelCounts[vox_pos_key] -= voxelCounts[vox_pos_key] / (MIN_POINTS_FOR_SHAPE / 10);
            } else { voxelCounts[vox_pos_key]++; }
            int pointCount = voxelCounts[vox_pos_key];

            if (pointCount >= MIN_POINTS_FOR_SHAPE)
            {
                Vector3 voxelMin = new Vector3(
                        vox_pos_key.x * voxelSize,
                        vox_pos_key.y * voxelSize,
                        vox_pos_key.z * voxelSize
                    );
                Vector3 voxelMax = new Vector3(
                        (vox_pos_key.x + 1) * voxelSize,
                        (vox_pos_key.y + 1) * voxelSize,
                        (vox_pos_key.z + 1) * voxelSize
                    );
                Vector3[] points = CreateNewBox(voxelMin, voxelMax);
                if (!voxelRef.ContainsKey(vox_pos_key)) {
                    voxelRef[vox_pos_key] = CreateGameObjs(points);
                }
            } else {
                if (voxelRef.ContainsKey(vox_pos_key)) {
                    Destroy(voxelRef[vox_pos_key]);
                    voxelRef.Remove(vox_pos_key);
                }
            }
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
    private GameObject CreateGameObjs(Vector3[] points)
    {
        GameObject newGameObj = GameObject.CreatePrimitive(PrimitiveType.Cube);
        Mesh mesh = new Mesh();
        mesh.vertices = points;
        mesh.triangles = cubeTriangles;
        // mesh.RecalculateNormals();
        newGameObj.GetComponent<MeshFilter>().mesh = mesh;
        newGameObj.GetComponent<MeshRenderer>().material = hullMaterial;
        Destroy(newGameObj.GetComponent<BoxCollider>());
        MeshCollider boxCollider = newGameObj.AddComponent<MeshCollider>();
        // boxCollider.convex = false;
        boxCollider.convex = true;
        return newGameObj;
    }
    private void Initialize()
    {
        voxelRef.Clear();
        voxelCounts.Clear();
    }
}
