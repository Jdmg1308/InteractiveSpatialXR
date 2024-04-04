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

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class RsPointCloudToMesh : MonoBehaviour
{
    // Changes <
    private float DIST_THRESHOLD;
    public Material hullMaterial;
    private GameObject temp;
    private GameObject MeshGameObj;
    private bool first;
    private Dictionary<int, GameObject> GameObjReference;
    private Dictionary<int, Point3d[]> VerticesReference;
    public int[] cube_triangles = new int[] 
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
    // Changes >
    public RsFrameProvider Source;
    private Mesh mesh;
    private Texture2D uvmap;

    [NonSerialized]
    private Vector3[] vertices;

    FrameQueue q;
    // private int totalTimes;
    // private double totalSum;
    // private double minSum = double.MaxValue;
    // public double maxSum = double.MinValue;

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
                        Helper();
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
    // Changes <
    public void Helper() {
        int key = 0;
        // if (first) { 
        //     Point3d[] firstPoints = createNewBox(vertices[300].ToCGALPoint3d(), vertices[300].ToCGALPoint3d());
        //     SetPointsReference(key, firstPoints);
        //     CreateGameObjs(key, firstPoints);
        //     key++;
        //     first = false;
        // }
        double MaxVal = 4;
        double MinVal = 0.3;
        for (int i = 0; i < vertices.Length; i++)
        {
            // UnityEngine.Debug.Log(vertices[i]);
            if (vertices[i] != new Vector3(0, 0, 0) && (Math.Abs(vertices[i].x) < MaxVal && Math.Abs(vertices[i].y) < MaxVal && Math.Abs(vertices[i].z) < MaxVal) && (Math.Abs(vertices[i].x) > MinVal && Math.Abs(vertices[i].y) > MinVal && Math.Abs(vertices[i].z) > MinVal)) {
                // if ((Math.Abs(vertices[i].x) < MaxVal && Math.Abs(vertices[i].y) < MaxVal && Math.Abs(vertices[i].z) < MaxVal) && (Math.Abs(vertices[i].x) > MinVal && Math.Abs(vertices[i].y) > MinVal && Math.Abs(vertices[i].z) > MinVal)) {
                if (!TryAddPoint(vertices[i].ToCGALPoint3d())) {
                    // could not add point, create new set.
                    Point3d[] points = createNewBox(vertices[i].ToCGALPoint3d(), vertices[i].ToCGALPoint3d());
                    SetPointsReference(key, points);
                    CreateGameObjs(key, points);
                    key++;
                }
            }
        }
    }

    private void CreateGameObjs(int key, Point3d[] points) {
        // create gameobj
        GameObject newGameObj = Instantiate(MeshGameObj);
        // create mesh
        Mesh New_mesh = new Mesh();
        
        New_mesh.Clear();

        New_mesh.vertices = points.ToUnityVector3();
        New_mesh.triangles = cube_triangles;

        // mesh.RecalculateNormals();
        
        if (newGameObj.GetComponent<MeshFilter>()) {       
            // set mesh
            newGameObj.GetComponent<MeshFilter>().mesh = New_mesh;
            // set collider
            // if (newGameObj.GetComponent<MeshFilter>().mesh.vertices.Length >= 8) { 
            //     MeshCollider ShapeMeshCollider = newGameObj.GetComponent<MeshCollider>();
            //     ShapeMeshCollider.sharedMesh = New_mesh;
            //     ShapeMeshCollider.convex = false;
            //     ShapeMeshCollider.convex = true;
            // }
        }
        // save gameObj
        SetGameObjectReference(key, newGameObj);
    }

    private void Initialize()
    {
        DIST_THRESHOLD = 1;
        MeshGameObj = GameObject.Find("Hull");
        GameObjReference = new Dictionary<int, GameObject>();
        VerticesReference = new Dictionary<int, Point3d[]>();
        first = true;
    }
    
    bool TryAddPoint(Point3d point) {
        if (VerticesReference.Keys.Count == 0) {
            return false;
        }
        double min_dist = double.MaxValue;
        int minBox = 0;
        // find closest box and distance
        foreach (int i in VerticesReference.Keys) {
            // check if already inside a bounding box
            if (insideOfBox(GetPointsReference(i), point)) {
                return false;
            } else {
                // get closest distance to a vertex in the mesh
                
                double dist = closestPointDistance(point, GetPointsReference(i));
                if (dist <= min_dist) {
                    min_dist = dist;
                    minBox = i;
                }
            }
        }
        // if within threshold then update closest set
        updateSet(minBox, point);
        // if (min_dist <= DIST_THRESHOLD) {
        //     updateSet(minBox, point);
        //     return true;
        // }
        return true;
    }
    double closestPointDistance(Point3d p, Point3d[] bpx) {
        // to get closest mesh get closest point
        double min_dist = double.MaxValue;
        for (int i = 0; i < bpx.Length; i++) {
            double d1 = Math.Abs((bpx[i] - p).Magnitude); //////////////////////////////////////////////////////////////////////////////////////
            min_dist = Math.Min(d1, min_dist);
        }
        return min_dist; 
    }
    
    Point3d[] createNewBox(Point3d max, Point3d min) {
        // we need at least 4 valid points
        Point3d[] points = new Point3d[8];
        points[0] = min;
        points[1] = new Point3d(max.x, min.y, min.z);
        points[2] = new Point3d(max.x, min.y, max.z);
        points[3] = new Point3d(min.x, min.y, max.z);
        points[4] = new Point3d(min.x, max.y, min.z);
        points[5] = new Point3d(max.x, max.y, min.z);
        points[6] = max;
        points[7] = new Point3d(min.x, max.y, max.z);
        return points;
    }
    bool insideOfBox (Point3d[] box, Point3d p) {
        
        double XMin = box[0].x;
        double XMax = box[6].x;

        double YMin = box[0].y;
        double YMax = box[6].y;

        double ZMin = box[0].z;
        double ZMax = box[6].z;
        // return Math.Abs(point.x) < Math.Abs(box[6].x) && Math.Abs(point.y) < Math.Abs(box[6].y) && Math.Abs(point.z) < Math.Abs(box[6].z)
        // && Math.Abs(point.x) > Math.Abs(box[0].x) && Math.Abs(point.y) > Math.Abs(box[0].y) && Math.Abs(point.z) > Math.Abs(box[0].z);
        return !(p.x < XMin || p.x > XMax || p.y < YMin || p.y > YMax || p.z < ZMin || p.z > ZMax);
    }
    public void updateSet(int key, Point3d p) {
        // update the mesh
        Point3d[] points = GetPointsReference(key);
        GameObject gameObj = GetGameObjectReference(key);
        
        if (gameObj == null) { UnityEngine.Debug.LogError("gameObj component not found." + key); return; }
        if (points == null) { UnityEngine.Debug.LogError("Points array not found." + key); return; }

        // Point3d curr_max = points[6];
        // Point3d curr_min = points[0];

        double XMin = points[0].x;
        double XMax = points[6].x;

        double YMin = points[0].y;
        double YMax = points[6].y;

        double ZMin = points[0].z;
        double ZMax = points[6].z;
        
        // rethink this 
        // Point3d new_max = new Point3d(Math.Max(p.x, curr_max.x), Math.Max(p.y, curr_max.y), Math.Max(p.z, curr_max.z));
        // Point3d new_min = new Point3d(Math.Min(p.x, curr_min.x), Math.Min(p.y, curr_min.y), Math.Min(p.z, curr_min.z));


        if (p.x < XMin || p.x > XMax || p.y < YMin || p.y > YMax || p.z < ZMin || p.z > ZMax)
        {
            // Point is outside the current bounding box, adjust the dimensions
            XMin = Math.Min(XMin, p.x);
            YMin = Math.Min(YMin, p.y);
            ZMin = Math.Min(ZMin, p.z);
            XMax = Math.Max(XMax, p.x);
            YMax = Math.Max(YMax, p.y);
            ZMax = Math.Max(ZMax, p.z);
        }
        
        Point3d new_max = new Point3d(XMax, YMax, ZMax);
        Point3d new_min = new Point3d(XMin, YMin, ZMin);

        // float currentSurfaceArea = BoundingBoxSurfaceArea(curr_max, curr_min);
        // Point3d[] newBox = points;
        // if (BoundingBoxSurfaceArea(p, curr_min) > currentSurfaceArea) {
        //     newBox = createNewBox(p, curr_min);
        // } else if (BoundingBoxSurfaceArea(curr_max, p) > currentSurfaceArea) {
        //     newBox = createNewBox(curr_max, p);
        // }
        UpdateMesh(createNewBox(new_max, new_min).ToUnityVector3(), gameObj);
    }
    float BoundingBoxSurfaceArea(Point3d p1, Point3d p2) {
        // Calculate the absolute differences between the coordinates
        float width = Mathf.Abs((float)(p1.x - p2.x));
        float height = Mathf.Abs((float) (p1.y - p2.y));
        float depth = Mathf.Abs((float)(p1.z - p2.z));
        // Calculate the surface area
        return 2 * (width * height + width * depth + height * depth);
    }
    void UpdateMesh(Vector3[] vertices, GameObject gameObject) 
    {
        MeshFilter meshFilter = gameObject.GetComponent<MeshFilter>();
        // MeshCollider meshCollider = gameObject.GetComponent<MeshCollider>();

        if (meshFilter == null) { UnityEngine.Debug.LogError("MeshFilter component not found on the GameObject."); return; }
        // if (meshCollider == null) { UnityEngine.Debug.LogError("MeshCollider component not found on the GameObject."); return; }

        Mesh mesh = meshFilter.mesh;
        // int[] triValues = mesh.triangles;

        // UnityEngine.Debug.Log("vertex n" + mesh.vertices.Length);
        // UnityEngine.Debug.Log("triangles n" + triValues.Length);

        mesh.Clear();
        mesh.vertices = vertices;
        mesh.triangles = cube_triangles;
        mesh.RecalculateNormals();

        // Update the collider
        // meshCollider.sharedMesh = null; // Clear previous mesh
        // meshCollider.sharedMesh = mesh; // Assign the updated mesh to the collider
    }
    
    // calculate overall plane
    
    // public static Plane CalculateAveragePlane(Vector3[] points)
    // {
    //     if (points == null || points.Length < 3)
    //     {
    //         UnityEngine.Debug.LogError("Insufficient points to calculate average plane.");
    //         return new Plane();
    //     }

    //     // Calculate the centroid of the points
    //     Vector3 centroid = Vector3.zero;
    //     foreach (Vector3 point in points)
    //     {
    //         centroid += point;
    //     }
    //     centroid /= points.Length;

    //     // Compute the covariance matrix
    //     Matrix4x4 covarianceMatrix = new Matrix4x4();
    //     foreach (Vector3 point in points)
    //     {
    //         Vector3 deviation = point - centroid;
    //         covarianceMatrix.m00 += deviation.x * deviation.x;
    //         covarianceMatrix.m01 += deviation.x * deviation.y;
    //         covarianceMatrix.m02 += deviation.x * deviation.z;
    //         covarianceMatrix.m11 += deviation.y * deviation.y;
    //         covarianceMatrix.m12 += deviation.y * deviation.z;
    //         covarianceMatrix.m22 += deviation.z * deviation.z;
    //     }
    //     covarianceMatrix /= points.Length;

    //     // Compute the normal vector of the best-fit plane using PCA
    //     Vector3 normal = Vector3.zero;
    //     if (PCA(covarianceMatrix, ref normal))
    //     {
    //         return new Plane(normal, centroid);
    //     }
    //     else
    //     {
    //         UnityEngine.Debug.LogError("Failed to compute normal vector.");
    //         return new Plane();
    //     }
    // }

    // private static bool PCA(Matrix4x4 covarianceMatrix, ref Vector3 normal)
    // {
    //     // Perform eigenvalue decomposition to find the eigenvector corresponding to the smallest eigenvalue
    //     Matrix4x4 eigenVectors;
    //     if (!covarianceMatrix.Diagonalize(out eigenVectors))
    //     {
    //         return false;
    //     }

    //     // The eigenvector corresponding to the smallest eigenvalue is the normal of the plane
    //     normal = eigenVectors.GetColumn(2).normalized;

    //     return true;
    // }
    
    // Method to set the GameObj for a given key
    public void SetGameObjectReference(int key, GameObject value)
    {
        if (GameObjReference.ContainsKey(key)) { 
            // Key already exists, update the value 
            GameObjReference[key] = value;
        }
        else
        {
            // Key doesn't exist, add a new entry
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
    public void SetPointsReference(int key, Point3d[] value)
    {
        if (VerticesReference.ContainsKey(key)) { 
            // Key already exists, update the value 
            VerticesReference[key] = value;
        }
        else
        {
            // Key doesn't exist, add a new entry
            VerticesReference.Add(key, value);
        }
    }
    // Method to get the points array reference for a given key
    public Point3d[] GetPointsReference(int key)
    {
        Point3d[] points = null;
        VerticesReference.TryGetValue(key, out points);
        return points;
    }

}

