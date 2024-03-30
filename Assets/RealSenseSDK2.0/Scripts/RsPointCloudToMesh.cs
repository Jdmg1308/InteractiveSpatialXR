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
    // Changes >
    public RsFrameProvider Source;
    private Mesh mesh;
    private Texture2D uvmap;

    [NonSerialized]
    private Vector3[] vertices;

    FrameQueue q;

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
                        // ConvexHullMethod();
                        Helper();
                        
                    }
                }
        }
    }
    // Changes <
    public void Helper() {
        int key = 0;
        if (first) { 
            SetPointsReference(key, createNewBox(new Point3d(1, 0.5, 4), new Point3d(0.5, 0.1, 3.5)));
            key++;
            CreateGameObjs();
            first = false;
        }
        // System.Random rand = new System.Random();
        // int ra = rand.Next(0, Objects.Count);
        // Point3d[] minSet = Objects[0];
        // ra = rand.Next(vertices.Length/2, vertices.Length);
        // addPoint(vertices[ra].ToCGALPoint3d());
        // for each point
        for (int i = 0; i < vertices.Length; i++)
        {
            // UnityEngine.Debug.Log(vertices[i]);
            if (vertices[i] != new Vector3(0, 0, 0)) {
                if (!TryAddPoint(vertices[i].ToCGALPoint3d())) {
                    // could not add point, create new set.
                    // SetPointsReference(key, createNewBox(new Point3d(0.2, 0.1, 4), new Point3d(0.1, 0.0, 4)));
                    // key++;
                }
            }
        }
        // for each one that has not been instantiated
        CreateGameObjs();
    }

    private void CreateGameObjs() {
        foreach (int i in VerticesReference.Keys) {
            // if new
            if (GetGameObjectReference(i) == null) {   
                Point3d[] points = GetPointsReference(i);
                // create temp gameobj
                temp = ConvexHull3<EEK>.Instance.CreateHullAsPolyhedron(points, points.Length).ToUnityMesh("Hull", hullMaterial);
                GameObject newGameObj = Instantiate(MeshGameObj);
                if (temp.GetComponent<MeshFilter>()) {       
                    // set mesh
                    newGameObj.GetComponent<MeshFilter>().mesh = temp.GetComponent<MeshFilter>().mesh;
                    // set collider
                    if (temp.GetComponent<MeshFilter>().mesh.vertices.Length >= 8) { 
                        MeshCollider ShapeMeshCollider = newGameObj.GetComponent<MeshCollider>();
                        ShapeMeshCollider.sharedMesh = temp.GetComponent<MeshFilter>().mesh;
                        ShapeMeshCollider.convex = false;
                        ShapeMeshCollider.convex = true;
                    }
                }
                // destroy temp gameobj
                Destroy(temp);
                // save gameObj
                SetGameObjectReference(i, newGameObj);
            }
        }
    }
    private void Initialize()
    {
        DIST_THRESHOLD = 0.3f;
        MeshGameObj = GameObject.Find("Hull");
        GameObjReference = new Dictionary<int, GameObject>();
        VerticesReference = new Dictionary<int, Point3d[]>();
        first = true;
    }
    
    bool TryAddPoint(Point3d point) {
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
        if (min_dist <= DIST_THRESHOLD) {
            updateSet(minBox, point);
            return true;
        }
        return false;
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
    int closestPointIndex(Point3d p, Point3d[] set) {
        // to get closest mesh get closest point
        int min_closest = 0;
        double min_dist = double.MaxValue;
        for (int i = 0; i < set.Length; i++) {
            double d1 = Math.Abs((set[i] - p).Magnitude);
            if (d1 <= min_dist) {
                min_dist = d1;
                min_closest = i;
            }
        }
        return min_closest; 
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
    bool insideOfBox (Point3d[] box, Point3d point) {
        return point.x < box[6].x && point.y < box[6].y && point.z < box[6].z && point.x > box[0].x && point.y > box[0].y && point.z > box[0].z;
    }
    public void updateSet(int key, Point3d p) {
        // update the mesh
        Point3d[] points = GetPointsReference(key);
        GameObject gameObj = GetGameObjectReference(key);
        
        if (gameObj == null) { UnityEngine.Debug.LogError("gameObj component not found." + key); return; }
        if (points == null) { UnityEngine.Debug.LogError("Points array not found." + key); return; }

        Point3d max = new Point3d(Math.Max(p.x, points[6].x), Math.Max(p.y, points[6].y), Math.Max(p.z, points[6].z));
        Point3d min = new Point3d(Math.Min(p.x, points[0].x), Math.Min(p.y, points[0].y), Math.Min(p.z, points[0].z));

        UpdateMesh(createNewBox(max, min).ToUnityVector3(), gameObj);
    }
    void UpdateMesh(Vector3[] vertices, GameObject gameObject) 
    {
        MeshFilter meshFilter = gameObject.GetComponent<MeshFilter>();
        MeshCollider meshCollider = gameObject.GetComponent<MeshCollider>();

        if (meshFilter == null) { UnityEngine.Debug.LogError("MeshFilter component not found on the GameObject."); return; }
        if (meshCollider == null) { UnityEngine.Debug.LogError("MeshCollider component not found on the GameObject."); return; }

        Mesh mesh = meshFilter.mesh;

        mesh.Clear();
        mesh.vertices = vertices;
        // mesh.triangles = triangles;
        mesh.RecalculateNormals();

        // Update the collider
        meshCollider.sharedMesh = null; // Clear previous mesh
        meshCollider.sharedMesh = mesh; // Assign the updated mesh to the collider
    }
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

