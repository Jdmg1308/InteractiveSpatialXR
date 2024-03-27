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

[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class RsPointCloudToMesh : MonoBehaviour
{
    // Changes <
    private float DIST_THRESHOLD;
    public Material hullMaterial;
    private GameObject temp;
    private GameObject MeshGameObj;
    private List<Point3d[]> Objects;
    // MeshCollider ShapeMeshCollider;

    // Dictionary to map keys (string in this example) to binary values (bool)
    private Dictionary<Point3d[], bool> rendered;
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
                        ConvexHullMethod();
                        
                    }
                }
        }
    }
    // Changes <
     private void Initialize()
    {
        DIST_THRESHOLD = 1f;
        // mesh = new Mesh();
        MeshGameObj = GameObject.Find("Hull");
        Objects = new List<Point3d[]>();
        rendered = new Dictionary<Point3d[], bool>();
        Objects.Add(createNewSet(new Point3d(0, 0, 0)));
    }
    private void ConvexHullMethod()
    {
        // potential algorithms

        // if angle between 3 points is larger than a set value then it becomes a different shape

        // take away all the error points, create different sets of points

        // for each set of points convex hull and add it to the scene

        // try to do this in one go O(N) (how? how does the lidar data work? randomly?) 

        // no need to process points inside of current mesh, you would have to check if points inside of a mesh that has already been created

        // k tree?
       
        for (int i = 0; i < vertices.Length; i++)
        {
            addPoint(new Point3d(vertices[i].x, vertices[i].y, vertices[i].z));
        }
        
        // for each one that has not been instantiated
        foreach (Point3d[] setOfPoints in Objects) {
            if (!rendered.ContainsKey(setOfPoints)) {
                UnityEngine.Debug.Log("Chicken Balls lmao 1");
                // create temp gameobj
                temp = ConvexHull3<EEK>.Instance.CreateHullAsPolyhedron(setOfPoints, setOfPoints.Length).ToUnityMesh("Hull", hullMaterial);
                // temp = polyhedron.ToUnityMesh("Hull", hullMaterial);
                GameObject newGameObj = Instantiate(MeshGameObj);
                
                if (temp.GetComponent<MeshFilter>()) {       
                    // set mesh
                    newGameObj.GetComponent<MeshFilter>().mesh = temp.GetComponent<MeshFilter>().mesh;
                    // set collider
                    if (temp.GetComponent<MeshFilter>().mesh.vertices.Length > 10) { 
                        MeshCollider ShapeMeshCollider = newGameObj.GetComponent<MeshCollider>();
                        ShapeMeshCollider.sharedMesh = temp.GetComponent<MeshFilter>().mesh;
                        ShapeMeshCollider.convex = false;
                        ShapeMeshCollider.convex = true;
                    }
                }
                // destroy temp gameobj
                Destroy(temp);
                SetRendered(setOfPoints, true);
            } else {
                if (!GetRendered(setOfPoints)) {
                    // false, therefore needs updating
                    // we simply need to update mesh right? but how
                    UnityEngine.Debug.Log("Chicken Balls lmao 2");
                    // create temp gameobj
                    temp = ConvexHull3<EEK>.Instance.CreateHullAsPolyhedron(setOfPoints, setOfPoints.Length).ToUnityMesh("Hull", hullMaterial);
                    // temp = polyhedron.ToUnityMesh("Hull", hullMaterial);
                    GameObject newGameObj = Instantiate(MeshGameObj);
                    
                    if (temp.GetComponent<MeshFilter>()) {       
                        // set mesh
                        newGameObj.GetComponent<MeshFilter>().mesh = temp.GetComponent<MeshFilter>().mesh;
                        // set collider
                        if (temp.GetComponent<MeshFilter>().mesh.vertices.Length > 10) { 
                            MeshCollider ShapeMeshCollider = newGameObj.GetComponent<MeshCollider>();
                            ShapeMeshCollider.sharedMesh = temp.GetComponent<MeshFilter>().mesh;
                            ShapeMeshCollider.convex = false;
                            ShapeMeshCollider.convex = true;
                        }
                    }
                    // destroy temp gameobj
                    Destroy(temp);
                    SetRendered(setOfPoints, true);
                }
            }
        }


    }

    void addPoint(Point3d point) {
        // store which set it is closest to.
        double min_dist = double.MaxValue;
        System.Random rand = new System.Random();
        int r = rand.Next(0, Objects.Count);
        Point3d[] minSet = Objects[r];
        r = rand.Next(0, 7);
        int min_closest = r;
        min_dist = rand.Next(0, 2);

        // for (int j = 0; j < Objects.Count; j++) {
            // if (!insideOfBox(Objects[j], point)) {
                // // it was not found, get closest set
                // int closest = closestPoint(point, set);
                // double dist = Math.Abs((set[closest] - point).Magnitude);
                // if (min_dist >= dist) {
                //     min_dist = dist;
                //     minSet = set;
                //     min_closest = closest;
                // }
            // }
        // }
        // Not found
        // if within threshold then update closest set
        if (min_dist <= DIST_THRESHOLD) {
            // update the mesh (minP)
            // replace closest Point
            minSet[min_closest] = point;
            SetRendered(minSet, false);
        } else {
            // else new set must be created 
            // create new object
            Objects.Add(createNewSet(point));
        }
    }

    Point3d[] createNewSet(Point3d p){
        // we need at least 4 valid points
        Point3d[] points = new Point3d[8];
        points[0] = p;
        points[1] = new Point3d(p.x + 0.05, p.y, p.z);
        points[2] = new Point3d(p.x + 0.05, p.y, p.z + 0.05);
        points[3] = new Point3d(p.x, p.y, p.z + 0.05);
        points[4] = new Point3d(p.x, p.y + 0.05, p.z);
        points[5] = new Point3d(p.x + 0.05, p.y + 0.05, p.z);
        points[6] = new Point3d(p.x + 0.05, p.y + 0.05, p.z + 0.05);
        points[7] = new Point3d(p.x, p.y + 0.05, p.z + 0.05);
        return points;
    }

    int closestPoint(Point3d p, Point3d[] set) {
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

    bool insideOfBox (Point3d[] box, Point3d point) {
        // if the point is between lower x higher x
        // if the point is between lower y and higer y
        // if the point is between lower y and higer y
        if (point.x >= box[0].x && point.x <= box[6].x &&
               point.y >= box[0].y && point.y <= box[6].y &&
               point.z >= box[0].z && point.z <= box[6].z) {
                return true;

        }
        return false;
    }

    // Method to set the binary value for a given key
    public void SetRendered(Point3d[] key, bool value)
    {
        if (rendered.ContainsKey(key))
        {
            // Key already exists, update the value
            rendered[key] = value;
        }
        else
        {
            // Key doesn't exist, add a new entry
            rendered.Add(key, value);
        }
    }

    // Method to get the binary value for a given key
    public bool GetRendered(Point3d[] key)
    {
        bool value = false;
        rendered.TryGetValue(key, out value);
        return value;
    }
    
    // Changes >
}
