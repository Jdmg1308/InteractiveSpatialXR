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
    private Dictionary<int, Vector3[]> VerticesReference;
    private Dictionary<int, Vector3> PlaneNormalReference;
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
    // private float totalSum;
    // private float minSum = float.MaxValue;
    // public float maxSum = float.MinValue;

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
        float MaxVal = 2;
        float MinVal = 0.1f;
        for (int i = 0; i < vertices.Length; i++)
        {
            // UnityEngine.Debug.Log(vertices[i]);
            if (vertices[i] != new Vector3(0, 0, 0) && (Math.Abs(vertices[i].x) < MaxVal && Math.Abs(vertices[i].y) < MaxVal && Math.Abs(vertices[i].z) < MaxVal) && (Math.Abs(vertices[i].x) > MinVal && Math.Abs(vertices[i].y) > MinVal && Math.Abs(vertices[i].z) > MinVal)) {
                bool outside = false;
                if (!TryAddPoint(vertices[i], ref outside)) {
                    if (outside) {
                        // could not add point, if outside create new set.
                        Vector3[] points = createNewBox(vertices[i], vertices[i]);
                        SetPointsReference(key, points);
                        SetPlaneNormalReference(key, new Vector3(0, 0, -1));
                        CreateGameObjs(key, points);
                        key++;
                    }
                }
            }
        }
    }

    private void CreateGameObjs(int key, Vector3[] points) {
        // create gameobj
        GameObject newGameObj = Instantiate(MeshGameObj);
        // create mesh
        Mesh New_mesh = new Mesh();
        
        New_mesh.Clear();

        New_mesh.vertices = points;
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
        DIST_THRESHOLD = 0.5f;
        MeshGameObj = GameObject.Find("Hull");
        GameObjReference = new Dictionary<int, GameObject>();
        VerticesReference = new Dictionary<int, Vector3[]>();
        PlaneNormalReference = new Dictionary<int, Vector3>();
        first = true;
    }
    
    bool TryAddPoint(Vector3 point, ref bool outside) {
        if (VerticesReference.Keys.Count == 0) {
            outside = true;
            return false;
        }
        float min_dist = float.MaxValue;
        int minBox = 0;
        // find closest box and distance
        foreach (int i in VerticesReference.Keys) {
            // check if already inside a bounding box
            if (insideOfBox(GetPointsReference(i), point)) {
                outside = false;
                return false;
            } else {
                // get closest distance to a vertex in the mesh
                float dist = closestPointDistance(point, GetPointsReference(i));
                if (dist <= min_dist) {
                    min_dist = dist;
                    minBox = i;
                }
            }
        }
        outside = true;
        // if within threshold and angle then update closest set
        if (minBox == 0) {
            Vector3[] points = GetPointsReference(minBox);
            // UnityEngine.Debug.Log("box with max [" + points[6] + "] and min [" + points[0] + "] p: ["  + point + "]");
            UnityEngine.Debug.DrawLine(points[6], points[0], Color.red);
            UnityEngine.Debug.Log(points[6] == points[0]);
            // UnityEngine.Debug.DrawLine(points[6].ToUnityVector3(), point.ToUnityVector3(), Color.blue);
            // UnityEngine.Debug.DrawLine(points[0].ToUnityVector3(), point.ToUnityVector3(), Color.green);
        }
        if (min_dist <= DIST_THRESHOLD) {
            if (distanceToPlane(minBox, point) < DIST_THRESHOLD) {
                updateSet(minBox, point);
                return true;
            }
        }
        return false;
    }
    float distanceToPlane(int key, Vector3 point) {
        Vector3 planeNormal = GetPlaneNormalReference(key);
        Vector3[] points = GetPointsReference(key);
        // Calculate the vector from the point on the plane to the point to check
        Vector3 pointToPlane = point - points[6];
        // Calculate the dot product of the vector and the plane's normal
        float dotProduct = Vector3.Dot(pointToPlane, planeNormal);
        // Calculate the distance using the formula
        float distance = Mathf.Abs(dotProduct) / planeNormal.magnitude;
        return distance;

    }
    float closestPointDistance(Vector3 p, Vector3[] bpx) {
        // to get closest mesh get closest point
        float min_dist = float.MaxValue;
        for (int i = 0; i < bpx.Length; i++) {
            float d1 = Math.Abs((bpx[i] - p).magnitude); //////////////////////////////////////////////////////////////////////////////////////
            min_dist = Math.Min(d1, min_dist);
        }
        return min_dist; 
    }
    
    Vector3[] createNewBox(Vector3 max, Vector3 min) {
        // we need at least 4 valid points
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
    bool insideOfBox (Vector3[] box, Vector3 p) {
        float XMin = box[0].x;
        float XMax = box[6].x;
        float YMin = box[0].y;
        float YMax = box[6].y;
        float ZMin = box[0].z;
        float ZMax = box[6].z;
        return !(p.x < XMin || p.x > XMax || p.y < YMin || p.y > YMax || p.z < ZMin || p.z > ZMax);
    }
    public void updateSet(int key, Vector3 p) {
        // update the mesh
        Vector3[] points = GetPointsReference(key);
        GameObject gameObj = GetGameObjectReference(key);
        
        if (gameObj == null) { UnityEngine.Debug.LogError("gameObj component not found." + key); return; }
        if (points == null) { UnityEngine.Debug.LogError("Points array not found." + key); return; }

        // Vector3 curr_max = points[6];
        // Vector3 curr_min = points[0];

        float XMin = points[0].x;
        float XMax = points[6].x;

        float YMin = points[0].y;
        float YMax = points[6].y;

        float ZMin = points[0].z;
        float ZMax = points[6].z;
        
        // rethink this 
        // Vector3 new_max = new Vector3(Math.Max(p.x, curr_max.x), Math.Max(p.y, curr_max.y), Math.Max(p.z, curr_max.z));
        // Vector3 new_min = new Vector3(Math.Min(p.x, curr_min.x), Math.Min(p.y, curr_min.y), Math.Min(p.z, curr_min.z));

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
        
        Vector3 new_max = new Vector3(XMax, YMax, ZMax);
        Vector3 new_min = new Vector3(XMin, YMin, ZMin);

        // float currentSurfaceArea = BoundingBoxSurfaceArea(curr_max, curr_min);
        // Vector3[] newBox = points;
        // if (BoundingBoxSurfaceArea(p, curr_min) > currentSurfaceArea) {
        //     newBox = createNewBox(p, curr_min);
        // } else if (BoundingBoxSurfaceArea(curr_max, p) > currentSurfaceArea) {
        //     newBox = createNewBox(curr_max, p);
        // }
        Vector3[] newBox = createNewBox(new_max, new_min);
        if (newBox[6].z - newBox[0].z > 0.5) {
            RotationMethod(key, newBox, gameObj);
            // update plane
        } else {  
            UpdateMesh(newBox, gameObj);
            SetPointsReference(key, newBox);
            // update plane
            UpdatePlane(key, new Quaternion(0, 0, 0, 0));
        }
    }

    public void UpdatePlane(int key, Quaternion q) {
        Vector3[] points = GetPointsReference(key);
        if (q.x == 0 && q.y == 0 && q.z == 0 && q.w == 0) {
            float width = Mathf.Abs((float)(points[6].x - points[0].x));
            float height = Mathf.Abs((float) (points[6].y - points[0].y));
            float depth = Mathf.Abs((float)(points[6].z - points[0].z));
            if (width > height && width > depth) {
                //width
                SetPlaneNormalReference(key, new Vector3(1, 0, 0));
            } else if (depth > width) {
                // depth
                SetPlaneNormalReference(key, new Vector3(0, 0, 1));
            } else {
                // height
                SetPlaneNormalReference(key, new Vector3(0, 1, 0));
            }
        } else {
            Vector3 normal = GetPlaneNormalReference(key);
            // Convert the normal to a quaternion with a w value of 0
            Quaternion normalQuaternion = new Quaternion(normal.x, normal.y, normal.z, 0f);
            // Rotate the normal quaternion by the rotation quaternion
            Quaternion rotatedNormalQuaternion = q * normalQuaternion;
            // Convert the rotated normal quaternion back to a vector
            Vector3 rotatedNormal = new Vector3(rotatedNormalQuaternion.x, rotatedNormalQuaternion.y, rotatedNormalQuaternion.z);
            SetPlaneNormalReference(key, rotatedNormal);
        }
    }
    void RotationMethod(int key, Vector3[] points, GameObject obj) {
        // Calculate extents
        Vector3 minPoint = points[0];
        Vector3 maxPoint = points[0];
        foreach (Vector3 point in points)
        {
            minPoint = Vector3.Min(minPoint, point);
            maxPoint = Vector3.Max(maxPoint, point);
        }

        // Calculate center
        Vector3 center = (minPoint + maxPoint) * 0.5f;

        // Calculate rotation
        Vector3 direction = maxPoint - minPoint;
        Quaternion rotation = Quaternion.LookRotation(direction, Vector3.up);

        // Create the bounding box
        GameObject boundingBox = obj;
        boundingBox.transform.position = center;
        boundingBox.transform.rotation = rotation;

        // Scale the bounding box based on extents
        // Vector3 size = maxPoint - minPoint;
        // boundingBox.transform.localScale = size;

        // Optional: Make it thinner
        // float thickness = 0.1f; // Adjust as needed
        // boundingBox.transform.localScale = new Vector3(thickness, size.y, size.z);
        // SetPointsReference(key, points);
        UpdatePlane(key, rotation);
    }
    float BoundingBoxSurfaceArea(Vector3 p1, Vector3 p2) {
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
    

    // box has a certain tangent plane, if it grows beyond a certain limit then dont and instead rotate 
    //Find the Extents: Calculate the minimum and maximum points along each axis (x, y, and z) from your set of points. These points will define the corners of your bounding box.
    // Calculate Center: Compute the center point of the bounding box by averaging the minimum and maximum points along each axis.
    // Calculate Rotation: Determine the rotation of the bounding box. Since you want the bounding box to be aligned with the diagonal shape, you can calculate the rotation based on the direction of the diagonal. This can be achieved by finding the vector that connects the minimum and maximum points.
    // Create the Bounding Box: Using the calculated center and rotation, you can instantiate a bounding box GameObject in Unity and set its position and rotation accordingly.

    
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
    public void SetPointsReference(int key, Vector3[] value)
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
        {
            // Key doesn't exist, add a new entry
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

}

// reformat the code so you use vect3

