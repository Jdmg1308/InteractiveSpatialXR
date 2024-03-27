using System;
using UnityEngine;
using Intel.RealSense;
using UnityEngine.Rendering;
using UnityEngine.Assertions;
using System.Runtime.InteropServices;
using System.Threading;
using System.Collections.Generic;
using System.Linq;
using CGALDotNet.Processing;
using CGALDotNet;
using CGALDotNet.Triangulations;
using CGALDotNetGeometry.Numerics;
using CGALDotNetGeometry.Shapes;
using CGALDotNet.Polyhedra;
using CGALDotNet.Extensions;
using CGALDotNetGeometry.Extensions;
using CGALDotNet.Hulls;
using System.Diagnostics;
using JetBrains.Annotations;
using System.ComponentModel;
using static CGALDotNet.CGALGlobal;


[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class PointCloudToMesh : MonoBehaviour
{
    // public RsFrameProvider Source;
    private int numberOfVertices = 3000;
    private int width = 100;
    private int height = 100;
    private Mesh mesh;
    public Material vertexMaterial;
    public Material edgeMaterial;
    public Material hullMaterial;
    private GameObject m_hull;
    public Stopwatch sw;

    public double totalSum;
    public double maxSum;
    public double minSum;
    public int totalTimes;

    [NonSerialized]
    private Vector3[] vertices;

    MeshCollider ShapeMeshCollider;
    void Start()
    {
        // Source.OnStart += OnStartStreaming;
        // Source.OnStop += Dispose;
        InitializeMesh();   
        // Generate dummy point cloud data
        totalSum = 0;
        minSum = double.MaxValue;
        maxSum = double.MinValue;
        totalSum = 0;
        totalTimes = 0;
        sw = new Stopwatch();
    }
    private void InitializeMesh()
    {
        // Set up UV map texture
        // Create mesh
        mesh = new Mesh()
        {
            indexFormat = IndexFormat.UInt32,
        };
        vertices = new Vector3[numberOfVertices];
        var indices = new int[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
            indices[i] = i;
        // mesh.MarkDynamic();
        mesh.vertices = vertices;
        mesh.SetIndices(indices, MeshTopology.Points, 0, false);
        mesh.bounds = new Bounds(Vector3.zero, Vector3.one * 10f);
        GetComponent<MeshFilter>().sharedMesh = mesh;
        ShapeMeshCollider = GameObject.Find("Hull").GetComponent<MeshCollider>();
        ShapeMeshCollider.sharedMesh = mesh;
    }
    protected void LateUpdate()
    {
        sw = Stopwatch.StartNew();
        GenerateDummyPointCloud();
        ConvexHullMethod();
        sw.Stop();
        // Get the elapsed time
        TimeSpan elapsedTime = sw.Elapsed;

        // Print the time elapsed
        // UnityEngine.Debug.Log("Time elapsed: " + elapsedTime.TotalMilliseconds);

        // totalSum += elapsedTime.TotalMilliseconds;
        // totalTimes++;

        // minSum = minSum < elapsedTime.TotalMilliseconds ? minSum : elapsedTime.TotalMilliseconds;
        // maxSum = maxSum > elapsedTime.TotalMilliseconds ? maxSum : elapsedTime.TotalMilliseconds;

        // UnityEngine.Debug.Log("Average Time elapsed: " + totalSum/totalTimes + " min Time elapsed: " + minSum + " max Time elapsed: " + maxSum);
        
        // UnityEngine.Debug.Log("min Time elapsed: " + minSum);
        // UnityEngine.Debug.Log("max Time elapsed: " + maxSum);

    }
    private void GenerateDummyPointCloud()
    {
        // Check if mesh and vertices are initialized
        if (mesh == null || vertices == null)
            return;
        // Generate random point cloud data
        for (int i = 0; i < vertices.Length; i++)
        {
            // Generate random points within a specific range
            // float x = UnityEngine.Random.Range(-20f, 20f);
            // float y = UnityEngine.Random.Range(-20f, 20f);
            // float z = UnityEngine.Random.Range(0f, 40f);
            
            float x = UnityEngine.Random.Range(0, width);
            float y = UnityEngine.Random.Range(0, width);
            float z = UnityEngine.Random.Range(0, height);
            vertices[i] = new Vector3(x, y, z);
            // Vector3 newPoint = new Vector3(x, y, z);
            // if (!pointInMesh(newPoint)) {
            //     vertices[i] = newPoint;
            // }
        }
        // Update mesh vertices
        mesh.vertices = vertices;
        mesh.UploadMeshData(false);
    }
    
    private void ConvexHullMethod()
    {
        Point3d[] points = new Point3d[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
        {
            var v = vertices[i];
            points[i] = new Point3d(v.x, v.y, v.z);
        }
        var poly = ConvexHull3<EEK>.Instance.CreateHullAsPolyhedron(points, points.Length);
        m_hull = poly.ToUnityMesh("Hull", hullMaterial);
        MeshFilter hullmeshFilter = GameObject.Find("Hull").GetComponent<MeshFilter>();
        hullmeshFilter.mesh = m_hull.GetComponent<MeshFilter>().mesh;
        ShapeMeshCollider.sharedMesh = m_hull.GetComponent<MeshFilter>().mesh;
        Destroy(m_hull);
        ShapeMeshCollider.convex = false;
        ShapeMeshCollider.convex = true;
    }

    private bool pointInMesh(Vector3 point)
    {
        // if points are colliding with a plane then do not add them
        // go through every plane if they collide do not add them
        int[] triangles = ShapeMeshCollider.sharedMesh.triangles;

        for (int i = 0; i < triangles.Length; i += 3)
        {
            // Get the vertex indices for this triangle
            int index1 = triangles[i];
            int index2 = triangles[i + 1];
            int index3 = triangles[i + 2];

            // Get the actual vertices of the triangle
            Vector3 vertex1 = vertices[index1];
            Vector3 vertex2 = vertices[index2];
            Vector3 vertex3 = vertices[index3];

            // Now you have the vertices of the triangle, you can do whatever you need with them
            // For example, you can create a Triangle object or perform computations
            if (IsPointInTriangleCGAL(point, vertex1, vertex2, vertex3)) {
                return true;
            }
        }

        return false;
        
    }

    public bool IsPointInTriangleCGAL(Vector3 pt, Vector3 v1, Vector3 v2, Vector3 v3)
    {
        var pt3d = new Point3d(pt.x, pt.y, pt.z);
        var v13d = new Point3d(v1.x, v1.y, v1.z);
        var v23d = new Point3d(v2.x, v2.y, v2.z);
        var v33d = new Point3d(v3.x, v3.y, v3.z);
        //Coplanar(pt3d, v13d, v23d, v33d)
        if (true) {
            Vector3 AB = v2 - v1;
            Vector3 AC = v3 - v1;
            Vector3 N = Vector3.Cross(AB, AC);  // Normal vector of the triangle
            N.Normalize();

            Vector3 AP = pt - v1;
            Vector3 BP = pt - v2;
            Vector3 CP = pt - v3;

            float dotAPN = Vector3.Dot(AP, N);
            float dotBPN = Vector3.Dot(BP, N);
            float dotCPN = Vector3.Dot(CP, N);

            if ((dotAPN > 0 && dotBPN > 0 && dotCPN > 0) || (dotAPN < 0 && dotBPN < 0 && dotCPN < 0) && Coplanar(pt3d, v13d, v23d, v33d)){
                return true;  // Point is inside the triangle
            } else {
                return false;
            }
        } 
    }

    // public bool IsPointInTriangle(Vector3 pt, Vector3 v1, Vector3 v2, Vector3 v3)
    // {
    //     // Compute vectors
    //     Vector3 v0 = v3 - v1;
    //     Vector3 v2v1 = v2 - v1;
    //     Vector3 vpt = pt - v1;

    //     // Compute dot products
    //     float dot00 = Vector3.Dot(v0, v0);
    //     float dot01 = Vector3.Dot(v0, v2v1);
    //     float dot02 = Vector3.Dot(v0, vpt);
    //     float dot11 = Vector3.Dot(v2v1, v2v1);
    //     float dot12 = Vector3.Dot(v2v1, vpt);

    //     // Compute barycentric coordinates
    //     float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
    //     float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
    //     float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

    //     // Check if point is in triangle
    //     return (u >= 0) && (v >= 0) && (u + v < 1);
    // }

    // private void RemoveInside()
    // {
    //     //Step 1. Find the vertex with the smallest x coordinate
    //     //If several have the same x coordinate, find the one with the smallest z
    //     Vector3[] convexHull = new Vector3[8];
    //     Vector3 startPos = vertices[0];
    //     float smallestMagnitude = float.MaxValue;
    //     Vector3 endPos = vertices[0];
    //     float LargestMagnitude = float.MinValue;
    //     for (int i = 0; i < vertices.Length; i++)
    //     {
    //         Vector3 testPos = vertices[i];
    //         float magnitude = testPos.x + testPos.y + testPos.z;
    //         //Because of precision issues, we use Mathf.Approximately to test if the x positions are the same
    //         if (magnitude < smallestMagnitude) //testPos.x < startPos.x && testPos.y < startPos.y && testPos.z < startPos.z
    //         {
    //             smallestMagnitude = magnitude;
    //             startPos = vertices[i];
    //         }
    //         if (magnitude > LargestMagnitude) //testPos.x > endPos.x && testPos.y > endPos.y && testPos.z > endPos.z
    //         {
    //             LargestMagnitude = magnitude;
    //             endPos = vertices[i];
    //         }
    //     }
    //     convexHull[0] = startPos;
    //     convexHull[1] = endPos;
    //     convexHull[2] = new Vector3(startPos.x, startPos.y, endPos.z);
    //     convexHull[3] = new Vector3(endPos.x, endPos.y, startPos.z);
    //     convexHull[4] = new Vector3(startPos.x, endPos.y, startPos.z);
    //     convexHull[5] = new Vector3(endPos.x, startPos.y, endPos.z);
    //     convexHull[6] = new Vector3(endPos.x, startPos.y, startPos.z);
    //     convexHull[7] = new Vector3(startPos.x, endPos.y, endPos.z);
        
    //     // Create cube mesh
    //     Mesh mesh_cube = new Mesh(){
    //         indexFormat = IndexFormat.UInt32,
    //     };
    //     mesh_cube.name = "Cube Mesh";
    //     var indices = new int[convexHull.Length];
    //     for (int i = 0; i < convexHull.Length; i++)
    //         indices[i] = i;
        
    //     mesh_cube.vertices = convexHull;
    //     mesh_cube.SetIndices(indices, MeshTopology.Points, 0, false);
    //     mesh_cube.bounds = new Bounds(Vector3.zero, Vector3.one * 10f);
        
    //     MeshFilter meshFilter = GameObject.Find("ResultingPoints").GetComponent<MeshFilter>();
        
    //     meshFilter.mesh = mesh_cube;
    //     int[] triangles = new int[]
    //     {
    //         // Front Face
    //         4, 3, 0,
    //         0, 3, 6,
    //         // // Right Face
    //         3, 1, 6,
    //         5, 6, 1,
    //         // // Top Face
    //         3, 4, 1,
    //         7, 1, 4,
    //         // // Bottom Face
    //         5, 0, 6,
    //         5, 2, 0,
    //         // // Left Face
    //         4, 0, 7,
    //         0, 2, 7,
    //         // // Back Face
    //         2, 5, 1,
    //         7, 2, 1
    //     };
    //     mesh_cube.triangles = triangles;
    // }
    // void GenerateAlphaShape()
    // {
    //     var points = new Point3d[vertices.Length];
    //     for (int i = 0; i < vertices.Length; i++)
    //         points[i] = new Point3d(vertices[i].x, vertices[i].y, vertices[i].z);
    //     // Add points to the triangulation
    //     delaunay = new DelaunayTriangulation3<EEK>(points);
    //     CreateTriangulation();
        
        
    //     GetComponent<MeshFilter>().sharedMesh = mesh;
    // }
    // private void DestroyObjects()
    //     {
    //         Destroy(delaunayGO);
    //         Destroy(m_hull);
    //         foreach (var sphere in m_spheres)
    //             Destroy(sphere);
    //         m_spheres.Clear();
    //         foreach (var edge in m_edges)
    //             Destroy(edge);
    //         m_edges.Clear();
    //     }

    private void Update()
    {
        // if(Input.GetMouseButtonDown(0))
        // {
        //     RaycastHit hit;
        //     Ray ray = GetComponent<Camera>().ScreenPointToRay(Input.mousePosition);
        //     if (Physics.Raycast(ray, out hit))
        //     {
        //         Transform objectHit = hit.transform;
        //         int i = int.Parse(objectHit.name);
        //         Debug.Log(i);
        //         if (delaunay.GetVertex(i, out TriVertex3 vert))
        //         {
        //             Debug.Log(vert);
        //         }       
        //     }
        // }            
    }
}