using System;
using UnityEngine;
using Intel.RealSense;
using UnityEngine.Rendering;
using UnityEngine.Assertions;
using System.Runtime.InteropServices;
using System.Threading;
using System.Collections.Generic;
using System.Linq;
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


[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class PointCloudToMesh : MonoBehaviour
{
    // public RsFrameProvider Source;
    private int reductionValue = 5000; // amount by which the dummy point cloud data is reduced.
    private Mesh mesh;
    public Material vertexMaterial;
    public Material edgeMaterial;
    public Material hullMaterial;
    private GameObject m_hull;

    [NonSerialized]
    private Vector3[] vertices;

    MeshCollider ShapeMeshCollider;
    void Start()
    {
        // Source.OnStart += OnStartStreaming;
        // Source.OnStop += Dispose;
        InitializeMesh();   
        // Generate dummy point cloud data

        Stopwatch sw = new Stopwatch();
        for(int index = 0; index < 10; index++)
        {
            sw = Stopwatch.StartNew();
            GenerateDummyPointCloud();
            ConvexHullMethod();
            UnityEngine.Debug.Log(sw.ElapsedMilliseconds);
        }
        sw.Stop();
    }
    private void InitializeMesh()
    {
        // Set up UV map texture
        int width = 640; // Set your desired width
        int height = 480; // Set your desired height
        // Create mesh
        mesh = new Mesh()
        {
            indexFormat = IndexFormat.UInt32,
        };
        vertices = new Vector3[(width * height) / reductionValue];
        var indices = new int[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
            indices[i] = i;
        // mesh.MarkDynamic();
        mesh.vertices = vertices;
        mesh.SetIndices(indices, MeshTopology.Points, 0, false);
        mesh.bounds = new Bounds(Vector3.zero, Vector3.one * 10f);
        GetComponent<MeshFilter>().sharedMesh = mesh;
        ShapeMeshCollider = GameObject.Find("Hull").GetComponent<MeshCollider>();
    }
    protected void LateUpdate()
    {
        GenerateDummyPointCloud();
        ConvexHullMethod();
        // if (q != null)
        // {
        //     Points points;
        //     if (q.PollForFrame<Points>(out points))
        //         using (points)
        //         {
        //             if (points.Count != mesh.vertexCount)
        //             {
        //                 using (var p = points.GetProfile<VideoStreamProfile>())
        //                     ResetMesh(p.Width, p.Height);
        //             }
        //             if (points.TextureData != IntPtr.Zero)
        //             {
        //                 uvmap.LoadRawTextureData(points.TextureData, points.Count * sizeof(float) * 2);
        //                 uvmap.Apply();
        //             }
        //             if (points.VertexData != IntPtr.Zero)
        //             {
        //                 points.CopyVertices(vertices);
        //                 mesh.vertices = vertices;
        //                 mesh.UploadMeshData(false);
        //             }
        //         }
        // }
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
            float x = UnityEngine.Random.Range(-20f, 20f);
            float y = UnityEngine.Random.Range(-20f, 20f);
            float z = UnityEngine.Random.Range(0f, 40f);
            vertices[i] = new Vector3(x, y, z);
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