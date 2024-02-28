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
// using Common.Unity.Drawing;
// using CGALDotNetUnity.Hulls;


[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class PointCloudToMesh : MonoBehaviour
{
    // public RsFrameProvider Source;
    private int reductionValue = 300; // amount by which the dummy point cloud data is reduced.
    
    private Mesh mesh;
    private Texture2D uvmap;
    public Material vertexMaterial;
    public Material edgeMaterial;
    private GameObject delaunayGO;
    private List<GameObject> m_spheres = new List<GameObject>();
    private List<GameObject> m_edges = new List<GameObject>();
    public Material hullMaterial;
    private GameObject m_hull;
    [NonSerialized]
    private Vector3[] vertices;
    
    // private DelaunayTriangulation3<EEK> delaunay;
    private DelaunayTriangulation3<EEK> delaunay;
    // FrameQueue q;
    void Start()
    {
        // Source.OnStart += OnStartStreaming;
        // Source.OnStop += Dispose;
        InitializeMesh();   
        // Generate dummy point cloud data
        GenerateDummyPointCloud();
        ConvexHullMethod();
        // RemoveInside();
        // GenerateAlphaShape();
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
    }
    protected void LateUpdate()
    {
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
            float x = UnityEngine.Random.Range(-2f, 2f);
            float y = UnityEngine.Random.Range(-2f, 2f);
            float z = UnityEngine.Random.Range(0f, 4f);
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
        // m_hull = poly.ToUnityMesh("Hull", hullMaterial);
        // vertices[] = points.ToUnityVector3();
        // SurfaceMesh3<K> convexHull = CreateHullAsSurfaceMesh(points, points.Length);
    }
    private void RemoveInside()
    {
        //Step 1. Find the vertex with the smallest x coordinate
        //If several have the same x coordinate, find the one with the smallest z
        Vector3[] convexHull = new Vector3[8];
        Vector3 startPos = vertices[0];
        float smallestMagnitude = float.MaxValue;
        Vector3 endPos = vertices[0];
        float LargestMagnitude = float.MinValue;
        for (int i = 0; i < vertices.Length; i++)
        {
            Vector3 testPos = vertices[i];
            float magnitude = testPos.x + testPos.y + testPos.z;
            //Because of precision issues, we use Mathf.Approximately to test if the x positions are the same
            if (magnitude < smallestMagnitude) //testPos.x < startPos.x && testPos.y < startPos.y && testPos.z < startPos.z
            {
                smallestMagnitude = magnitude;
                startPos = vertices[i];
            }
            if (magnitude > LargestMagnitude) //testPos.x > endPos.x && testPos.y > endPos.y && testPos.z > endPos.z
            {
                LargestMagnitude = magnitude;
                endPos = vertices[i];
            }
        }
        convexHull[0] = startPos;
        convexHull[1] = endPos;
        convexHull[2] = new Vector3(startPos.x, startPos.y, endPos.z);
        convexHull[3] = new Vector3(endPos.x, endPos.y, startPos.z);
        convexHull[4] = new Vector3(startPos.x, endPos.y, startPos.z);
        convexHull[5] = new Vector3(endPos.x, startPos.y, endPos.z);
        convexHull[6] = new Vector3(endPos.x, startPos.y, startPos.z);
        convexHull[7] = new Vector3(startPos.x, endPos.y, endPos.z);
        
        // Create cube mesh
        Mesh mesh_cube = new Mesh(){
            indexFormat = IndexFormat.UInt32,
        };
        mesh_cube.name = "Cube Mesh";
        var indices = new int[convexHull.Length];
        for (int i = 0; i < convexHull.Length; i++)
            indices[i] = i;
        
        mesh_cube.vertices = convexHull;
        mesh_cube.SetIndices(indices, MeshTopology.Points, 0, false);
        mesh_cube.bounds = new Bounds(Vector3.zero, Vector3.one * 10f);
        
        MeshFilter meshFilter = GameObject.Find("ResultingPoints").GetComponent<MeshFilter>();
        
        meshFilter.mesh = mesh_cube;
        int[] triangles = new int[]
        {
            // Front Face
            4, 3, 0,
            0, 3, 6,
            // // Right Face
            3, 1, 6,
            5, 6, 1,
            // // Top Face
            3, 4, 1,
            7, 1, 4,
            // // Bottom Face
            5, 0, 6,
            5, 2, 0,
            // // Left Face
            4, 0, 7,
            0, 2, 7,
            // // Back Face
            2, 5, 1,
            7, 2, 1
        };
        mesh_cube.triangles = triangles;
    }
    void GenerateAlphaShape()
    {
        // DelaunayTriangulation3<EEK> dt = new DelaunayTriangulation3<EEK>();
    
        // for (int i = 0; i < vertices.Length; i++)
        // {
        //     Point3d p = new Point3d(vertices[i][0],vertices[i][1],vertices[i][2]);
        //     dt.Insert(p);
        // }
        // Create Delaunay triangulation object\
        var points = new Point3d[vertices.Length];
        for (int i = 0; i < vertices.Length; i++)
            points[i] = new Point3d(vertices[i].x, vertices[i].y, vertices[i].z);
        // Add points to the triangulation
        delaunay = new DelaunayTriangulation3<EEK>(points);
        CreateTriangulation();
        
        
        // Compute alpha shape
        // AlphaShape3<DelaunayTriangulation3<EEK>> a_s = new AlphaShape3<DelaunayTriangulation3<EEK>>(dt);
            
        // Console.WriteLine("Alpha shape computed in REGULARIZED mode by default.");
        // // Find optimal alpha values  
        // AlphaShape3<DelaunayTriangulation3<EEK>>.NT alphaSolid = a_s.FindAlphaSolid();
            
        // IEnumerable<AlphaShape3<DelaunayTriangulation3<EEK>>.NT> opt = a_s.FindOptimalAlpha(1);
            
        // Console.WriteLine("Smallest alpha value to get a solid through data points is " + alphaSolid);
        // Console.WriteLine("Optimal alpha value to get one connected component is " + opt.First());
        
        // a_s.SetAlpha(opt.First());
        
        // Assert.IsTrue(a_s.NumberOfSolidComponents() == 1); 
        // // var mesh = polygon.ToUnityMesh("polygon", position, material);
        // // Get alpha shape vertices
        // List<Point3d> verticesAS = new List<Point3d>();
        // foreach(Vertex_handle vh in a_s.Alpha_shape_vertices()) {
        //     verticesAS.Add(vh.Point());
        // }
        // // Get alpha shape triangles
        // List<int> trianglesAS = new List<int>();
        // foreach(Facet_handle fh in a_s.Alpha_shape_facets()) {
        //     foreach(int i in fh.Facet_indices()) {
        //         trianglesAS.Add(i); 
        //     }
        // }
        // mesh.vertices = verticesAS;
        // mesh.triangles = trianglesAS;
        
        GetComponent<MeshFilter>().sharedMesh = mesh;
        // Compute Delaunay triangulation (or use existing library)
        // For simplicity, let's assume the point cloud is already triangulated
        // Compute alpha shape edges
        // List<Edge> alphaEdges = ComputeAlphaShapeEdges();
        // // Convert edges to mesh vertices and triangles
        // List<Vector3> vertices = new List<Vector3>();
        // List<int> triangles = new List<int>();
        // foreach (var edge in alphaEdges)
        // {
        //     int startIndex = vertices.Count;
        //     vertices.Add(edge.Start);
        //     vertices.Add(edge.End);
        //     triangles.Add(startIndex);
        //     triangles.Add(startIndex + 1);
        // }
        // // Create mesh
        // mesh = new Mesh();
        // mesh.vertices = vertices.ToArray();
        // mesh.triangles = triangles.ToArray();
        // mesh.RecalculateNormals();
        // // Assign mesh to mesh filter
        // GetComponent<MeshFilter>().mesh = mesh;
    }
    private void DestroyObjects()
        {
            Destroy(delaunayGO);
            // Destroy(m_hull);
            foreach (var sphere in m_spheres)
                Destroy(sphere);
            m_spheres.Clear();
            foreach (var edge in m_edges)
                Destroy(edge);
            m_edges.Clear();
        }
    private void CreateTriangulation()
        {
            DestroyObjects();
            var points = new Point3d[delaunay.VertexCount];
            delaunay.GetPoints(points, points.Length);
            points.Round(2);
            var verts = new TriVertex3[delaunay.VertexCount];
            delaunay.GetVertices(verts, verts.Length);
            verts.Round(2);
            var segments = new Segment3d[delaunay.EdgeCount];
            delaunay.GetSegments(segments, segments.Length);
            Debug.Log("Segments = " + segments.Length);
            //var triangles = new int[delaunay.TriangleCount * 3];
            //delaunay.GetTriangleIndices(triangles, triangles.Length);
            //var tetrahedrons = new int[delaunay.TetrahedronCount * 4];
            //delaunay.GetTetrahedronIndices(tetrahedrons, tetrahedrons.Length);
            //var cells = new TriCell3[delaunay.TetrahedronCount];
            //delaunay.GetCells(cells, cells.Length);
            //var segments2 = segments.RemoveDuplicateSegments();
            //var triangles2 = triangles.RemoveDuplicateTriangles();
            delaunayGO = new GameObject("Triangulation");
            if (vertexMaterial != null)
            {
                for(int i = 0; i < verts.Length; i++)
                {
                    var v = verts[i];
                    if (v.IsInfinite) continue;
                    var sphere = GameObject.CreatePrimitive(PrimitiveType.Sphere);
                    sphere.transform.parent = delaunayGO.transform;
                    var renderer = sphere.GetComponent<Renderer>();
                    if (renderer != null)
                        renderer.sharedMaterial = vertexMaterial;
                    sphere.transform.position = ToVector3(v.Point);
                    sphere.transform.localScale = new Vector3(0.5f, 0.5f, 0.5f);
                    sphere.layer = LayerMask.NameToLayer("Geometry");
                    sphere.name = v.Index.ToString();
                    m_spheres.Add(sphere);
                }
            }
            if (edgeMaterial != null)
            {
                int edges = 0;
                for (int i = 0; i < segments.Length; i++)
                {
                    var seg = segments[i];
                    var a = seg.A;
                    var b = seg.B;
                    //Debug.Log("Create edge " + A + " " + B);
                    if (!a.IsFinite)
                    {
                        //Debug.Log("a is not finite");
                        //Debug.Log(a);
                        continue;
                    }
                    if (!b.IsFinite)
                    {
                        //Debug.Log("b is not finite");
                        //Debug.Log(b);
                        continue;
                    }
                    CreateCylinderBetweenPoints(ToVector3(a), ToVector3(b), 0.1f);
                    edges++;
                    //Debug.Log("Create edge " + a + " " + b);
                }
                //Debug.Log("Created " + edges + " edges)");
                /*
                for (int i = 0; i < segments2.Count; i++)
                {
                    var s = segments2[i];
                    if (s.A < 0 || s.A >= points.Length) continue;
                    if (s.B < 0 || s.B >= points.Length) continue;
                    var a = points[s.A];
                    var b = points[s.B];
        
                    if (!a.IsFinite || !b.IsFinite) continue;
                    CreateCylinderBetweenPoints(ToVector3(a), ToVector3(b), 0.1f);
                }
                */
                /*
                for (int i = 0; i < segments.Length/ 2; i++)
                {
                    int A = segments[i * 2 + 0];
                    int B = segments[i * 2 + 1];
                    if (A < 0 || A >= points.Length) continue;
                    if (B < 0 || B >= points.Length) continue;
                    var a = points[A];
                    var b = points[B];
                    if (!a.IsFinite || !b.IsFinite) continue;
                    CreateCylinderBetweenPoints(ToVector3(a), ToVector3(b), 0.1f);
                }
                /*
                for (int i = 0; i < triangles2.Count; i++)
                {
                    var t = triangles2[i];
                    //if (t.A < 0 || t.A >= points.Length) continue;
                    //if (t.B < 0 || t.B >= points.Length) continue;
                    //if (t.C < 0 || t.C >= points.Length) continue;
                    var a = points[t.A];
                    var b = points[t.B];
                    var c = points[t.C];
                    if (!a.IsFinite || !b.IsFinite || !c.IsFinite) continue;
                    //CreateCylinderBetweenPoints(ToVector3(a), ToVector3(b), 0.1f);
                    //CreateCylinderBetweenPoints(ToVector3(a), ToVector3(c), 0.1f);
                    //CreateCylinderBetweenPoints(ToVector3(c), ToVector3(b), 0.1f);
                }
                */
            }
        }
    private void CreateCylinderBetweenPoints(Vector3 start, Vector3 end, float width)
        {
            var offset = end - start;
            var scale = new Vector3(width, offset.magnitude / 2.0f, width);
            var position = start + (offset / 2.0f);
            var cylinder = GameObject.CreatePrimitive(PrimitiveType.Cylinder);
            cylinder.transform.parent = delaunayGO.transform;
            var renderer = cylinder.GetComponent<Renderer>();
            if (renderer != null)
                renderer.sharedMaterial = edgeMaterial;
            cylinder.transform.position = position;
            cylinder.transform.up = offset;
            cylinder.transform.localScale = scale;
            Debug.Log(scale);
            m_edges.Add(cylinder);
        }
    private void Update()
        {
            if(Input.GetMouseButtonDown(0))
            {
                RaycastHit hit;
                Ray ray = GetComponent<Camera>().ScreenPointToRay(Input.mousePosition);
                if (Physics.Raycast(ray, out hit))
                {
                    Transform objectHit = hit.transform;
                    int i = int.Parse(objectHit.name);
                    Debug.Log(i);
                    if (delaunay.GetVertex(i, out TriVertex3 vert))
                    {
                        Debug.Log(vert);
                    }
                    
                }
            }
            
        }
    private Vector3 ToVector3(Point3d point)
        {
            return new Vector3((float)point.x, (float)point.y, (float)point.z);
        }
}