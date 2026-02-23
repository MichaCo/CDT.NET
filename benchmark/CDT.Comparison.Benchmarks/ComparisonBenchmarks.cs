// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Configs;
using CDT;
using CdtEdge = CDT.Edge;
using NtsCoordinate = NetTopologySuite.Geometries.Coordinate;
using NtsGeometryFactory = NetTopologySuite.Geometries.GeometryFactory;
using NtsLineString = NetTopologySuite.Geometries.LineString;
using NtsMultiLineString = NetTopologySuite.Geometries.MultiLineString;
using NtsMultiPoint = NetTopologySuite.Geometries.MultiPoint;
using NtsPoint = NetTopologySuite.Geometries.Point;
using TnMesher = TriangleNet.Meshing.GenericMesher;
using TnPolygon = TriangleNet.Geometry.Polygon;
using TnSegment = TriangleNet.Geometry.Segment;
using TnVertex = TriangleNet.Geometry.Vertex;

// ---------------------------------------------------------------------------
// Shared input reader
// Reads the same .txt files used by CDT.Tests / CDT.Benchmarks.
// Format: nVerts nEdges\n  x y\n…  v1 v2\n…
// ---------------------------------------------------------------------------
internal static class InputReader
{
    public static (double[] Xs, double[] Ys, int[] EdgeV1, int[] EdgeV2) Read(string fileName)
    {
        var path = Path.Combine(AppContext.BaseDirectory, "inputs", fileName);
        if (!File.Exists(path))
            throw new FileNotFoundException($"Benchmark input not found: {fileName}");

        using var sr = new StreamReader(path);
        var header = sr.ReadLine()!.Trim().Split(' ', StringSplitOptions.RemoveEmptyEntries);
        int nVerts = int.Parse(header[0]);
        int nEdges = int.Parse(header[1]);

        var xs = new double[nVerts];
        var ys = new double[nVerts];
        for (int i = 0; i < nVerts; i++)
        {
            var tok = sr.ReadLine()!.Trim().Split(' ', StringSplitOptions.RemoveEmptyEntries);
            xs[i] = double.Parse(tok[0], System.Globalization.CultureInfo.InvariantCulture);
            ys[i] = double.Parse(tok[1], System.Globalization.CultureInfo.InvariantCulture);
        }

        var ev1 = new int[nEdges];
        var ev2 = new int[nEdges];
        for (int i = 0; i < nEdges; i++)
        {
            var tok = sr.ReadLine()!.Trim().Split(' ', StringSplitOptions.RemoveEmptyEntries);
            ev1[i] = int.Parse(tok[0]);
            ev2[i] = int.Parse(tok[1]);
        }

        return (xs, ys, ev1, ev2);
    }
}

// ---------------------------------------------------------------------------
// Adapter — CDT.NET (baseline)
// ---------------------------------------------------------------------------
internal static class CdtNetAdapter
{
    public static int VerticesOnly(double[] xs, double[] ys)
    {
        var verts = new List<V2d<double>>(xs.Length);
        for (int i = 0; i < xs.Length; i++)
            verts.Add(new V2d<double>(xs[i], ys[i]));

        var cdt = new Triangulation<double>(VertexInsertionOrder.Auto);
        cdt.InsertVertices(verts);
        return cdt.Triangles.Length;
    }

    public static int Constrained(double[] xs, double[] ys, int[] ev1, int[] ev2)
    {
        var verts = new List<V2d<double>>(xs.Length);
        for (int i = 0; i < xs.Length; i++)
            verts.Add(new V2d<double>(xs[i], ys[i]));

        var edges = new List<CdtEdge>(ev1.Length);
        for (int i = 0; i < ev1.Length; i++)
            edges.Add(new CdtEdge(ev1[i], ev2[i]));

        var cdt = new Triangulation<double>(VertexInsertionOrder.Auto);
        cdt.InsertVertices(verts);
        cdt.InsertEdges(edges);
        return cdt.Triangles.Length;
    }
}

// ---------------------------------------------------------------------------
// Adapter — Triangle.NET  (Unofficial.Triangle.NET 0.0.1)
// True CDT: segments become hard constraint edges in the mesh.
// ---------------------------------------------------------------------------
internal static class TriangleNetAdapter
{
    public static int VerticesOnly(double[] xs, double[] ys)
    {
        var polygon = new TnPolygon(xs.Length);
        for (int i = 0; i < xs.Length; i++)
            polygon.Add(new TnVertex(xs[i], ys[i]));

        return new TnMesher().Triangulate(polygon).Triangles.Count;
    }

    public static int Constrained(double[] xs, double[] ys, int[] ev1, int[] ev2)
    {
        var polygon = new TnPolygon(xs.Length);
        var verts = new TnVertex[xs.Length];
        for (int i = 0; i < xs.Length; i++)
        {
            verts[i] = new TnVertex(xs[i], ys[i]);
            polygon.Add(verts[i]);
        }
        for (int i = 0; i < ev1.Length; i++)
            polygon.Add(new TnSegment(verts[ev1[i]], verts[ev2[i]]));

        return new TnMesher().Triangulate(polygon).Triangles.Count;
    }
}

// ---------------------------------------------------------------------------
// Adapter — NetTopologySuite  (2.6.0)
// Conforming CDT: constraint edges are honoured but Steiner points may be
// inserted to satisfy the Delaunay criterion (results differ from true CDT).
// VerticesOnly uses the plain DelaunayTriangulationBuilder.
// ---------------------------------------------------------------------------
internal static class NtsAdapter
{
    private static readonly NtsGeometryFactory Gf = new();

    public static int VerticesOnly(double[] xs, double[] ys)
    {
        var coords = new NtsCoordinate[xs.Length];
        for (int i = 0; i < xs.Length; i++)
            coords[i] = new NtsCoordinate(xs[i], ys[i]);

        var builder = new NetTopologySuite.Triangulate.DelaunayTriangulationBuilder();
        builder.SetSites(coords);
        return builder.GetTriangles(Gf).NumGeometries;
    }

    public static int Conforming(double[] xs, double[] ys, int[] ev1, int[] ev2)
    {
        var pts = new NtsPoint[xs.Length];
        for (int i = 0; i < xs.Length; i++)
            pts[i] = Gf.CreatePoint(new NtsCoordinate(xs[i], ys[i]));

        var segments = new NtsLineString[ev1.Length];
        for (int i = 0; i < ev1.Length; i++)
            segments[i] = Gf.CreateLineString(new[]
            {
                new NtsCoordinate(xs[ev1[i]], ys[ev1[i]]),
                new NtsCoordinate(xs[ev2[i]], ys[ev2[i]]),
            });

        var builder = new NetTopologySuite.Triangulate.ConformingDelaunayTriangulationBuilder();
        builder.SetSites(new NtsMultiPoint(pts));
        builder.Constraints = new NtsMultiLineString(segments);
        return builder.GetTriangles(Gf).NumGeometries;
    }
}


// (~2 600 vertices, ~2 600 constraint edges)
// ---------------------------------------------------------------------------
[MemoryDiagnoser]
[GroupBenchmarksBy(BenchmarkLogicalGroupRule.ByCategory)]
[CategoriesColumn]
[ShortRunJob]
public class ComparisonBenchmarks
{
    private double[] _xs = null!;
    private double[] _ys = null!;
    private int[] _ev1 = null!;
    private int[] _ev2 = null!;

    [GlobalSetup]
    public void Setup() =>
        (_xs, _ys, _ev1, _ev2) = InputReader.Read("Constrained Sweden.txt");

    // -- VerticesOnly --------------------------------------------------------

    [Benchmark(Baseline = true, Description = "CDT.NET")]
    [BenchmarkCategory("VerticesOnly")]
    public int VO_CdtNet() => CdtNetAdapter.VerticesOnly(_xs, _ys);

    [Benchmark(Description = "Triangle.NET")]
    [BenchmarkCategory("VerticesOnly")]
    public int VO_TriangleNet() => TriangleNetAdapter.VerticesOnly(_xs, _ys);

    [Benchmark(Description = "NTS")]
    [BenchmarkCategory("VerticesOnly")]
    public int VO_Nts() => NtsAdapter.VerticesOnly(_xs, _ys);

    // -- Constrained ---------------------------------------------------------

    [Benchmark(Baseline = true, Description = "CDT.NET")]
    [BenchmarkCategory("Constrained")]
    public int CDT_CdtNet() => CdtNetAdapter.Constrained(_xs, _ys, _ev1, _ev2);

    [Benchmark(Description = "Triangle.NET")]
    [BenchmarkCategory("Constrained")]
    public int CDT_TriangleNet() => TriangleNetAdapter.Constrained(_xs, _ys, _ev1, _ev2);

    [Benchmark(Description = "NTS (Conforming CDT)")]
    [BenchmarkCategory("Constrained")]
    public int CDT_Nts() => NtsAdapter.Conforming(_xs, _ys, _ev1, _ev2);
}
