// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Configs;
using BenchmarkDotNet.Diagnosers;
using BenchmarkDotNet.Running;
using CDT;

BenchmarkSwitcher.FromAssembly(typeof(BenchmarkInputReader).Assembly).Run(args);

// ---------------------------------------------------------------------------
// Helpers shared by all benchmarks
// ---------------------------------------------------------------------------
internal static class BenchmarkInputReader
{
    /// <summary>
    /// Reads a CDT input file.
    /// Format: <c>nVerts nEdges\n x y\n … v1 v2\n …</c>
    /// </summary>
    public static (List<V2d<double>> Vertices, List<Edge> Edges) Read(string fileName)
    {
        var path = Path.Combine(AppContext.BaseDirectory, "inputs", fileName);

        if (!File.Exists(path))
            throw new FileNotFoundException($"Benchmark input file not found: {fileName}");

        using var sr = new StreamReader(path);
        var header = sr.ReadLine()!.Trim().Split(' ', StringSplitOptions.RemoveEmptyEntries);
        int nVerts = int.Parse(header[0]);
        int nEdges = int.Parse(header[1]);

        var verts = new List<V2d<double>>(nVerts);
        for (int i = 0; i < nVerts; i++)
        {
            var tok = sr.ReadLine()!.Trim().Split(' ', StringSplitOptions.RemoveEmptyEntries);
            verts.Add(new V2d<double>(
                double.Parse(tok[0], System.Globalization.CultureInfo.InvariantCulture),
                double.Parse(tok[1], System.Globalization.CultureInfo.InvariantCulture)));
        }

        var edges = new List<Edge>(nEdges);
        for (int i = 0; i < nEdges; i++)
        {
            var tok = sr.ReadLine()!.Trim().Split(' ', StringSplitOptions.RemoveEmptyEntries);
            edges.Add(new Edge(int.Parse(tok[0]), int.Parse(tok[1])));
        }

        return (verts, edges);
    }
}

// ---------------------------------------------------------------------------
// Constrained Sweden – mirrors the C++ benchmark suite exactly
// ---------------------------------------------------------------------------

/// <summary>
/// Benchmarks using the "Constrained Sweden" dataset (~2 600 vertices,
/// ~2 600 constraint edges) – the same dataset used in the C++ CDT benchmarks.
/// </summary>
[MemoryDiagnoser]
[EventPipeProfiler(EventPipeProfile.CpuSampling)]
[GroupBenchmarksBy(BenchmarkLogicalGroupRule.ByCategory)]
[CategoriesColumn]
[ShortRunJob]
public class ConstrainedSwedenBenchmarks
{
    private List<V2d<double>> _vertices = null!;
    private List<Edge> _edges = null!;

    [GlobalSetup]
    public void Setup()
    {
        (_vertices, _edges) = BenchmarkInputReader.Read("Constrained Sweden.txt");
    }

    // -- Vertices only -------------------------------------------------------

    [Benchmark(Description = "Vertices only – AsProvided")]
    [BenchmarkCategory("VerticesOnly")]
    public Triangulation<double> VerticesOnly_AsProvided()
    {
        var cdt = new Triangulation<double>(VertexInsertionOrder.AsProvided);
        cdt.InsertVertices(_vertices);
        return cdt;
    }

    [Benchmark(Description = "Vertices only – Auto")]
    [BenchmarkCategory("VerticesOnly")]
    public Triangulation<double> VerticesOnly_Auto()
    {
        var cdt = new Triangulation<double>(VertexInsertionOrder.Auto);
        cdt.InsertVertices(_vertices);
        return cdt;
    }

    // -- Vertices + edges (constrained DT) -----------------------------------

    [Benchmark(Description = "Constrained – AsProvided")]
    [BenchmarkCategory("Constrained")]
    public Triangulation<double> Constrained_AsProvided()
    {
        var cdt = new Triangulation<double>(VertexInsertionOrder.AsProvided);
        cdt.InsertVertices(_vertices);
        cdt.InsertEdges(_edges);
        return cdt;
    }

    [Benchmark(Description = "Constrained – Auto")]
    [BenchmarkCategory("Constrained")]
    public Triangulation<double> Constrained_Auto()
    {
        var cdt = new Triangulation<double>(VertexInsertionOrder.Auto);
        cdt.InsertVertices(_vertices);
        cdt.InsertEdges(_edges);
        return cdt;
    }

    // -- Conforming DT -------------------------------------------------------

    [Benchmark(Description = "Conforming – Auto")]
    [BenchmarkCategory("Conforming")]
    public Triangulation<double> Conforming_Auto()
    {
        var cdt = new Triangulation<double>(VertexInsertionOrder.Auto);
        cdt.InsertVertices(_vertices);
        cdt.ConformToEdges(_edges);
        return cdt;
    }

    // -- Full pipeline (insert + erase outer + holes) ------------------------

    [Benchmark(Description = "Full pipeline – Auto")]
    [BenchmarkCategory("FullPipeline")]
    public Triangulation<double> FullPipeline_Auto()
    {
        var cdt = new Triangulation<double>(VertexInsertionOrder.Auto,
            IntersectingConstraintEdges.TryResolve, 0.0);
        cdt.InsertVertices(_vertices);
        cdt.InsertEdges(_edges);
        cdt.EraseOuterTrianglesAndHoles();
        return cdt;
    }
}

// ---------------------------------------------------------------------------
// Smaller datasets for micro-benchmarks
// ---------------------------------------------------------------------------

/// <summary>
/// Benchmarks on the smaller "cdt.txt" dataset (~101 vertices, ~103 edges)
/// to measure per-vertex overhead without the noise of large datasets.
/// </summary>
[MemoryDiagnoser]
[EventPipeProfiler(EventPipeProfile.CpuSampling)]
[GroupBenchmarksBy(BenchmarkLogicalGroupRule.ByCategory)]
[CategoriesColumn]
[ShortRunJob]
public class SmallDatasetBenchmarks
{
    private List<V2d<double>> _vertices = null!;
    private List<Edge> _edges = null!;

    [GlobalSetup]
    public void Setup()
    {
        (_vertices, _edges) = BenchmarkInputReader.Read("cdt.txt");
    }

    [Benchmark(Description = "Small – Constrained Auto")]
    [BenchmarkCategory("Small")]
    public Triangulation<double> Small_Constrained_Auto()
    {
        var cdt = new Triangulation<double>(VertexInsertionOrder.Auto);
        cdt.InsertVertices(_vertices);
        cdt.InsertEdges(_edges);
        return cdt;
    }

    [Benchmark(Description = "Small – Constrained AsProvided")]
    [BenchmarkCategory("Small")]
    public Triangulation<double> Small_Constrained_AsProvided()
    {
        var cdt = new Triangulation<double>(VertexInsertionOrder.AsProvided);
        cdt.InsertVertices(_vertices);
        cdt.InsertEdges(_edges);
        return cdt;
    }

    [Benchmark(Description = "Small – float vs double: double Auto")]
    [BenchmarkCategory("FloatVsDouble")]
    public Triangulation<double> FloatVsDouble_Double()
    {
        var cdt = new Triangulation<double>(VertexInsertionOrder.Auto);
        cdt.InsertVertices(_vertices);
        cdt.InsertEdges(_edges);
        return cdt;
    }

    [Benchmark(Description = "Small – float vs double: float Auto")]
    [BenchmarkCategory("FloatVsDouble")]
    public Triangulation<float> FloatVsDouble_Float()
    {
        var vf = _vertices.Select(v => new V2d<float>((float)v.X, (float)v.Y)).ToList();
        var ef = _edges;
        var cdt = new Triangulation<float>(VertexInsertionOrder.Auto);
        cdt.InsertVertices(vf);
        cdt.InsertEdges(ef);
        return cdt;
    }
}
