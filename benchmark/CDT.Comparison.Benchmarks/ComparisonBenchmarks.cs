// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Configs;
using CDT;

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

        var edges = new List<Edge>(ev1.Length);
        for (int i = 0; i < ev1.Length; i++)
            edges.Add(new Edge(ev1[i], ev2[i]));

        var cdt = new Triangulation<double>(VertexInsertionOrder.Auto);
        cdt.InsertVertices(verts);
        cdt.InsertEdges(edges);
        return cdt.Triangles.Length;
    }
}

// ---------------------------------------------------------------------------
// Benchmark class — "Constrained Sweden" dataset
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

    // -- Constrained ---------------------------------------------------------

    [Benchmark(Baseline = true, Description = "CDT.NET")]
    [BenchmarkCategory("Constrained")]
    public int CDT_CdtNet() => CdtNetAdapter.Constrained(_xs, _ys, _ev1, _ev2);
}
