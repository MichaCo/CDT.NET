// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using BenchmarkDotNet.Attributes;
using BenchmarkDotNet.Configs;
using BenchmarkDotNet.Diagnosers;
using BenchmarkDotNet.Running;
using CDT.Predicates;

// ---------------------------------------------------------------------------
// Predicates microbenchmarks
// ---------------------------------------------------------------------------

/// <summary>
/// Microbenchmarks that compare <see cref="PredicatesAdaptive"/> and
/// <see cref="PredicatesExact"/> head-to-head for every predicate variant.
///
/// <para>
/// Two input scenarios are covered for each method:
/// <list type="bullet">
///   <item><b>General</b> – random coordinates in general position.
///   The adaptive fast-path (Stage A) returns immediately without any
///   expansion arithmetic, so these numbers reflect the absolute best
///   case for <see cref="PredicatesAdaptive"/>.</item>
///   <item><b>NearDegenerate</b> – nearly-collinear / nearly-co-circular
///   points that force the adaptive predicate through Stage B–D (and
///   therefore all the way to the exact expansion code).  These numbers
///   show the cost of the slow path and are comparable to what
///   <see cref="PredicatesExact"/> always pays.</item>
/// </list>
/// </para>
/// </summary>
[MemoryDiagnoser]
[EventPipeProfiler(EventPipeProfile.CpuSampling)]
[GroupBenchmarksBy(BenchmarkLogicalGroupRule.ByCategory)]
[CategoriesColumn]
[ShortRunJob]
public class PredicatesBenchmarks
{
    // Number of predicate calls per benchmark iteration – large enough to
    // dominate timer resolution, small enough for a short-run job.
    private const int N = 4_000;

    // ── General-position input arrays ──────────────────────────────────────
    // Orient2d uses 3 points (6 doubles / 6 floats) per call.
    // InCircle  uses 4 points (8 doubles / 8 floats) per call.
    private double[] _orient2dD = null!;
    private float[]  _orient2dF = null!;
    private double[] _inCircleD = null!;
    private float[]  _inCircleF = null!;

    // ── Near-degenerate input arrays ───────────────────────────────────────
    // Orient2d: nearly-collinear triples.
    // InCircle:  points nearly on a common circumcircle.
    private double[] _orient2dDeg = null!;
    private float[]  _orient2dFDeg = null!;
    private double[] _inCircleDeg = null!;
    private float[]  _inCircleFDeg = null!;

    [GlobalSetup]
    public void Setup()
    {
        var rng = new Random(unchecked((int)0xDEAD_BEEF));

        // ── general-position ───────────────────────────────────────────────
        _orient2dD = FillDoubles(rng, 6 * N, scale: 1000.0);
        _orient2dF = ToFloats(_orient2dD);
        _inCircleD = FillDoubles(rng, 8 * N, scale: 1000.0);
        _inCircleF = ToFloats(_inCircleD);

        // ── near-degenerate ────────────────────────────────────────────────
        // Orient2d: triple (A, B, C) where C is nearly on line AB.
        // We construct A=(0,0), B=(1,1), C=(t, t+ε) for varying t and ε≈1e-14.
        _orient2dDeg = BuildNearCollinear(N);
        _orient2dFDeg = ToFloats(_orient2dDeg);

        // InCircle: quadruple (A,B,C,D) where D is nearly on the
        // circumcircle of A, B, C.
        _inCircleDeg = BuildNearCircle(N);
        _inCircleFDeg = ToFloats(_inCircleDeg);
    }

    // =========================================================================
    // Orient2d – double
    // =========================================================================

    [Benchmark(Description = "Adaptive – Orient2d double (general)")]
    [BenchmarkCategory("Orient2d-double")]
    public double Adaptive_Orient2d_Double_General()
    {
        double acc = 0.0;
        double[] pts = _orient2dD;
        for (int i = 0; i < N; i++)
        {
            int j = i * 6;
            acc += PredicatesAdaptive.Orient2d(
                pts[j], pts[j + 1], pts[j + 2], pts[j + 3], pts[j + 4], pts[j + 5]);
        }
        return acc;
    }

    [Benchmark(Description = "Exact – Orient2d double (general)")]
    [BenchmarkCategory("Orient2d-double")]
    public double Exact_Orient2d_Double_General()
    {
        double acc = 0.0;
        double[] pts = _orient2dD;
        for (int i = 0; i < N; i++)
        {
            int j = i * 6;
            acc += PredicatesExact.Orient2d(
                pts[j], pts[j + 1], pts[j + 2], pts[j + 3], pts[j + 4], pts[j + 5]);
        }
        return acc;
    }

    [Benchmark(Description = "Adaptive – Orient2d double (near-degenerate)")]
    [BenchmarkCategory("Orient2d-double-hard")]
    public double Adaptive_Orient2d_Double_NearDegenerate()
    {
        double acc = 0.0;
        double[] pts = _orient2dDeg;
        for (int i = 0; i < N; i++)
        {
            int j = i * 6;
            acc += PredicatesAdaptive.Orient2d(
                pts[j], pts[j + 1], pts[j + 2], pts[j + 3], pts[j + 4], pts[j + 5]);
        }
        return acc;
    }

    [Benchmark(Description = "Exact – Orient2d double (near-degenerate)")]
    [BenchmarkCategory("Orient2d-double-hard")]
    public double Exact_Orient2d_Double_NearDegenerate()
    {
        double acc = 0.0;
        double[] pts = _orient2dDeg;
        for (int i = 0; i < N; i++)
        {
            int j = i * 6;
            acc += PredicatesExact.Orient2d(
                pts[j], pts[j + 1], pts[j + 2], pts[j + 3], pts[j + 4], pts[j + 5]);
        }
        return acc;
    }

    // =========================================================================
    // Orient2d – float
    // =========================================================================

    [Benchmark(Description = "Adaptive – Orient2d float (general)")]
    [BenchmarkCategory("Orient2d-float")]
    public float Adaptive_Orient2d_Float_General()
    {
        float acc = 0f;
        float[] pts = _orient2dF;
        for (int i = 0; i < N; i++)
        {
            int j = i * 6;
            acc += PredicatesAdaptive.Orient2d(
                pts[j], pts[j + 1], pts[j + 2], pts[j + 3], pts[j + 4], pts[j + 5]);
        }
        return acc;
    }

    [Benchmark(Description = "Exact – Orient2d float (general)")]
    [BenchmarkCategory("Orient2d-float")]
    public float Exact_Orient2d_Float_General()
    {
        float acc = 0f;
        float[] pts = _orient2dF;
        for (int i = 0; i < N; i++)
        {
            int j = i * 6;
            acc += PredicatesExact.Orient2d(
                pts[j], pts[j + 1], pts[j + 2], pts[j + 3], pts[j + 4], pts[j + 5]);
        }
        return acc;
    }

    [Benchmark(Description = "Adaptive – Orient2d float (near-degenerate)")]
    [BenchmarkCategory("Orient2d-float-hard")]
    public float Adaptive_Orient2d_Float_NearDegenerate()
    {
        float acc = 0f;
        float[] pts = _orient2dFDeg;
        for (int i = 0; i < N; i++)
        {
            int j = i * 6;
            acc += PredicatesAdaptive.Orient2d(
                pts[j], pts[j + 1], pts[j + 2], pts[j + 3], pts[j + 4], pts[j + 5]);
        }
        return acc;
    }

    [Benchmark(Description = "Exact – Orient2d float (near-degenerate)")]
    [BenchmarkCategory("Orient2d-float-hard")]
    public float Exact_Orient2d_Float_NearDegenerate()
    {
        float acc = 0f;
        float[] pts = _orient2dFDeg;
        for (int i = 0; i < N; i++)
        {
            int j = i * 6;
            acc += PredicatesExact.Orient2d(
                pts[j], pts[j + 1], pts[j + 2], pts[j + 3], pts[j + 4], pts[j + 5]);
        }
        return acc;
    }

    // =========================================================================
    // InCircle – double
    // =========================================================================

    [Benchmark(Description = "Adaptive – InCircle double (general)")]
    [BenchmarkCategory("InCircle-double")]
    public double Adaptive_InCircle_Double_General()
    {
        double acc = 0.0;
        double[] pts = _inCircleD;
        for (int i = 0; i < N; i++)
        {
            int j = i * 8;
            acc += PredicatesAdaptive.InCircle(
                pts[j], pts[j + 1], pts[j + 2], pts[j + 3],
                pts[j + 4], pts[j + 5], pts[j + 6], pts[j + 7]);
        }
        return acc;
    }

    [Benchmark(Description = "Exact – InCircle double (general)")]
    [BenchmarkCategory("InCircle-double")]
    public double Exact_InCircle_Double_General()
    {
        double acc = 0.0;
        double[] pts = _inCircleD;
        for (int i = 0; i < N; i++)
        {
            int j = i * 8;
            acc += PredicatesExact.InCircle(
                pts[j], pts[j + 1], pts[j + 2], pts[j + 3],
                pts[j + 4], pts[j + 5], pts[j + 6], pts[j + 7]);
        }
        return acc;
    }

    [Benchmark(Description = "Adaptive – InCircle double (near-degenerate)")]
    [BenchmarkCategory("InCircle-double-hard")]
    public double Adaptive_InCircle_Double_NearDegenerate()
    {
        double acc = 0.0;
        double[] pts = _inCircleDeg;
        for (int i = 0; i < N; i++)
        {
            int j = i * 8;
            acc += PredicatesAdaptive.InCircle(
                pts[j], pts[j + 1], pts[j + 2], pts[j + 3],
                pts[j + 4], pts[j + 5], pts[j + 6], pts[j + 7]);
        }
        return acc;
    }

    [Benchmark(Description = "Exact – InCircle double (near-degenerate)")]
    [BenchmarkCategory("InCircle-double-hard")]
    public double Exact_InCircle_Double_NearDegenerate()
    {
        double acc = 0.0;
        double[] pts = _inCircleDeg;
        for (int i = 0; i < N; i++)
        {
            int j = i * 8;
            acc += PredicatesExact.InCircle(
                pts[j], pts[j + 1], pts[j + 2], pts[j + 3],
                pts[j + 4], pts[j + 5], pts[j + 6], pts[j + 7]);
        }
        return acc;
    }

    // =========================================================================
    // InCircle – float
    // =========================================================================

    [Benchmark(Description = "Adaptive – InCircle float (general)")]
    [BenchmarkCategory("InCircle-float")]
    public float Adaptive_InCircle_Float_General()
    {
        float acc = 0f;
        float[] pts = _inCircleF;
        for (int i = 0; i < N; i++)
        {
            int j = i * 8;
            acc += PredicatesAdaptive.InCircle(
                pts[j], pts[j + 1], pts[j + 2], pts[j + 3],
                pts[j + 4], pts[j + 5], pts[j + 6], pts[j + 7]);
        }
        return acc;
    }

    [Benchmark(Description = "Exact – InCircle float (general)")]
    [BenchmarkCategory("InCircle-float")]
    public float Exact_InCircle_Float_General()
    {
        float acc = 0f;
        float[] pts = _inCircleF;
        for (int i = 0; i < N; i++)
        {
            int j = i * 8;
            acc += PredicatesExact.InCircle(
                pts[j], pts[j + 1], pts[j + 2], pts[j + 3],
                pts[j + 4], pts[j + 5], pts[j + 6], pts[j + 7]);
        }
        return acc;
    }

    [Benchmark(Description = "Adaptive – InCircle float (near-degenerate)")]
    [BenchmarkCategory("InCircle-float-hard")]
    public float Adaptive_InCircle_Float_NearDegenerate()
    {
        float acc = 0f;
        float[] pts = _inCircleFDeg;
        for (int i = 0; i < N; i++)
        {
            int j = i * 8;
            acc += PredicatesAdaptive.InCircle(
                pts[j], pts[j + 1], pts[j + 2], pts[j + 3],
                pts[j + 4], pts[j + 5], pts[j + 6], pts[j + 7]);
        }
        return acc;
    }

    [Benchmark(Description = "Exact – InCircle float (near-degenerate)")]
    [BenchmarkCategory("InCircle-float-hard")]
    public float Exact_InCircle_Float_NearDegenerate()
    {
        float acc = 0f;
        float[] pts = _inCircleFDeg;
        for (int i = 0; i < N; i++)
        {
            int j = i * 8;
            acc += PredicatesExact.InCircle(
                pts[j], pts[j + 1], pts[j + 2], pts[j + 3],
                pts[j + 4], pts[j + 5], pts[j + 6], pts[j + 7]);
        }
        return acc;
    }

    // =========================================================================
    // Helpers
    // =========================================================================

    private static double[] FillDoubles(Random rng, int count, double scale)
    {
        var arr = new double[count];
        for (int i = 0; i < arr.Length; i++)
            arr[i] = (rng.NextDouble() - 0.5) * 2.0 * scale;
        return arr;
    }

    private static float[] ToFloats(double[] src)
    {
        var arr = new float[src.Length];
        for (int i = 0; i < src.Length; i++)
            arr[i] = (float)src[i];
        return arr;
    }

    /// <summary>
    /// Builds an array of nearly-collinear Orient2d triples.
    /// A=(0,0), B=(t,t), C=(2t, 2t+ε) – nearly collinear because ε is tiny.
    /// Varies t across [1,100] and ε across [1e-14, 1e-13] to keep results
    /// diverse while reliably triggering Stage B/C of the adaptive predicate.
    /// </summary>
    private static double[] BuildNearCollinear(int n)
    {
        var arr = new double[6 * n];
        for (int i = 0; i < n; i++)
        {
            double t = 1.0 + i * (99.0 / n);          // t in [1, 100]
            double eps = 1e-14 * (1.0 + i % 10);      // ε in [1e-14, 1e-13]
            int j = i * 6;
            arr[j]     = 0.0;  arr[j + 1] = 0.0;     // A
            arr[j + 2] = t;    arr[j + 3] = t;        // B
            arr[j + 4] = 2 * t;  arr[j + 5] = 2 * t + eps; // C (nearly collinear)
        }
        return arr;
    }

    /// <summary>
    /// Builds nearly-co-circular InCircle quadruples.
    /// A, B, C are vertices of an equilateral triangle inscribed in the unit
    /// circle; D is placed at radius 1 + ε so it is just barely outside –
    /// but close enough that the fast floating-point test cannot decide.
    /// </summary>
    private static double[] BuildNearCircle(int n)
    {
        var arr = new double[8 * n];
        for (int i = 0; i < n; i++)
        {
            double r = 100.0 + i * (900.0 / n);       // radius in [100, 1000]
            double eps = 1e-11 * r * (1.0 + i % 5);   // ε scaled to radius

            double ax = r, ay = 0.0;
            double bx = r * Math.Cos(2 * Math.PI / 3),
                   by = r * Math.Sin(2 * Math.PI / 3);
            double cx = r * Math.Cos(4 * Math.PI / 3),
                   cy = r * Math.Sin(4 * Math.PI / 3);
            double dx = r + eps, dy = 0.0;             // D just outside

            int j = i * 8;
            arr[j]     = ax; arr[j + 1] = ay;
            arr[j + 2] = bx; arr[j + 3] = by;
            arr[j + 4] = cx; arr[j + 5] = cy;
            arr[j + 6] = dx; arr[j + 7] = dy;
        }
        return arr;
    }
}
