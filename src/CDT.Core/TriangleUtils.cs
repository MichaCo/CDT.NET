// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System.Numerics;
using System.Runtime.CompilerServices;

namespace CDT;

/// <summary>
/// Triangle-index-aware geometric utilities and helper methods.
/// </summary>
internal static class TriangleUtils
{
    // -------------------------------------------------------------------------
    // Index cycling
    // -------------------------------------------------------------------------

    /// <summary>Next vertex/neighbor index (CCW): 0→1→2→0.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Ccw(int i) => (i + 1) % 3;

    /// <summary>Previous vertex/neighbor index (CW): 0→2→1→0.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Cw(int i) => (i + 2) % 3;

    /// <summary>Neighbor index opposed to vertex index: 0→1, 1→2, 2→0.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int OpposedNeighborIndex(int vertexIndex) => vertexIndex switch
    {
        0 => 1,
        1 => 2,
        _ => 0,
    };

    /// <summary>Vertex index opposed to neighbor index: 0→2, 1→0, 2→1.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int OpposedVertexIndex(int neighborIndex) => neighborIndex switch
    {
        0 => 2,
        1 => 0,
        _ => 1,
    };

    // -------------------------------------------------------------------------
    // Per-triangle queries
    // -------------------------------------------------------------------------

    /// <summary>Index (0,1,2) of vertex <paramref name="v"/> within triangle <paramref name="t"/>.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int VertexIndex(in Triangle t, int v)
    {
        if (t.V0 == v) return 0;
        if (t.V1 == v) return 1;
        return 2;
    }

    /// <summary>
    /// Neighbor-index (0,1,2) of the neighbor sharing vertices <paramref name="va"/>
    /// and <paramref name="vb"/> in triangle <paramref name="t"/>.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int EdgeNeighborIndex(in Triangle t, int va, int vb)
    {
        // Neighbor at index i shares edge (v[i], v[ccw(i)])
        if (t.V0 == va)
        {
            if (t.V1 == vb) return 0; // edge (V0,V1) → n[0]
            return 2; // edge (V0,V2) → n[2]
        }
        if (t.V0 == vb)
        {
            if (t.V1 == va) return 0; // edge (V1,V0) → n[0]
            return 2; // edge (V2,V0) → n[2]
        }
        return 1; // edge (V1,V2) → n[1]
    }

    /// <summary>Index (0,1,2) of triangle <paramref name="iTopo"/> in the neighbor list of <paramref name="t"/>.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int NeighborIndex(in Triangle t, int iTopo)
    {
        if (t.N0 == iTopo) return 0;
        if (t.N1 == iTopo) return 1;
        return 2;
    }

    /// <summary>Returns the vertex of <paramref name="t"/> opposed to neighbor <paramref name="iTopo"/>.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int OpposedVertex(in Triangle t, int iTopo)
        => t.GetVertex(OpposedVertexIndex(NeighborIndex(t, iTopo)));

    /// <summary>Returns the neighbor of <paramref name="t"/> opposed to vertex <paramref name="v"/>.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int OpposedTriangle(in Triangle t, int v)
        => t.GetNeighbor(OpposedNeighborIndex(VertexIndex(t, v)));

    /// <summary>Returns the neighbor of <paramref name="t"/> that shares edge (va,vb).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int EdgeNeighbor(in Triangle t, int va, int vb)
        => t.GetNeighbor(EdgeNeighborIndex(t, va, vb));

    // -------------------------------------------------------------------------
    // Super-triangle
    // -------------------------------------------------------------------------

    /// <summary>Returns true if any vertex index belongs to the super-triangle (0,1,2).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TouchesSuperTriangle(in Triangle t)
        => t.V0 < Indices.SuperTriangleVertexCount
        || t.V1 < Indices.SuperTriangleVertexCount
        || t.V2 < Indices.SuperTriangleVertexCount;

    // -------------------------------------------------------------------------
    // Geometric helpers
    // -------------------------------------------------------------------------

    /// <summary>Squared Euclidean distance between two points.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static T DistanceSquared<T>(T ax, T ay, T bx, T by)
        where T : IFloatingPoint<T>
    {
        T dx = bx - ax;
        T dy = by - ay;
        return dx * dx + dy * dy;
    }

    /// <summary>Squared Euclidean distance between two points.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static T DistanceSquared<T>(V2d<T> a, V2d<T> b)
        where T : IFloatingPoint<T>
        => DistanceSquared(a.X, a.Y, b.X, b.Y);

    /// <summary>Euclidean distance between two points.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static T Distance<T>(V2d<T> a, V2d<T> b)
        where T : IFloatingPoint<T>, IRootFunctions<T>
        => T.Sqrt(DistanceSquared(a, b));

    /// <summary>Classifies the sign of an orientation value.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static PtLineLocation ClassifyOrientation<T>(T orientation, T tolerance)
        where T : IFloatingPoint<T>
    {
        if (orientation < -tolerance) return PtLineLocation.Right;
        if (orientation > tolerance) return PtLineLocation.Left;
        return PtLineLocation.OnLine;
    }

    /// <summary>Returns true when the location is on any triangle edge.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsOnEdge(PtTriLocation location)
        => location is PtTriLocation.OnEdge0
                    or PtTriLocation.OnEdge1
                    or PtTriLocation.OnEdge2;

    /// <summary>Returns the edge neighbor index from an on-edge location.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int EdgeNeighborFromLocation(PtTriLocation location)
        => (int)(location - PtTriLocation.OnEdge0);

    /// <summary>
    /// Locates a point relative to the directed line from <paramref name="v1"/>
    /// to <paramref name="v2"/>.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static PtLineLocation LocatePointLine<T>(V2d<T> p, V2d<T> v1, V2d<T> v2, T tolerance = default)
        where T : unmanaged, IFloatingPoint<T>
        => ClassifyOrientation(Orient2D(p, v1, v2), tolerance);

    /// <summary>Classifies the position of point <paramref name="p"/> within triangle (v1,v2,v3).</summary>
    public static PtTriLocation LocatePointTriangle<T>(V2d<T> p, V2d<T> v1, V2d<T> v2, V2d<T> v3)
        where T : unmanaged, IFloatingPoint<T>
    {
        PtTriLocation result = PtTriLocation.Inside;

        PtLineLocation e = LocatePointLine(p, v1, v2);
        if (e == PtLineLocation.Right) return PtTriLocation.Outside;
        if (e == PtLineLocation.OnLine) result = PtTriLocation.OnEdge0;

        e = LocatePointLine(p, v2, v3);
        if (e == PtLineLocation.Right) return PtTriLocation.Outside;
        if (e == PtLineLocation.OnLine)
            result = result == PtTriLocation.Inside ? PtTriLocation.OnEdge1 : PtTriLocation.OnVertex;

        e = LocatePointLine(p, v3, v1);
        if (e == PtLineLocation.Right) return PtTriLocation.Outside;
        if (e == PtLineLocation.OnLine)
            result = result == PtTriLocation.Inside ? PtTriLocation.OnEdge2 : PtTriLocation.OnVertex;

        return result;
    }

    /// <summary>
    /// Tests whether point <paramref name="p"/> is inside the circumcircle of triangle (v1,v2,v3).
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsInCircumcircle<T>(V2d<T> p, V2d<T> v1, V2d<T> v2, V2d<T> v3)
        where T : unmanaged, IFloatingPoint<T>
    {
        if (typeof(T) == typeof(double))
            return Predicates.InCircle(
                Unsafe.As<T, double>(ref Unsafe.AsRef(in v1.X)), Unsafe.As<T, double>(ref Unsafe.AsRef(in v1.Y)),
                Unsafe.As<T, double>(ref Unsafe.AsRef(in v2.X)), Unsafe.As<T, double>(ref Unsafe.AsRef(in v2.Y)),
                Unsafe.As<T, double>(ref Unsafe.AsRef(in v3.X)), Unsafe.As<T, double>(ref Unsafe.AsRef(in v3.Y)),
                Unsafe.As<T, double>(ref Unsafe.AsRef(in p.X)),  Unsafe.As<T, double>(ref Unsafe.AsRef(in p.Y))) > 0.0;
        else
            return Predicates.InCircle(
                Unsafe.As<T, float>(ref Unsafe.AsRef(in v1.X)), Unsafe.As<T, float>(ref Unsafe.AsRef(in v1.Y)),
                Unsafe.As<T, float>(ref Unsafe.AsRef(in v2.X)), Unsafe.As<T, float>(ref Unsafe.AsRef(in v2.Y)),
                Unsafe.As<T, float>(ref Unsafe.AsRef(in v3.X)), Unsafe.As<T, float>(ref Unsafe.AsRef(in v3.Y)),
                Unsafe.As<T, float>(ref Unsafe.AsRef(in p.X)),  Unsafe.As<T, float>(ref Unsafe.AsRef(in p.Y))) > 0f;
    }

    /// <summary>Tests whether any pair of triangles (given as vertex triangle lists) share an edge.</summary>
    public static bool VerticesShareEdge(List<int> aTris, List<int> bTris)
    {
        foreach (int t in aTris)
        {
            if (bTris.Contains(t)) return true;
        }
        return false;
    }

    /// <summary>Computes the intersection point of segments (a,b) and (c,d).</summary>
    public static V2d<T> IntersectionPosition<T>(V2d<T> a, V2d<T> b, V2d<T> c, V2d<T> d)
        where T : unmanaged, IFloatingPoint<T>
    {
        T acd = Orient2D(a, c, d);
        T bcd = Orient2D(b, c, d);
        T tab = acd / (acd - bcd);

        T cab = Orient2D(c, a, b);
        T dab = Orient2D(d, a, b);
        T tcd = cab / (cab - dab);

        static T Lerp(T x, T y, T t) => (T.One - t) * x + t * y;
        return new V2d<T>(
            T.Abs(a.X - b.X) < T.Abs(c.X - d.X) ? Lerp(a.X, b.X, tab) : Lerp(c.X, d.X, tcd),
            T.Abs(a.Y - b.Y) < T.Abs(c.Y - d.Y) ? Lerp(a.Y, b.Y, tab) : Lerp(c.Y, d.Y, tcd));
    }

    /// <summary>Returns the signed area (orientation) of triangle (v1, v2, p).</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static T Orient2D<T>(V2d<T> p, V2d<T> v1, V2d<T> v2)
        where T : unmanaged, IFloatingPoint<T>
    {
        if (typeof(T) == typeof(double))
        {
            double r = Predicates.Orient2D(
                Unsafe.As<T, double>(ref Unsafe.AsRef(in v1.X)), Unsafe.As<T, double>(ref Unsafe.AsRef(in v1.Y)),
                Unsafe.As<T, double>(ref Unsafe.AsRef(in v2.X)), Unsafe.As<T, double>(ref Unsafe.AsRef(in v2.Y)),
                Unsafe.As<T, double>(ref Unsafe.AsRef(in p.X)),  Unsafe.As<T, double>(ref Unsafe.AsRef(in p.Y)));
            return Unsafe.As<double, T>(ref r);
        }
        else
        {
            float r = Predicates.Orient2D(
                Unsafe.As<T, float>(ref Unsafe.AsRef(in v1.X)), Unsafe.As<T, float>(ref Unsafe.AsRef(in v1.Y)),
                Unsafe.As<T, float>(ref Unsafe.AsRef(in v2.X)), Unsafe.As<T, float>(ref Unsafe.AsRef(in v2.Y)),
                Unsafe.As<T, float>(ref Unsafe.AsRef(in p.X)),  Unsafe.As<T, float>(ref Unsafe.AsRef(in p.Y)));
            return Unsafe.As<float, T>(ref r);
        }
    }
}
