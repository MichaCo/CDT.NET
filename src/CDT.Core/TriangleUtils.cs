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
    public static PtLineLocation LocatePointLine(
        V2d<double> p, V2d<double> v1, V2d<double> v2, double tolerance = 0.0)
    {
        double o = Predicates.Orient2D(v1.X, v1.Y, v2.X, v2.Y, p.X, p.Y);
        return ClassifyOrientation(o, tolerance);
    }

    /// <inheritdoc cref="LocatePointLine(V2d{double},V2d{double},V2d{double},double)"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static PtLineLocation LocatePointLine(
        V2d<float> p, V2d<float> v1, V2d<float> v2, float tolerance = 0f)
    {
        float o = Predicates.Orient2D(v1.X, v1.Y, v2.X, v2.Y, p.X, p.Y);
        return ClassifyOrientation(o, tolerance);
    }

    /// <summary>Classifies the position of point <paramref name="p"/> within triangle (v1,v2,v3).</summary>
    public static PtTriLocation LocatePointTriangle(
        V2d<double> p,
        V2d<double> v1, V2d<double> v2, V2d<double> v3)
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

    /// <inheritdoc cref="LocatePointTriangle(V2d{double},V2d{double},V2d{double},V2d{double})"/>
    public static PtTriLocation LocatePointTriangle(
        V2d<float> p,
        V2d<float> v1, V2d<float> v2, V2d<float> v3)
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
    public static bool IsInCircumcircle(
        V2d<double> p,
        V2d<double> v1, V2d<double> v2, V2d<double> v3)
        => Predicates.InCircle(v1.X, v1.Y, v2.X, v2.Y, v3.X, v3.Y, p.X, p.Y) > 0.0;

    /// <inheritdoc cref="IsInCircumcircle(V2d{double},V2d{double},V2d{double},V2d{double})"/>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsInCircumcircle(
        V2d<float> p,
        V2d<float> v1, V2d<float> v2, V2d<float> v3)
        => Predicates.InCircle(v1.X, v1.Y, v2.X, v2.Y, v3.X, v3.Y, p.X, p.Y) > 0f;

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
    public static V2d<double> IntersectionPosition(
        V2d<double> a, V2d<double> b,
        V2d<double> c, V2d<double> d)
    {
        double acd = Predicates.Orient2D(c.X, c.Y, d.X, d.Y, a.X, a.Y);
        double bcd = Predicates.Orient2D(c.X, c.Y, d.X, d.Y, b.X, b.Y);
        double tab = acd / (acd - bcd);

        double cab = Predicates.Orient2D(a.X, a.Y, b.X, b.Y, c.X, c.Y);
        double dab = Predicates.Orient2D(a.X, a.Y, b.X, b.Y, d.X, d.Y);
        double tcd = cab / (cab - dab);

        return new V2d<double>(
            Math.Abs(a.X - b.X) < Math.Abs(c.X - d.X) ? Lerp(a.X, b.X, tab) : Lerp(c.X, d.X, tcd),
            Math.Abs(a.Y - b.Y) < Math.Abs(c.Y - d.Y) ? Lerp(a.Y, b.Y, tab) : Lerp(c.Y, d.Y, tcd));
    }

    /// <inheritdoc cref="IntersectionPosition(V2d{double},V2d{double},V2d{double},V2d{double})"/>
    public static V2d<float> IntersectionPosition(
        V2d<float> a, V2d<float> b,
        V2d<float> c, V2d<float> d)
    {
        // Use float-precision Orient2D throughout, matching C++ detail::intersectionPosition<float>.
        float acd = Predicates.Orient2D(c.X, c.Y, d.X, d.Y, a.X, a.Y);
        float bcd = Predicates.Orient2D(c.X, c.Y, d.X, d.Y, b.X, b.Y);
        float tab = acd / (acd - bcd);

        float cab = Predicates.Orient2D(a.X, a.Y, b.X, b.Y, c.X, c.Y);
        float dab = Predicates.Orient2D(a.X, a.Y, b.X, b.Y, d.X, d.Y);
        float tcd = cab / (cab - dab);

        return new V2d<float>(
            MathF.Abs(a.X - b.X) < MathF.Abs(c.X - d.X) ? LerpF(a.X, b.X, tab) : LerpF(c.X, d.X, tcd),
            MathF.Abs(a.Y - b.Y) < MathF.Abs(c.Y - d.Y) ? LerpF(a.Y, b.Y, tab) : LerpF(c.Y, d.Y, tcd));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static double Lerp(double a, double b, double t) => (1.0 - t) * a + t * b;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float LerpF(float a, float b, float t) => (1f - t) * a + t * b;
}
