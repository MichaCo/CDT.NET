// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System.Collections.ObjectModel;
using System.Numerics;
using System.Runtime.CompilerServices;
using CDT.Predicates;

namespace CDT;

/// <summary>
/// Duplicates information for vertex deduplication.
/// </summary>
public sealed class DuplicatesInfo
{
    /// <summary>
    /// Maps each original vertex index to its canonical (deduplicated) index.
    /// </summary>
    public IReadOnlyList<int> Mapping { get; }

    /// <summary>Indices of duplicate vertices in the original list.</summary>
    public IReadOnlyList<int> Duplicates { get; }

    internal DuplicatesInfo(int[] mapping, List<int> duplicates)
    {
        Mapping = Array.AsReadOnly(mapping);
        Duplicates = duplicates.AsReadOnly();
    }
}

/// <summary>
/// Utility methods for duplicate removal, edge remapping, edge extraction,
/// and geometry helpers. Corresponds to C++ <c>CDTUtils.h</c> free functions
/// in the flat <c>CDT::</c> namespace.
/// </summary>
public static class CdtUtils
{
    // -------------------------------------------------------------------------
    // Duplicate removal
    // -------------------------------------------------------------------------

    /// <summary>
    /// Finds duplicate vertices (same X and Y) in the given list and returns
    /// mapping and duplicate index information.
    /// </summary>
    /// <typeparam name="T">Floating-point coordinate type.</typeparam>
    /// <param name="vertices">The vertex list to check for duplicates.</param>
    /// <returns>
    /// A <see cref="DuplicatesInfo"/> containing a mapping from each original
    /// index to its canonical index, and the list of duplicate indices.
    /// </returns>
    public static DuplicatesInfo FindDuplicates<T>(IReadOnlyList<V2d<T>> vertices)
        where T : IFloatingPoint<T>
    {
        int n = vertices.Count;
        var mapping = new int[n];
        var duplicates = new List<int>();
        var posToIndex = new Dictionary<V2dKey<T>, int>(n);

        int iOut = 0;
        for (int iIn = 0; iIn < n; iIn++)
        {
            var key = new V2dKey<T>(vertices[iIn].X, vertices[iIn].Y);
            if (posToIndex.TryGetValue(key, out int existing))
            {
                mapping[iIn] = existing;
                duplicates.Add(iIn);
            }
            else
            {
                posToIndex[key] = iOut;
                mapping[iIn] = iOut++;
            }
        }
        return new DuplicatesInfo(mapping, duplicates);
    }

    /// <summary>
    /// Finds duplicate vertices (same X and Y) in the given span and returns
    /// mapping and duplicate index information.
    /// </summary>
    /// <typeparam name="T">Floating-point coordinate type.</typeparam>
    /// <param name="vertices">The vertex span to check for duplicates.</param>
    /// <returns>
    /// A <see cref="DuplicatesInfo"/> containing a mapping from each original
    /// index to its canonical index, and the list of duplicate indices.
    /// </returns>
    public static DuplicatesInfo FindDuplicates<T>(ReadOnlySpan<V2d<T>> vertices)
        where T : IFloatingPoint<T>
    {
        int n = vertices.Length;
        var mapping = new int[n];
        var duplicates = new List<int>();
        var posToIndex = new Dictionary<V2dKey<T>, int>(n);

        int iOut = 0;
        for (int iIn = 0; iIn < n; iIn++)
        {
            var key = new V2dKey<T>(vertices[iIn].X, vertices[iIn].Y);
            if (posToIndex.TryGetValue(key, out int existing))
            {
                mapping[iIn] = existing;
                duplicates.Add(iIn);
            }
            else
            {
                posToIndex[key] = iOut;
                mapping[iIn] = iOut++;
            }
        }
        return new DuplicatesInfo(mapping, duplicates);
    }

    /// <summary>
    /// Removes duplicate vertices from the list in-place using the result from
    /// <see cref="FindDuplicates{T}(IReadOnlyList{V2d{T}})"/>.
    /// </summary>
    /// <typeparam name="T">Floating-point coordinate type.</typeparam>
    /// <param name="vertices">The vertex list to modify in-place.</param>
    /// <param name="duplicates">The list of duplicate indices to remove.</param>
    /// <remarks>The <paramref name="vertices"/> list is mutated in-place.</remarks>
    public static void RemoveDuplicates<T>(List<V2d<T>> vertices, IReadOnlyList<int> duplicates)
        where T : IFloatingPoint<T>
    {
        if (duplicates.Count == 0) return;
        var toRemove = new HashSet<int>(duplicates);
        int write = 0;
        for (int i = 0; i < vertices.Count; i++)
        {
            if (!toRemove.Contains(i))
                vertices[write++] = vertices[i];
        }
        vertices.RemoveRange(write, vertices.Count - write);
    }

    /// <summary>
    /// Finds and removes duplicate vertices, and remaps all edges accordingly.
    /// </summary>
    /// <typeparam name="T">Floating-point coordinate type.</typeparam>
    /// <param name="vertices">The vertex list to modify in-place.</param>
    /// <param name="edges">The edge list to remap in-place.</param>
    /// <returns>
    /// A <see cref="DuplicatesInfo"/> with the vertex index mapping and duplicate indices.
    /// </returns>
    public static DuplicatesInfo RemoveDuplicatesAndRemapEdges<T>(
        List<V2d<T>> vertices,
        List<Edge> edges)
        where T : IFloatingPoint<T>
    {
        var info = FindDuplicates(vertices);
        RemoveDuplicates(vertices, info.Duplicates);
        RemapEdges(edges, info.Mapping);
        return info;
    }

    // -------------------------------------------------------------------------
    // Edge remapping
    // -------------------------------------------------------------------------

    /// <summary>
    /// Remaps all edges using the given vertex index mapping (in-place).
    /// </summary>
    /// <param name="edges">The edge list to modify in-place.</param>
    /// <param name="mapping">
    /// Vertex index mapping: for each old vertex index <c>i</c>, <c>mapping[i]</c> is the new index.
    /// </param>
    /// <remarks>The <paramref name="edges"/> list is mutated in-place.</remarks>
    public static void RemapEdges(List<Edge> edges, IReadOnlyList<int> mapping)
    {
        for (int i = 0; i < edges.Count; i++)
        {
            edges[i] = new Edge(mapping[edges[i].V1], mapping[edges[i].V2]);
        }
    }

    // -------------------------------------------------------------------------
    // Edge extraction
    // -------------------------------------------------------------------------

    /// <summary>Extracts all unique edges from a triangle list.</summary>
    /// <param name="triangles">The triangle list to extract edges from.</param>
    /// <returns>A set containing all unique edges in the triangulation.</returns>
    public static HashSet<Edge> ExtractEdgesFromTriangles(IReadOnlyList<Triangle> triangles)
    {
        var edges = new HashSet<Edge>(triangles.Count * 3);
        foreach (var t in triangles)
        {
            edges.Add(new Edge(t.V0, t.V1));
            edges.Add(new Edge(t.V1, t.V2));
            edges.Add(new Edge(t.V2, t.V0));
        }
        return edges;
    }

    /// <summary>
    /// Calculates per-vertex triangle adjacency lists.
    /// Returns an array where each entry is the list of triangle indices adjacent to that vertex.
    /// </summary>
    /// <param name="triangles">The triangle list.</param>
    /// <param name="verticesCount">Total number of vertices.</param>
    /// <returns>
    /// A read-only list of read-only lists: for each vertex index, the list of adjacent triangle indices.
    /// </returns>
    public static IReadOnlyList<IReadOnlyList<int>> CalculateTrianglesByVertex(
        IReadOnlyList<Triangle> triangles,
        int verticesCount)
    {
        var result = new List<int>[verticesCount];
        for (int i = 0; i < verticesCount; i++) result[i] = new List<int>();
        for (int i = 0; i < triangles.Count; i++)
        {
            var t = triangles[i];
            result[t.V0].Add(i);
            result[t.V1].Add(i);
            result[t.V2].Add(i);
        }
        var readOnly = new IReadOnlyList<int>[verticesCount];
        for (int i = 0; i < verticesCount; i++) readOnly[i] = result[i].AsReadOnly();
        return new ReadOnlyCollection<IReadOnlyList<int>>(readOnly);
    }

    /// <summary>
    /// Converts a piece→originals edge mapping to the inverse: original→pieces.
    /// </summary>
    /// <param name="pieceToOriginals">
    /// A dictionary mapping each piece edge to the list of original edges it was split from.
    /// </param>
    /// <returns>
    /// A read-only dictionary mapping each original edge to the list of piece edges it was split into.
    /// </returns>
    public static IReadOnlyDictionary<Edge, IReadOnlyList<Edge>> EdgeToPiecesMapping(
        IReadOnlyDictionary<Edge, IReadOnlyList<Edge>> pieceToOriginals)
    {
        var edgeToPieces = new Dictionary<Edge, List<Edge>>();
        foreach (var kv in pieceToOriginals)
        {
            foreach (var orig in kv.Value)
            {
                if (!edgeToPieces.TryGetValue(orig, out var pieces))
                {
                    pieces = new List<Edge>();
                    edgeToPieces[orig] = pieces;
                }
                if (!pieces.Contains(kv.Key)) pieces.Add(kv.Key);
            }
        }
        var result = new Dictionary<Edge, IReadOnlyList<Edge>>(edgeToPieces.Count);
        foreach (var kv in edgeToPieces) result[kv.Key] = kv.Value.AsReadOnly();
        return result;
    }

    // -------------------------------------------------------------------------
    // Index cycling (C++: ccw / cw)
    // -------------------------------------------------------------------------

    /// <summary>
    /// Advances a vertex or neighbor index counter-clockwise within a triangle: 0→1→2→0.
    /// Corresponds to C++ <c>CDT::ccw()</c>.
    /// </summary>
    /// <param name="i">Current index (0, 1, or 2).</param>
    /// <returns>The next index in CCW order.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Ccw(int i) => (i + 1) % 3;

    /// <summary>
    /// Advances a vertex or neighbor index clockwise within a triangle: 0→2→1→0.
    /// Corresponds to C++ <c>CDT::cw()</c>.
    /// </summary>
    /// <param name="i">Current index (0, 1, or 2).</param>
    /// <returns>The next index in CW order.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int Cw(int i) => (i + 2) % 3;

    // -------------------------------------------------------------------------
    // Triangle index queries
    // -------------------------------------------------------------------------

    /// <summary>
    /// Returns the neighbor index (0, 1, or 2) opposed to the given vertex index.
    /// Corresponds to C++ <c>CDT::opoNbr()</c>.
    /// </summary>
    /// <param name="vertexIndex">Vertex index (0, 1, or 2).</param>
    /// <returns>The neighbor index opposed to the vertex.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int OpposedNeighborIndex(int vertexIndex) => vertexIndex switch
    {
        0 => 1,
        1 => 2,
        _ => 0,
    };

    /// <summary>
    /// Returns the vertex index (0, 1, or 2) opposed to the given neighbor index.
    /// Corresponds to C++ <c>CDT::opoVrt()</c>.
    /// </summary>
    /// <param name="neighborIndex">Neighbor index (0, 1, or 2).</param>
    /// <returns>The vertex index opposed to the neighbor.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int OpposedVertexIndex(int neighborIndex) => neighborIndex switch
    {
        0 => 2,
        1 => 0,
        _ => 1,
    };

    /// <summary>
    /// Returns the local index (0, 1, or 2) of vertex <paramref name="v"/> within triangle <paramref name="t"/>.
    /// Corresponds to C++ <c>CDT::vertexInd()</c>.
    /// </summary>
    /// <param name="t">The triangle to search.</param>
    /// <param name="v">The vertex index to find.</param>
    /// <returns>The local vertex index (0, 1, or 2) within the triangle.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int VertexIndex(in Triangle t, int v)
    {
        if (t.V0 == v) return 0;
        if (t.V1 == v) return 1;
        return 2;
    }

    /// <summary>
    /// Returns the local neighbor index (0, 1, or 2) of the edge with vertices
    /// <paramref name="va"/> and <paramref name="vb"/> in triangle <paramref name="t"/>.
    /// Corresponds to C++ <c>CDT::edgeNeighborInd()</c>.
    /// </summary>
    /// <param name="t">The triangle containing the edge.</param>
    /// <param name="va">First vertex of the edge.</param>
    /// <param name="vb">Second vertex of the edge.</param>
    /// <returns>The neighbor-slot index (0, 1, or 2) for the edge.</returns>
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

    /// <summary>
    /// Returns the neighbor-slot index for the edge shared between the triangle <paramref name="t"/>
    /// and the neighbor at index <paramref name="iTopo"/>.
    /// Corresponds to C++ <c>CDT::opposedVertexInd()</c>.
    /// </summary>
    /// <param name="t">The triangle to query.</param>
    /// <param name="iTopo">The triangle index of the neighbor to find.</param>
    /// <returns>The local neighbor index (0, 1, or 2) within the triangle.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int NeighborIndex(in Triangle t, int iTopo)
    {
        if (t.N0 == iTopo) return 0;
        if (t.N1 == iTopo) return 1;
        return 2;
    }

    /// <summary>
    /// Returns the vertex index in <paramref name="topo"/> that is opposed to (not shared with) triangle <paramref name="iT"/>.
    /// Corresponds to C++ <c>CDT::opposedTriangleInd()</c>.
    /// </summary>
    /// <param name="topo">The triangle to query.</param>
    /// <param name="iT">The triangle index of the neighbor.</param>
    /// <returns>The vertex index (global) opposed to the shared edge with <paramref name="iT"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int OpposedVertex(in Triangle topo, int iT)
        => topo.GetVertex(OpposedVertexIndex(NeighborIndex(topo, iT)));

    /// <summary>
    /// Returns the triangle index of the neighbor of <paramref name="t"/> opposed to vertex <paramref name="iV"/>.
    /// Corresponds to C++ <c>CDT::opposedTriangle()</c>.
    /// </summary>
    /// <param name="t">The triangle to query.</param>
    /// <param name="iV">The vertex index whose opposite neighbor is sought.</param>
    /// <returns>The triangle index of the neighbor opposed to the vertex.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int OpposedTriangle(in Triangle t, int iV)
        => t.GetNeighbor(OpposedNeighborIndex(VertexIndex(t, iV)));

    /// <summary>
    /// Returns the triangle index adjacent to edge (<paramref name="va"/>, <paramref name="vb"/>) in triangle <paramref name="t"/>.
    /// </summary>
    /// <param name="t">The triangle containing the edge.</param>
    /// <param name="va">First vertex of the edge.</param>
    /// <param name="vb">Second vertex of the edge.</param>
    /// <returns>The index of the neighboring triangle on the given edge.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int EdgeNeighbor(in Triangle t, int va, int vb)
        => t.GetNeighbor(EdgeNeighborIndex(t, va, vb));

    // -------------------------------------------------------------------------
    // Super-triangle
    // -------------------------------------------------------------------------

    /// <summary>
    /// Returns <c>true</c> if any vertex of triangle <paramref name="t"/> is one of the three
    /// super-triangle vertices (indices 0, 1, 2).
    /// </summary>
    /// <param name="t">The triangle to check.</param>
    /// <returns><c>true</c> if the triangle touches the super-triangle; otherwise <c>false</c>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TouchesSuperTriangle(in Triangle t)
        => t.V0 < Indices.SuperTriangleVertexCount
        || t.V1 < Indices.SuperTriangleVertexCount
        || t.V2 < Indices.SuperTriangleVertexCount;

    // -------------------------------------------------------------------------
    // Point location
    // -------------------------------------------------------------------------

    /// <summary>
    /// Orient2d wrapper using <see cref="PredicatesAdaptive.Orient2d(double, double, double, double, double, double)"/>.
    /// Returns the signed area determinant of the triangle formed by <paramref name="p"/>, <paramref name="v1"/>, <paramref name="v2"/>.
    /// Positive = p is to the left of v1→v2; zero = collinear; negative = to the right.
    /// Corresponds to C++ <c>CDT::orient2D()</c>.
    /// </summary>
    /// <typeparam name="T">Floating-point coordinate type (<see cref="double"/> or <see cref="float"/>).</typeparam>
    /// <param name="p">The query point.</param>
    /// <param name="v1">Start point of the directed line.</param>
    /// <param name="v2">End point of the directed line.</param>
    /// <returns>
    /// Positive if <paramref name="p"/> is to the left of v1→v2;
    /// zero if collinear; negative if to the right.
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static T Orient2D<T>(V2d<T> p, V2d<T> v1, V2d<T> v2)
        where T : unmanaged, IFloatingPoint<T>
    {
        if (typeof(T) == typeof(double))
        {
            double r = PredicatesAdaptive.Orient2d(
                Unsafe.As<T, double>(ref Unsafe.AsRef(in v1.X)), Unsafe.As<T, double>(ref Unsafe.AsRef(in v1.Y)),
                Unsafe.As<T, double>(ref Unsafe.AsRef(in v2.X)), Unsafe.As<T, double>(ref Unsafe.AsRef(in v2.Y)),
                Unsafe.As<T, double>(ref Unsafe.AsRef(in p.X)), Unsafe.As<T, double>(ref Unsafe.AsRef(in p.Y)));
            return Unsafe.As<double, T>(ref r);
        }
        else
        {
            float r = PredicatesAdaptive.Orient2d(
                Unsafe.As<T, float>(ref Unsafe.AsRef(in v1.X)), Unsafe.As<T, float>(ref Unsafe.AsRef(in v1.Y)),
                Unsafe.As<T, float>(ref Unsafe.AsRef(in v2.X)), Unsafe.As<T, float>(ref Unsafe.AsRef(in v2.Y)),
                Unsafe.As<T, float>(ref Unsafe.AsRef(in p.X)), Unsafe.As<T, float>(ref Unsafe.AsRef(in p.Y)));
            return Unsafe.As<float, T>(ref r);
        }
    }

    /// <summary>
    /// Classifies the orientation value returned by <see cref="Orient2D{T}"/> using the given tolerance.
    /// Returns <see cref="PtLineLocation.Left"/> if orientation > tolerance,
    /// <see cref="PtLineLocation.Right"/> if orientation &lt; -tolerance,
    /// or <see cref="PtLineLocation.OnLine"/> otherwise.
    /// Corresponds to C++ <c>CDT::classifyOrientation()</c>.
    /// </summary>
    /// <typeparam name="T">Floating-point type.</typeparam>
    /// <param name="orientation">The orientation determinant value.</param>
    /// <param name="orientationTolerance">Tolerance for on-line classification (default: zero).</param>
    /// <returns>The classified point-line location.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static PtLineLocation ClassifyOrientation<T>(T orientation, T orientationTolerance = default!)
        where T : IFloatingPoint<T>
    {
        if (orientation < -orientationTolerance) return PtLineLocation.Right;
        if (orientation > orientationTolerance) return PtLineLocation.Left;
        return PtLineLocation.OnLine;
    }

    /// <summary>
    /// Determines whether point <paramref name="p"/> lies to the left of, on, or to the right of
    /// the directed line from <paramref name="v1"/> to <paramref name="v2"/>.
    /// Corresponds to C++ <c>CDT::locatePointLine()</c>.
    /// </summary>
    /// <typeparam name="T">Floating-point coordinate type.</typeparam>
    /// <param name="p">The query point.</param>
    /// <param name="v1">Start point of the directed line.</param>
    /// <param name="v2">End point of the directed line.</param>
    /// <param name="orientationTolerance">Tolerance for on-line classification (default: zero).</param>
    /// <returns>The location of <paramref name="p"/> relative to the directed line.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static PtLineLocation LocatePointLine<T>(V2d<T> p, V2d<T> v1, V2d<T> v2, T orientationTolerance = default)
        where T : unmanaged, IFloatingPoint<T>
        => ClassifyOrientation(Orient2D(p, v1, v2), orientationTolerance);

    /// <summary>
    /// Determines whether point <paramref name="p"/> is inside, outside, on an edge of, or on a vertex of
    /// the triangle defined by (<paramref name="v1"/>, <paramref name="v2"/>, <paramref name="v3"/>) in CCW order.
    /// Corresponds to C++ <c>CDT::locatePointTriangle()</c>.
    /// </summary>
    /// <typeparam name="T">Floating-point coordinate type.</typeparam>
    /// <param name="p">The query point.</param>
    /// <param name="v1">First triangle vertex.</param>
    /// <param name="v2">Second triangle vertex.</param>
    /// <param name="v3">Third triangle vertex.</param>
    /// <returns>The location of <paramref name="p"/> relative to the triangle.</returns>
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
    /// Returns <c>true</c> if <paramref name="location"/> represents a position on any of the three edges
    /// of a triangle (OnEdge0, OnEdge1, or OnEdge2).
    /// Corresponds to C++ <c>CDT::isOnEdge()</c>.
    /// </summary>
    /// <param name="location">The triangle location to test.</param>
    /// <returns><c>true</c> if the location is on an edge; otherwise <c>false</c>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsOnEdge(PtTriLocation location)
        => location is PtTriLocation.OnEdge0
                    or PtTriLocation.OnEdge1
                    or PtTriLocation.OnEdge2;

    /// <summary>
    /// Returns the neighbor-slot index (0, 1, or 2) corresponding to the edge indicated by <paramref name="location"/>.
    /// Only valid when <see cref="IsOnEdge"/> returns <c>true</c> for <paramref name="location"/>.
    /// Corresponds to C++ <c>CDT::edgeNeighbor()</c>.
    /// </summary>
    /// <param name="location">An on-edge location value.</param>
    /// <returns>The neighbor-slot index (0, 1, or 2).</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int EdgeNeighborFromLocation(PtTriLocation location)
        => (int)(location - PtTriLocation.OnEdge0);

    // -------------------------------------------------------------------------
    // Circumcircle / distance / intersection
    // -------------------------------------------------------------------------

    /// <summary>
    /// Returns <c>true</c> if point <paramref name="p"/> lies strictly inside the circumcircle
    /// of the triangle (<paramref name="v1"/>, <paramref name="v2"/>, <paramref name="v3"/>).
    /// Uses <see cref="PredicatesAdaptive.InCircle(double, double, double, double, double, double, double, double)"/> internally.
    /// Corresponds to C++ <c>CDT::isInCircumcircle()</c>.
    /// </summary>
    /// <typeparam name="T">Floating-point coordinate type.</typeparam>
    /// <param name="p">The query point.</param>
    /// <param name="v1">First triangle vertex (CCW).</param>
    /// <param name="v2">Second triangle vertex (CCW).</param>
    /// <param name="v3">Third triangle vertex (CCW).</param>
    /// <returns><c>true</c> if the point is inside the circumcircle; otherwise <c>false</c>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool IsInCircumcircle<T>(V2d<T> p, V2d<T> v1, V2d<T> v2, V2d<T> v3)
        where T : unmanaged, IFloatingPoint<T>
    {
        if (typeof(T) == typeof(double))
            return PredicatesAdaptive.InCircle(
                Unsafe.As<T, double>(ref Unsafe.AsRef(in v1.X)), Unsafe.As<T, double>(ref Unsafe.AsRef(in v1.Y)),
                Unsafe.As<T, double>(ref Unsafe.AsRef(in v2.X)), Unsafe.As<T, double>(ref Unsafe.AsRef(in v2.Y)),
                Unsafe.As<T, double>(ref Unsafe.AsRef(in v3.X)), Unsafe.As<T, double>(ref Unsafe.AsRef(in v3.Y)),
                Unsafe.As<T, double>(ref Unsafe.AsRef(in p.X)), Unsafe.As<T, double>(ref Unsafe.AsRef(in p.Y))) > 0.0;
        else
            return PredicatesAdaptive.InCircle(
                Unsafe.As<T, float>(ref Unsafe.AsRef(in v1.X)), Unsafe.As<T, float>(ref Unsafe.AsRef(in v1.Y)),
                Unsafe.As<T, float>(ref Unsafe.AsRef(in v2.X)), Unsafe.As<T, float>(ref Unsafe.AsRef(in v2.Y)),
                Unsafe.As<T, float>(ref Unsafe.AsRef(in v3.X)), Unsafe.As<T, float>(ref Unsafe.AsRef(in v3.Y)),
                Unsafe.As<T, float>(ref Unsafe.AsRef(in p.X)), Unsafe.As<T, float>(ref Unsafe.AsRef(in p.Y))) > 0f;
    }

    /// <summary>
    /// Computes the squared Euclidean distance between two points.
    /// </summary>
    /// <typeparam name="T">Floating-point coordinate type.</typeparam>
    /// <param name="a">First point.</param>
    /// <param name="b">Second point.</param>
    /// <returns>The squared distance between <paramref name="a"/> and <paramref name="b"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static T DistanceSquared<T>(V2d<T> a, V2d<T> b)
        where T : IFloatingPoint<T>
    {
        T dx = b.X - a.X;
        T dy = b.Y - a.Y;
        return dx * dx + dy * dy;
    }

    /// <summary>
    /// Computes the Euclidean distance between two points.
    /// </summary>
    /// <typeparam name="T">Floating-point coordinate type.</typeparam>
    /// <param name="a">First point.</param>
    /// <param name="b">Second point.</param>
    /// <returns>The Euclidean distance between <paramref name="a"/> and <paramref name="b"/>.</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static T Distance<T>(V2d<T> a, V2d<T> b)
        where T : IFloatingPoint<T>, IRootFunctions<T>
        => T.Sqrt(DistanceSquared(a, b));

    /// <summary>
    /// Computes the intersection point of line segments a–b and c–d.
    /// Precondition: the segments must intersect.
    /// Corresponds to C++ <c>CDT::detail::intersectionPosition()</c>.
    /// </summary>
    /// <typeparam name="T">Floating-point coordinate type.</typeparam>
    /// <param name="a">First endpoint of segment a–b.</param>
    /// <param name="b">Second endpoint of segment a–b.</param>
    /// <param name="c">First endpoint of segment c–d.</param>
    /// <param name="d">Second endpoint of segment c–d.</param>
    /// <returns>The intersection point of the two segments.</returns>
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

    // -------------------------------------------------------------------------
    // Adjacency helpers (internal use)
    // -------------------------------------------------------------------------

    /// <summary>Tests whether any pair of triangles (given as vertex triangle lists) share an edge.</summary>
    /// <param name="aTris">Triangle indices adjacent to vertex A.</param>
    /// <param name="bTris">Triangle indices adjacent to vertex B.</param>
    /// <returns><c>true</c> if any triangle is shared between the two lists.</returns>
    internal static bool VerticesShareEdge(List<int> aTris, List<int> bTris)
    {
        foreach (int t in aTris)
        {
            if (bTris.Contains(t)) return true;
        }
        return false;
    }
}

/// <summary>
/// Struct key for use in dictionaries that avoids the overhead of
/// <see cref="System.ValueTuple{T1,T2}"/>'s equality implementation.
/// Implementing <see cref="IEquatable{T}"/> allows
/// <see cref="EqualityComparer{T}.Default"/> to call <c>Equals</c> directly
/// without virtual dispatch.
/// </summary>
internal readonly struct V2dKey<T>(T x, T y) : IEquatable<V2dKey<T>>
    where T : IFloatingPoint<T>
{
    private readonly T _x = x;
    private readonly T _y = y;

    public bool Equals(V2dKey<T> other) => _x == other._x && _y == other._y;
    public override bool Equals(object? obj) => obj is V2dKey<T> k && Equals(k);
    public override int GetHashCode() => HashCode.Combine(_x, _y);
}
