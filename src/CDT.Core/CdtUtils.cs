// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System.Numerics;

namespace CDT;

/// <summary>
/// Duplicates information for vertex deduplication.
/// </summary>
public sealed class DuplicatesInfo
{
    /// <summary>
    /// Maps each original vertex index to its canonical (deduplicated) index.
    /// </summary>
    public int[] Mapping { get; }

    /// <summary>Indices of duplicate vertices in the original list.</summary>
    public List<int> Duplicates { get; }

    /// <inheritdoc/>
    public DuplicatesInfo(int[] mapping, List<int> duplicates)
    {
        Mapping = mapping;
        Duplicates = duplicates;
    }
}

/// <summary>
/// Utility methods for duplicate removal, edge remapping, and edge extraction.
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
    public static DuplicatesInfo FindDuplicates<T>(IReadOnlyList<V2d<T>> vertices)
        where T : IFloatingPoint<T>
    {
        int n = vertices.Count;
        var mapping = new int[n];
        var duplicates = new List<int>();
        var posToIndex = new Dictionary<(T, T), int>(n);

        int iOut = 0;
        for (int iIn = 0; iIn < n; iIn++)
        {
            var key = (vertices[iIn].X, vertices[iIn].Y);
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
    /// <see cref="FindDuplicates{T}"/>.
    /// </summary>
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
    public static void RemapEdges(List<Edge> edges, int[] mapping)
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
    /// Calculates per-vertex triangle lists (triangles adjacent to each vertex).
    /// </summary>
    public static List<int>[] CalculateTrianglesByVertex(
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
        return result;
    }

    /// <summary>
    /// Converts piece→original_edges mapping to original_edge→pieces mapping.
    /// </summary>
    public static Dictionary<Edge, List<Edge>> EdgeToPiecesMapping(
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
        return edgeToPieces;
    }
}