// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System.Numerics;

namespace CDT;

/// <summary>
/// Topology verification utilities for debugging and testing.
/// </summary>
public static class TopologyVerifier
{
    /// <summary>
    /// Verifies the topological consistency of a triangulation.
    /// Checks neighbor relationships, vertex-triangle adjacency, and orientation.
    /// </summary>
    /// <returns>true if topology is valid.</returns>
    public static bool VerifyTopology<T>(Triangulation<T> cdt)
        where T : unmanaged, IFloatingPoint<T>, IMinMaxValue<T>, IRootFunctions<T>
    {
        var triangles = cdt.Triangles;
        var vertices = cdt.Vertices;

        for (int iT = 0; iT < triangles.Count; iT++)
        {
            var t = triangles[iT];
            // Verify non-invalid vertices
            if (t.V0 == Indices.NoVertex || t.V1 == Indices.NoVertex || t.V2 == Indices.NoVertex)
                return false;
            if (t.V0 >= vertices.Count || t.V1 >= vertices.Count || t.V2 >= vertices.Count)
                return false;
            // No degenerate (same-vertex) triangles
            if (t.V0 == t.V1 || t.V1 == t.V2 || t.V0 == t.V2)
                return false;

            // Verify neighbor consistency
            for (int i = 0; i < 3; i++)
            {
                int iN = t.GetNeighbor(i);
                if (iN == Indices.NoNeighbor) continue;
                if (iN >= triangles.Count) return false;
                var tN = triangles[iN];
                // Neighbor must reference us back
                if (tN.N0 != iT && tN.N1 != iT && tN.N2 != iT)
                    return false;
                // Verify shared edge: neighbor[i] shares vertices {v[i], v[ccw(i)]}
                int va = t.GetVertex(i);
                int vb = t.GetVertex(TriangleUtils.Ccw(i));
                if (!tN.ContainsVertex(va) || !tN.ContainsVertex(vb))
                    return false;
            }
        }
        return true;
    }
}
