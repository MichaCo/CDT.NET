// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

namespace CDT.Tests;

/// <summary>Validates that the README code examples compile and produce the expected results.</summary>
public sealed class ReadmeExamplesTests
{
    // -------------------------------------------------------------------------
    // Example 1 – Delaunay triangulation without constraints (convex hull)
    // -------------------------------------------------------------------------

    [Fact]
    public void Example_DelaunayConvexHull()
    {
        var vertices = new List<V2d<double>>
        {
            new(0, 0), new(4, 0), new(4, 4), new(0, 4), new(2, 2), // inner point
        };

        var cdt = new Triangulation<double>();
        cdt.InsertVertices(vertices);
        cdt.EraseSuperTriangle(); // produces convex hull

        Assert.True(TopologyVerifier.VerifyTopology(cdt));
        Assert.Equal(5, cdt.Vertices.Length);
        Assert.True(cdt.Triangles.Length > 0);
        Assert.Empty(cdt.FixedEdges);
    }

    // -------------------------------------------------------------------------
    // Example 2 – Constrained Delaunay triangulation (bounded domain)
    // -------------------------------------------------------------------------

    [Fact]
    public void Example_ConstrainedDelaunay_BoundedDomain()
    {
        var vertices = new List<V2d<double>>
        {
            new(0, 0), new(4, 0), new(4, 4), new(0, 4),
        };
        var edges = new List<Edge>
        {
            new(0, 1), new(1, 2), new(2, 3), new(3, 0), // square boundary
        };

        var cdt = new Triangulation<double>();
        cdt.InsertVertices(vertices);
        cdt.InsertEdges(edges);
        cdt.EraseOuterTriangles(); // removes everything outside the boundary

        Assert.True(TopologyVerifier.VerifyTopology(cdt));
        Assert.Equal(4, cdt.Vertices.Length);
        Assert.Equal(2, cdt.Triangles.Length);
        Assert.Equal(4, cdt.FixedEdges.Count);
    }

    // -------------------------------------------------------------------------
    // Example 3 – Auto-detect outer triangles and holes
    // -------------------------------------------------------------------------

    [Fact]
    public void Example_AutoDetectBoundariesAndHoles()
    {
        // Outer square (vertices 0-3) + inner square hole (vertices 4-7)
        var vertices = new List<V2d<double>>
        {
            new(0, 0), new(6, 0), new(6, 6), new(0, 6), // outer square
            new(2, 2), new(4, 2), new(4, 4), new(2, 4), // inner hole
        };
        var edges = new List<Edge>
        {
            // outer boundary (CCW)
            new(0, 1), new(1, 2), new(2, 3), new(3, 0),
            // inner hole boundary (CW — opposite winding marks it as a hole)
            new(4, 7), new(7, 6), new(6, 5), new(5, 4),
        };

        var cdt = new Triangulation<double>();
        cdt.InsertVertices(vertices);
        cdt.InsertEdges(edges);
        cdt.EraseOuterTrianglesAndHoles(); // removes outer AND fills holes automatically

        Assert.True(TopologyVerifier.VerifyTopology(cdt));
        Assert.Equal(8, cdt.Vertices.Length);
        Assert.True(cdt.Triangles.Length > 0);
    }

    // -------------------------------------------------------------------------
    // Example 4 – Conforming Delaunay triangulation
    // -------------------------------------------------------------------------

    [Fact]
    public void Example_ConformingDelaunay()
    {
        var vertices = new List<V2d<double>>
        {
            new(0, 0), new(4, 0), new(4, 4), new(0, 4),
        };
        var edges = new List<Edge>
        {
            new(0, 1), new(1, 2), new(2, 3), new(3, 0),
        };

        var cdt = new Triangulation<double>();
        cdt.InsertVertices(vertices);
        cdt.ConformToEdges(edges); // may split edges and add new points
        cdt.EraseOuterTriangles();

        Assert.True(TopologyVerifier.VerifyTopology(cdt));
        Assert.True(cdt.Triangles.Length > 0);
        // ConformToEdges may have added midpoints, so vertex count >= 4
        Assert.True(cdt.Vertices.Length >= 4);
    }

    // -------------------------------------------------------------------------
    // Example 5 – Removing duplicates and remapping edges
    // -------------------------------------------------------------------------

    [Fact]
    public void Example_RemoveDuplicatesAndRemapEdges()
    {
        var vertices = new List<V2d<double>>
        {
            new(0, 0), new(4, 0), new(4, 4), new(0, 4),
            new(0, 0), // duplicate of vertex 0
        };
        var edges = new List<Edge>
        {
            new(0, 4), // references duplicate; will be remapped to (0, 0)
            new(1, 2),
        };

        CdtUtils.RemoveDuplicatesAndRemapEdges(vertices, edges);

        // Duplicate removed → 4 unique vertices
        Assert.Equal(4, vertices.Count);
        // Edge (0,4) remapped: both map to index 0 → self-edge (0,0)
        Assert.Equal(new Edge(0, 0), edges[0]);
        // Edge (1,2) unchanged
        Assert.Equal(new Edge(1, 2), edges[1]);

        var cdt = new Triangulation<double>();
        cdt.InsertVertices(vertices);
        cdt.InsertEdges(edges.Where(e => e.V1 != e.V2).ToList()); // skip degenerate self-edge
        cdt.EraseSuperTriangle();

        Assert.True(TopologyVerifier.VerifyTopology(cdt));
    }

    // -------------------------------------------------------------------------
    // Example 6 – Extract all edges from a triangulation
    // -------------------------------------------------------------------------

    [Fact]
    public void Example_ExtractEdgesFromTriangles()
    {
        var cdt = new Triangulation<double>();
        cdt.InsertVertices([new(0, 0), new(2, 0), new(1, 2)]);
        cdt.EraseSuperTriangle();

        // Extract all unique edges from every triangle
        HashSet<Edge> allEdges = CdtUtils.ExtractEdgesFromTriangles(cdt.Triangles.Span);

        // A single triangle has exactly 3 edges
        Assert.Equal(3, allEdges.Count);
    }

    // -------------------------------------------------------------------------
    // Example 7 – Resolve intersecting constraint edges
    // -------------------------------------------------------------------------

    [Fact]
    public void Example_ResolveIntersectingConstraints()
    {
        // Two edges that cross each other: (0,2) and (1,3) on a unit square
        var vertices = new List<V2d<double>>
        {
            new(0, 0), new(1, 0), new(1, 1), new(0, 1),
        };
        var edges = new List<Edge>
        {
            new(0, 2), // diagonal
            new(1, 3), // other diagonal — intersects (0,2)
        };

        // TryResolve splits intersecting edges by inserting the intersection point
        var cdt = new Triangulation<double>(
            VertexInsertionOrder.Auto,
            IntersectingConstraintEdges.TryResolve,
            0.0);

        cdt.InsertVertices(vertices);
        cdt.InsertEdges(edges);
        cdt.EraseSuperTriangle();

        Assert.True(TopologyVerifier.VerifyTopology(cdt));
        // An extra vertex is added at the intersection
        Assert.True(cdt.Vertices.Length > 4);
    }
}
