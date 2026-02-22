// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System.Runtime.InteropServices;

namespace CDT.Tests;

/// <summary>Basic triangulation tests (float and double).</summary>
public abstract class TriangulationTestsBase<T>
    where T : unmanaged,
              System.Numerics.IFloatingPoint<T>,
              System.Numerics.IMinMaxValue<T>,
              System.Numerics.IRootFunctions<T>
{
    protected static T F(double v) => T.CreateChecked(v);

    protected static V2d<T> Pt(double x, double y) => new(F(x), F(y));

    protected static Triangulation<T> CreateCdt(
        VertexInsertionOrder order = VertexInsertionOrder.Auto)
        => new(order);

    // -------------------------------------------------------------------------
    // Basic insertion
    // -------------------------------------------------------------------------

    [Fact]
    public void InsertFourPoints_TopologicallyValid()
    {
        var cdt = CreateCdt();
        cdt.InsertVertices([Pt(0, 0), Pt(1, 1), Pt(3, 1), Pt(3, 0)]);

        Assert.True(TopologyVerifier.VerifyTopology(cdt));
        // 4 user vertices + 3 super-triangle vertices
        Assert.Equal(7, cdt.Vertices.Count);
        Assert.Empty(cdt.FixedEdges);
        // 4 vertices inside a super-triangle → 9 triangles
        Assert.Equal(9, cdt.Triangles.Count);
    }

    [Fact]
    public void EraseSuperTriangle_LeavesConvexHull()
    {
        var cdt = CreateCdt();
        cdt.InsertVertices([Pt(0, 0), Pt(1, 1), Pt(3, 1), Pt(3, 0)]);
        cdt.EraseSuperTriangle();

        Assert.True(TopologyVerifier.VerifyTopology(cdt));
        Assert.Equal(4, cdt.Vertices.Count);
        Assert.Equal(2, cdt.Triangles.Count);
    }

    [Fact]
    public void AddConstraintEdge_PresentAfterErase()
    {
        var cdt = CreateCdt();
        cdt.InsertVertices([Pt(0, 0), Pt(1, 1), Pt(3, 1), Pt(3, 0)]);

        var constraint = new Edge(0, 2);
        cdt.InsertEdges([constraint]);
        cdt.EraseSuperTriangle();

        Assert.True(TopologyVerifier.VerifyTopology(cdt));
        Assert.Equal(4, cdt.Vertices.Count);
        Assert.Single(cdt.FixedEdges);
        Assert.Equal(2, cdt.Triangles.Count);
        Assert.Contains(constraint, cdt.FixedEdges);

        var edges = CdtUtils.ExtractEdgesFromTriangles(
            CollectionsMarshal.AsSpan((List<Triangle>)cdt.Triangles));
        Assert.Contains(constraint, edges);
    }

    // -------------------------------------------------------------------------
    // Duplicate detection
    // -------------------------------------------------------------------------

    [Fact]
    public void DuplicateVertex_ThrowsDuplicateVertexException()
    {
        var cdt = CreateCdt();
        var pts = new[] { Pt(0, 0), Pt(1, 0), Pt(0, 0) }; // pt[2] == pt[0]
        Assert.Throws<DuplicateVertexException>(() => cdt.InsertVertices(pts));
    }

    // -------------------------------------------------------------------------
    // Triangle extraction
    // -------------------------------------------------------------------------

    [Fact]
    public void ExtractEdges_ReturnsAllEdges()
    {
        var cdt = CreateCdt();
        cdt.InsertVertices([Pt(0, 0), Pt(1, 0), Pt(0.5, 1)]);
        cdt.EraseSuperTriangle();

        var edges = CdtUtils.ExtractEdgesFromTriangles(
            CollectionsMarshal.AsSpan((List<Triangle>)cdt.Triangles));
        Assert.Equal(3, edges.Count); // one triangle → 3 edges
    }

    // -------------------------------------------------------------------------
    // Square with diagonal constraint
    // -------------------------------------------------------------------------

    [Fact]
    public void Square_WithDiagonalConstraint()
    {
        var cdt = CreateCdt();
        cdt.InsertVertices([Pt(0, 0), Pt(1, 0), Pt(1, 1), Pt(0, 1)]);
        cdt.InsertEdges([new Edge(0, 2)]); // diagonal 0→2
        cdt.EraseSuperTriangle();

        Assert.True(TopologyVerifier.VerifyTopology(cdt));
        Assert.Equal(2, cdt.Triangles.Count);
        Assert.Single(cdt.FixedEdges);
    }

    // -------------------------------------------------------------------------
    // Insertion order variants
    // -------------------------------------------------------------------------

    [Fact]
    public void InsertionOrder_AsProvided_SameResult()
    {
        var pts = new[] { Pt(0, 0), Pt(1, 0), Pt(1, 1), Pt(0, 1) };

        var cdtAuto = new Triangulation<T>(VertexInsertionOrder.Auto);
        cdtAuto.InsertVertices(pts);
        cdtAuto.EraseSuperTriangle();

        var cdtProvided = new Triangulation<T>(VertexInsertionOrder.AsProvided);
        cdtProvided.InsertVertices(pts);
        cdtProvided.EraseSuperTriangle();

        Assert.Equal(cdtAuto.Triangles.Count, cdtProvided.Triangles.Count);
        Assert.Equal(cdtAuto.Vertices.Count, cdtProvided.Vertices.Count);
    }

    // -------------------------------------------------------------------------
    // Utility functions
    // -------------------------------------------------------------------------

    [Fact]
    public void FindDuplicates_DetectsExactMatches()
    {
        V2d<T>[] pts = [Pt(0, 0), Pt(1, 0), Pt(0, 0), Pt(2, 0)];
        var info = CdtUtils.FindDuplicates<T>(pts);
        Assert.Single(info.Duplicates);
        Assert.Equal(2, info.Duplicates[0]);
        // mapping[2] should equal mapping[0] = 0
        Assert.Equal(info.Mapping[0], info.Mapping[2]);
    }

    [Fact]
    public void RemoveDuplicatesAndRemapEdges_ProducesConsistentResult()
    {
        var verts = new List<V2d<T>>
        {
            Pt(0, 0), Pt(1, 0), Pt(0, 0), // index 2 is dup of 0
            Pt(0, 1),
        };
        var edges = new List<Edge> { new(0, 2), new(1, 3) };

        CdtUtils.RemoveDuplicatesAndRemapEdges(verts, edges);

        // After removing dup at index 2, vertices are: (0,0),(1,0),(0,1) → 3
        Assert.Equal(3, verts.Count);
        // Edge (0,2) should be remapped to (0,0) = same vertex → still (0,0)
        Assert.Equal(new Edge(0, 0), edges[0]);
        // Edge (1,3) → (1,2) after shifting
        Assert.Equal(new Edge(1, 2), edges[1]);
    }

    // -------------------------------------------------------------------------
    // Erase outer triangles
    // -------------------------------------------------------------------------

    [Fact]
    public void EraseOuterTriangles_RemovesOutside()
    {
        var cdt = CreateCdt();
        cdt.InsertVertices([Pt(0, 0), Pt(1, 0), Pt(1, 1), Pt(0, 1)]);
        cdt.InsertEdges([new Edge(0, 1), new Edge(1, 2), new Edge(2, 3), new Edge(3, 0)]);
        cdt.EraseOuterTriangles();

        Assert.True(TopologyVerifier.VerifyTopology(cdt));
        // Square → 2 triangles
        Assert.Equal(2, cdt.Triangles.Count);
    }

    // -------------------------------------------------------------------------
    // Single triangle
    // -------------------------------------------------------------------------

    [Fact]
    public void ThreePoints_SingleTriangle()
    {
        var cdt = CreateCdt();
        cdt.InsertVertices([Pt(0, 0), Pt(2, 0), Pt(1, 2)]);
        cdt.EraseSuperTriangle();

        Assert.True(TopologyVerifier.VerifyTopology(cdt));
        Assert.Equal(3, cdt.Vertices.Count);
        Assert.Single(cdt.Triangles);
    }

    // -------------------------------------------------------------------------
    // Collinear points
    // -------------------------------------------------------------------------

    [Fact]
    public void FivePoints_ValidTopology()
    {
        var cdt = CreateCdt();
        cdt.InsertVertices([
            Pt(0, 0), Pt(2, 0), Pt(4, 0),
            Pt(2, 2), Pt(0, 2),
        ]);
        cdt.EraseSuperTriangle();

        Assert.True(TopologyVerifier.VerifyTopology(cdt));
        Assert.Equal(5, cdt.Vertices.Count);
    }
}

/// <summary>Triangulation tests for <see cref="double"/>.</summary>
public sealed class TriangulationTests_Double : TriangulationTestsBase<double> { }

/// <summary>Triangulation tests for <see cref="float"/>.</summary>
public sealed class TriangulationTests_Float : TriangulationTestsBase<float> { }
