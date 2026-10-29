// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

namespace CDT.Tests;

public sealed class RefinementTests
{
    [Fact]
    public void RefineTriangles_SatisfiesSmallestAngleThreshold()
    {
        var cdt = CreateRectangle(100, 1);
        var toErase = cdt.CollectOuterTrianglesAndHoles();
        TRefine(cdt, RefinementCriterion.SmallestAngle, CdtUtils.DegToRad(20.0), 10000, toErase);

        Assert.Empty(cdt.FindEncroachedFixedEdges());
        cdt.FinalizeTriangulation(toErase);

        Assert.True(TopologyVerifier.VerifyTopology(cdt));
        Assert.True(TopologyVerifier.VerifyWinding(cdt));
        Assert.True(cdt.Triangles.Length > 2);
        Assert.Empty(cdt.FindUnrefinedTriangles(
            RefinementCriterion.SmallestAngle, CdtUtils.DegToRad(20.0)));
    }

    [Fact]
    public void RefineTriangles_SatisfiesLargestAreaThreshold()
    {
        var cdt = CreateRectangle(10, 10);
        var toErase = cdt.CollectOuterTrianglesAndHoles();
        TRefine(cdt, RefinementCriterion.LargestArea, 1.0, 1000, toErase);

        Assert.Empty(cdt.FindEncroachedFixedEdges());
        cdt.FinalizeTriangulation(toErase);

        Assert.True(TopologyVerifier.VerifyTopology(cdt));
        Assert.True(TopologyVerifier.VerifyWinding(cdt));
        Assert.Empty(cdt.FindUnrefinedTriangles(RefinementCriterion.LargestArea, 1.0));
    }

    [Fact]
    public void RefineTriangles_ZeroAngleThresholdIsNoOp()
    {
        var cdt = CreateRectangle(10, 10);
        int verticesBefore = cdt.Vertices.Length;

        var unrefined = cdt.RefineTriangles(
            100, RefinementCriterion.SmallestAngle, 0.0);

        Assert.Equal(verticesBefore, cdt.Vertices.Length);
        Assert.Equal(default, unrefined);
    }

    [Fact]
    public void RefineTriangles_DoesNotExpandConvexHull()
    {
        var cdt = new Triangulation<double>();
        cdt.InsertVertices([new(0, 0), new(10, 0), new(1, 1)]);
        int verticesBefore = cdt.Vertices.Length;

        var unrefined = cdt.RefineTriangles(
            10, RefinementCriterion.SmallestAngle, CdtUtils.DegToRad(20.0));

        Assert.Equal(verticesBefore, cdt.Vertices.Length);
        Assert.True(unrefined.CircumcenterOutside > 0);
    }

    [Fact]
    public void RefineTriangles_RespectsVertexBudget()
    {
        var cdt = CreateRectangle(100, 1);
        int verticesBefore = cdt.Vertices.Length;

        cdt.RefineTriangles(
            10, RefinementCriterion.SmallestAngle, CdtUtils.DegToRad(20.0));

        Assert.InRange(cdt.Vertices.Length, verticesBefore, verticesBefore + 10);
        Assert.True(TopologyVerifier.VerifyTopology(cdt));
        Assert.True(TopologyVerifier.VerifyWinding(cdt));
    }

    [Fact]
    public void RefineTriangles_ReportsUnrefinableSharpFixedCorner()
    {
        double angle = CdtUtils.DegToRad(10.0);
        var cdt = new Triangulation<double>();
        cdt.InsertVertices(
        [
            new V2d<double>(0, 0),
            new V2d<double>(8, 0),
            new V2d<double>(4, 5),
            new V2d<double>(Math.Cos(angle), Math.Sin(angle)),
        ]);
        cdt.InsertEdges(
        [
            new Edge(0, 1), new Edge(1, 2), new Edge(2, 3), new Edge(3, 0),
        ]);

        var unrefined = cdt.RefineTriangles(
            100, RefinementCriterion.SmallestAngle, CdtUtils.DegToRad(20.0),
            minEdgeLength: 0.05);

        Assert.Equal(1, unrefined.SharpFixedCorner);
        Assert.True(TopologyVerifier.VerifyTopology(cdt));
        Assert.True(TopologyVerifier.VerifyWinding(cdt));
    }

    [Fact]
    public void RefineTriangles_RejectsInvalidSplitVertexWithoutCorruptingTopology()
    {
        var cdt = new Triangulation<double>(
            VertexInsertionOrder.Auto,
            IntersectingConstraintEdges.TryResolve,
            1e-8);
        cdt.InsertVertices(
        [
            new(7.6, 2.2), new(2, 1.2), new(3.4, 0.4), new(1.4, 1.4),
            new(6.6, 6.2), new(3.8, 0.2), new(0.6, 5.8), new(2.8, 4),
            new(3.6, 7.4), new(4.6, 4.8), new(1, 2.8),
        ]);
        cdt.InsertEdges(
        [
            new(4, 7), new(0, 5), new(3, 6), new(1, 8), new(1, 5),
            new(3, 5), new(2, 4), new(8, 10), new(0, 2), new(2, 5),
        ]);

        var unrefined = cdt.RefineTriangles(
            300, RefinementCriterion.SmallestAngle, CdtUtils.DegToRad(25.0),
            minEdgeLength: 1e-6);

        Assert.True(unrefined.SplitVertexInvalid > 0);
        Assert.True(TopologyVerifier.VerifyTopology(cdt));
        Assert.True(TopologyVerifier.VerifyWinding(cdt));
    }

    [Fact]
    public void CollectAndRefineMethods_RejectFinalizedTriangulation()
    {
        var cdt = CreateRectangle(10, 10);
        cdt.EraseOuterTrianglesAndHoles();

        Assert.Throws<TriangulationFinalizedException>(() => cdt.CollectSuperTriangle());
        Assert.Throws<TriangulationFinalizedException>(() => cdt.CollectOuterTriangles());
        Assert.Throws<TriangulationFinalizedException>(() => cdt.CollectOuterTrianglesAndHoles());
        Assert.Throws<TriangulationFinalizedException>(() => cdt.FinalizeTriangulation(new HashSet<int>()));
        Assert.Throws<TriangulationFinalizedException>(() => cdt.FindEncroachedFixedEdges());
        Assert.Throws<TriangulationFinalizedException>(() => cdt.RefineTriangles(100));
    }

    private static Triangulation<double> CreateRectangle(double width, double height)
    {
        var cdt = new Triangulation<double>();
        cdt.InsertVertices(
        [
            new(0, 0), new(width, 0), new(width, height), new(0, height),
        ]);
        cdt.InsertEdges(
        [
            new(0, 1), new(1, 2), new(2, 3), new(3, 0),
        ]);
        return cdt;
    }

    private static void TRefine(
        Triangulation<double> cdt,
        RefinementCriterion criterion,
        double threshold,
        int budget,
        HashSet<int> toErase)
        => cdt.RefineTriangles(budget, criterion, threshold, toErase);
}
