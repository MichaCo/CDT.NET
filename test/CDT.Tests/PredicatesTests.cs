// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

namespace CDT.Tests;

/// <summary>Tests for geometric predicates and utilities.</summary>
public sealed class PredicatesTests
{
    [Fact]
    public void Orient2D_CounterClockwise_ReturnsPositive()
    {
        // v1=(0,0), v2=(1,0), p=(0.5,1) should be to the left → positive
        double r = Predicates.Orient2D(0.0, 0.0, 1.0, 0.0, 0.5, 1.0);
        Assert.True(r > 0);
    }

    [Fact]
    public void Orient2D_Clockwise_ReturnsNegative()
    {
        double r = Predicates.Orient2D(0.0, 0.0, 1.0, 0.0, 0.5, -1.0);
        Assert.True(r < 0);
    }

    [Fact]
    public void Orient2D_Collinear_ReturnsZero()
    {
        double r = Predicates.Orient2D(0.0, 0.0, 1.0, 0.0, 0.5, 0.0);
        Assert.Equal(0.0, r, 15);
    }

    [Fact]
    public void InCircle_InsideCircle_ReturnsPositive()
    {
        // Triangle: (0,0),(4,0),(2,4) - circumcircle contains (2,1)
        double r = Predicates.InCircle(0, 0, 4, 0, 2, 4, 2, 1);
        Assert.True(r > 0);
    }

    [Fact]
    public void InCircle_OutsideCircle_ReturnsNegative()
    {
        // (2,-2) should be well outside the circumcircle of (0,0),(4,0),(2,4)
        double r = Predicates.InCircle(0, 0, 4, 0, 2, 4, 2, -2);
        Assert.True(r < 0);
    }
}

/// <summary>Tests for triangle utils.</summary>
public sealed class TriangleUtilsTests
{
    [Theory]
    [InlineData(0, 1)]
    [InlineData(1, 2)]
    [InlineData(2, 0)]
    public void Ccw_CyclesForward(int i, int expected)
        => Assert.Equal(expected, TriangleUtils.Ccw(i));

    [Theory]
    [InlineData(0, 2)]
    [InlineData(1, 0)]
    [InlineData(2, 1)]
    public void Cw_CyclesBackward(int i, int expected)
        => Assert.Equal(expected, TriangleUtils.Cw(i));

    [Fact]
    public void LocatePointTriangle_Inside()
    {
        var p = new V2d<double>(0.5, 0.25);
        var v1 = new V2d<double>(0, 0);
        var v2 = new V2d<double>(1, 0);
        var v3 = new V2d<double>(0, 1);
        Assert.Equal(PtTriLocation.Inside, TriangleUtils.LocatePointTriangle(p, v1, v2, v3));
    }

    [Fact]
    public void LocatePointTriangle_Outside()
    {
        var p = new V2d<double>(2, 2);
        var v1 = new V2d<double>(0, 0);
        var v2 = new V2d<double>(1, 0);
        var v3 = new V2d<double>(0, 1);
        Assert.Equal(PtTriLocation.Outside, TriangleUtils.LocatePointTriangle(p, v1, v2, v3));
    }

    [Fact]
    public void LocatePointTriangle_OnEdge()
    {
        // Point on edge v1-v2
        var p = new V2d<double>(0.5, 0);
        var v1 = new V2d<double>(0, 0);
        var v2 = new V2d<double>(1, 0);
        var v3 = new V2d<double>(0, 1);
        var loc = TriangleUtils.LocatePointTriangle(p, v1, v2, v3);
        Assert.True(TriangleUtils.IsOnEdge(loc));
    }

    [Fact]
    public void IsInCircumcircle_PointInside_ReturnsTrue()
    {
        var p = new V2d<double>(0.5, 0.5);
        var v1 = new V2d<double>(0, 0);
        var v2 = new V2d<double>(2, 0);
        var v3 = new V2d<double>(1, 2);
        Assert.True(TriangleUtils.IsInCircumcircle(p, v1, v2, v3));
    }

    [Fact]
    public void DistanceSquared_CorrectResult()
    {
        var a = new V2d<double>(0, 0);
        var b = new V2d<double>(3, 4);
        Assert.Equal(25.0, TriangleUtils.DistanceSquared(a, b), 12);
    }
}

/// <summary>Tests for edge type.</summary>
public sealed class EdgeTests
{
    [Fact]
    public void Edge_NormalizesOrder()
    {
        var e1 = new Edge(3, 1);
        var e2 = new Edge(1, 3);
        Assert.Equal(e1, e2);
        Assert.Equal(1, e1.V1);
        Assert.Equal(3, e1.V2);
    }

    [Fact]
    public void Edge_HashConsistency()
    {
        var e1 = new Edge(5, 2);
        var e2 = new Edge(2, 5);
        Assert.Equal(e1.GetHashCode(), e2.GetHashCode());
    }

    [Fact]
    public void EdgeSet_TreatsSymmetricEdgesAsSame()
    {
        var set = new HashSet<Edge> { new Edge(1, 2) };
        Assert.Contains(new Edge(2, 1), set);
    }
}
