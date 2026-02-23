using Poly2Tri;
using Poly2Tri.Triangulation;
using Poly2Tri.Triangulation.Sets;

// Test with simple square + center point
var pts5 = new List<TriangulationPoint> {
    new(0.0, 0.0, 0), new(10.0, 0.0, 0), new(10.0, 10.0, 0),
    new(0.0, 10.0, 0), new(5.0, 5.0, 0)
};
var cps5 = new ConstrainedPointSet(pts5);
P2T.Triangulate(cps5, TriangulationAlgorithm.DTSweep);
Console.WriteLine($"5 pts (square+center)   -> {cps5.Triangles.Count} triangles (expected ~4 for full Delaunay)");

// Test with grid of 100 points
var grid = new List<TriangulationPoint>();
for (int i = 0; i < 10; i++)
    for (int j = 0; j < 10; j++)
        grid.Add(new((double)i, (double)j, 0));
var cpsGrid = new ConstrainedPointSet(grid);
P2T.Triangulate(cpsGrid, TriangulationAlgorithm.DTSweep);
Console.WriteLine($"100 pts (10x10 grid)    -> {cpsGrid.Triangles.Count} triangles (expected ~(2*100-5)≈195 for full Delaunay)");

// Now test with PointSet instead
var pts5b = new List<TriangulationPoint> {
    new(0.0, 0.0, 0), new(10.0, 0.0, 0), new(10.0, 10.0, 0),
    new(0.0, 10.0, 0), new(5.0, 5.0, 0)
};
var ps5 = new Poly2Tri.Triangulation.Sets.PointSet(pts5b);
P2T.Triangulate(ps5, TriangulationAlgorithm.DTSweep);
Console.WriteLine($"PointSet 5 pts          -> {ps5.Triangles.Count} triangles");

var gridB = new List<TriangulationPoint>();
for (int i = 0; i < 10; i++)
    for (int j = 0; j < 10; j++)
        gridB.Add(new((double)i, (double)j, 0));
var psGrid = new Poly2Tri.Triangulation.Sets.PointSet(gridB);
P2T.Triangulate(psGrid, TriangulationAlgorithm.DTSweep);
Console.WriteLine($"PointSet 100 pts        -> {psGrid.Triangles.Count} triangles");
