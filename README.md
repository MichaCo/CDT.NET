# CDT.NET

[![Build](https://github.com/MichaCo/CDT.NET/actions/workflows/ci-cd.yml/badge.svg?branch=main)](https://github.com/MichaCo/CDT.NET/actions/workflows/ci-cd.yml)
[![NuGet](https://img.shields.io/nuget/v/CDT.NET.svg)](https://www.nuget.org/packages/CDT.NET)

A C# port of the [artem-ogre/CDT](https://github.com/artem-ogre/CDT) Constrained Delaunay Triangulation library.

> **Credits:** This library is a C# port of the excellent [CDT C++ library](https://github.com/artem-ogre/CDT) by Artem Amirkhanov and contributors, licensed under MPL 2.0.
> For full algorithm documentation, research references, and in-depth API documentation please refer to the [original C++ repository](https://github.com/artem-ogre/CDT) and its [online documentation](https://artem-ogre.github.io/CDT/).

## What is CDT?

CDT is a library for generating **Constrained** and **Conforming** Delaunay Triangulations. It produces triangulations from a set of points and optional boundary/constraint edges. Unlike a plain Delaunay triangulation, CDT guarantees that the constraint edges you specify will appear in the final mesh.

## Features

- **Constrained Delaunay Triangulation** — force specific edges into the triangulation
- **Conforming Delaunay Triangulation** — split edges and add Steiner points until constraint edges are present in the triangulation
- **Convex-hull** triangulation — triangulate all points without any constraints
- **Automatic hole detection** — use `EraseOuterTrianglesAndHoles` to remove outer regions and holes based on even–odd winding depth
- **Robust geometric predicates** — numerically stable orientation and in-circle tests
- **KD-tree spatial indexing** — fast nearest-neighbor lookup during vertex insertion
- **Duplicate handling** — utilities to remove duplicate vertices and remap edges before triangulation
- **Intersecting constraint edges** — optionally resolve by splitting edges at the intersection point
- Multi-target: **.NET 8** and **.NET 10**

**Pre-conditions** (same as the C++ original):
- No duplicate vertices (use `CdtUtils.RemoveDuplicatesAndRemapEdges` to clean input)
- No two constraint edges may intersect (or pass `IntersectingConstraintEdges.TryResolve`)

**Post-conditions:**
- All triangles have **counter-clockwise (CCW) winding** in a coordinate system where X points right and Y points up.

## Installation

```
dotnet add package CDT.NET
```

## Usage

### Delaunay triangulation (convex hull, no constraints)

Insert vertices and call `EraseSuperTriangle` to get the convex-hull triangulation.

```csharp
using CDT;

var vertices = new List<V2d<double>>
{
    new(0, 0), new(4, 0), new(4, 4), new(0, 4), new(2, 2),
};

var cdt = new Triangulation<double>();
cdt.InsertVertices(vertices);
cdt.EraseSuperTriangle(); // produces convex hull

IReadOnlyList<Triangle>    triangles  = cdt.Triangles;
IReadOnlyList<V2d<double>> points     = cdt.Vertices;
HashSet<Edge>              allEdges   = CdtUtils.ExtractEdgesFromTriangles(triangles);
```

### Constrained Delaunay triangulation (bounded domain)

Insert boundary edges, then call `EraseOuterTriangles` to keep only the region inside the boundary.

```csharp
using CDT;

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

IReadOnlyList<Triangle> triangles = cdt.Triangles;
IReadOnlySet<Edge>      fixedEdges = cdt.FixedEdges; // the constraint edges
```

### Auto-detect boundaries and holes

Use `EraseOuterTrianglesAndHoles` to automatically remove the outer region **and** fill holes. The algorithm uses an even–odd depth rule: depth 0 = outside, depth 1 = inside, depth 2 = hole, etc.

```csharp
using CDT;

// Outer square (vertices 0-3) + inner square hole (vertices 4-7)
var vertices = new List<V2d<double>>
{
    new(0, 0), new(6, 0), new(6, 6), new(0, 6), // outer square
    new(2, 2), new(4, 2), new(4, 4), new(2, 4), // inner hole
};
var edges = new List<Edge>
{
    new(0, 1), new(1, 2), new(2, 3), new(3, 0), // outer boundary (CCW)
    new(4, 7), new(7, 6), new(6, 5), new(5, 4), // inner hole (CW — opposite winding)
};

var cdt = new Triangulation<double>();
cdt.InsertVertices(vertices);
cdt.InsertEdges(edges);
cdt.EraseOuterTrianglesAndHoles();

IReadOnlyList<Triangle> triangles = cdt.Triangles;
```

### Conforming Delaunay triangulation

Use `ConformToEdges` instead of `InsertEdges`. The algorithm may split constraint edges and insert Steiner points (midpoints) until the constraint is represented in the triangulation.

```csharp
using CDT;

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
cdt.ConformToEdges(edges); // may add Steiner points
cdt.EraseOuterTriangles();
```

### Removing duplicate vertices and remapping edges

Input data often contains duplicate vertices (e.g., from shared polygon boundaries). Use `CdtUtils.RemoveDuplicatesAndRemapEdges` to clean up before triangulation.

```csharp
using CDT;

var vertices = new List<V2d<double>>
{
    new(0, 0), new(4, 0), new(4, 4), new(0, 4),
    new(0, 0), // duplicate of vertex 0
};
var edges = new List<Edge>
{
    new(0, 4), // will be remapped since vertex 4 is a duplicate of vertex 0
    new(1, 2),
};

CdtUtils.RemoveDuplicatesAndRemapEdges(vertices, edges);
// vertices now has 4 entries; degenerate self-edges like (0,0) can be dropped

var cdt = new Triangulation<double>();
cdt.InsertVertices(vertices);
cdt.InsertEdges(edges.Where(e => e.V1 != e.V2).ToList());
cdt.EraseSuperTriangle();
```

### Resolving intersecting constraint edges

By default, intersecting constraint edges throw an exception. Pass `IntersectingConstraintEdges.TryResolve` to split them at their intersection point instead.

```csharp
using CDT;

// Two diagonals of a unit square that cross each other
var vertices = new List<V2d<double>>
{
    new(0, 0), new(1, 0), new(1, 1), new(0, 1),
};
var edges = new List<Edge>
{
    new(0, 2), // diagonal ↗
    new(1, 3), // diagonal ↖ — intersects (0,2)
};

var cdt = new Triangulation<double>(
    VertexInsertionOrder.Auto,
    IntersectingConstraintEdges.TryResolve,
    minDistToConstraintEdge: 0.0);

cdt.InsertVertices(vertices);
cdt.InsertEdges(edges); // intersection is resolved by inserting a new vertex
cdt.EraseSuperTriangle();
```

## Building

```bash
dotnet build
```

## Testing

```bash
dotnet run --project test/CDT.Tests
```

## Benchmarking

```bash
dotnet run -c Release --project benchmark/CDT.Benchmarks
```

## Comparison Benchmarks

CDT.NET is benchmarked against other C# and native CDT/Delaunay triangulation libraries on the **"Constrained Sweden"** dataset (~2 600 vertices, ~2 600 constraint edges).

**Libraries compared:** CDT.NET, Triangle.NET, NetTopologySuite (NTS), artem-ogre/CDT (C++), CGAL (C++), Spade (Rust).

> 12th Gen Intel Core i7-12700KF 3.60GHz, 1 CPU, 20 logical and 12 physical cores

| Method                 | Categories   | Mean      | Error      | StdDev    | Ratio |
|----------------------- |------------- |----------:|-----------:|----------:|------:|
| CDT.NET                | Conforming   |  1.442 ms |  0.1628 ms | 0.0089 ms |  1.00 |
| 'artem-ogre/CDT (C++)' | Conforming   |  1.976 ms |  0.0501 ms | 0.0027 ms |  1.37 |
| 'Spade (Rust)'         | Conforming   |  1.341 ms |  0.2933 ms | 0.0161 ms |  0.93 |
| 'CGAL (C++)'           | Conforming   |  4.110 ms |  0.3934 ms | 0.0216 ms |  2.85 |
| NTS                    | Conforming   | 38.288 ms | 39.8335 ms | 2.1834 ms | 26.55 |
| Triangle.NET           | Conforming   |  3.284 ms |  0.6901 ms | 0.0378 ms |  2.28 |
|                        |              |           |            |           |       |
| CDT.NET                | Constrained  |  1.167 ms |  0.0737 ms | 0.0040 ms |  1.00 |
| 'artem-ogre/CDT (C++)' | Constrained  |  1.766 ms |  0.0619 ms | 0.0034 ms |  1.51 |
| 'Spade (Rust)'         | Constrained  |  1.256 ms |  0.1233 ms | 0.0068 ms |  1.08 |
| 'CGAL (C++)'           | Constrained  |  2.613 ms |  0.3773 ms | 0.0207 ms |  2.24 |
| Triangle.NET           | Constrained  |  3.290 ms |  1.3341 ms | 0.0731 ms |  2.82 |
|                        |              |           |            |           |       |
| CDT.NET                | VerticesOnly |  1.072 ms |  0.0045 ms | 0.0002 ms |  1.00 |
| 'artem-ogre/CDT (C++)' | VerticesOnly |  1.568 ms |  0.2550 ms | 0.0140 ms |  1.46 |
| 'Spade (Rust)'         | VerticesOnly |  1.038 ms |  0.0224 ms | 0.0012 ms |  0.97 |
| 'CGAL (C++)'           | VerticesOnly |  2.156 ms |  0.2064 ms | 0.0113 ms |  2.01 |
| NTS                    | VerticesOnly |  5.608 ms |  2.5000 ms | 0.1370 ms |  5.23 |
| Triangle.NET           | VerticesOnly |  1.355 ms |  0.0418 ms | 0.0023 ms |  1.26 |

**Key takeaways:**
- **CDT.NET matches the original C++ implementation (artem-ogre/CDT) and Spade within ≤13%**.
- **CGAL** runs at ~2× CDT.NET. CGAL's `Constrained_Delaunay_triangulation_2` uses a more complex data structure (half-edge DCEL) with additional bookkeeping overhead vs. CDT.NET's compact flat arrays. For raw triangulation throughput CDT.NET is faster.
- **CDT.NET allocates 5–120× less managed memory** than Triangle.NET and NTS: Triangle.NET allocates ~5.7× more, NTS ~121× more.
- **NTS (conforming CDT)** is ~30× slower and allocates ~120× more memory — Steiner-point insertion is the main cost, and the result is semantically different (not true CDT).
- Native wrappers (artem-ogre/CDT, CGAL, Spade) show zero managed allocations as expected for P/Invoke calls into unmanaged code.

For full details, prerequisites, and instructions on running the comparison benchmarks, see the [CDT.Comparison.Benchmarks README](benchmark/CDT.Comparison.Benchmarks/README.md).

```bash
dotnet run -c Release --project benchmark/CDT.Comparison.Benchmarks
```

## License

[Mozilla Public License Version 2.0](LICENSE)

This software is based in part on [CDT — C++ library for constrained Delaunay triangulation](https://github.com/artem-ogre/CDT):
Copyright © 2019 Leica Geosystems Technology AB  
Copyright © The CDT Contributors  
Licensed under the MPL-2.0 license.
