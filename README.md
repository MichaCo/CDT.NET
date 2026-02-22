# CDT.NET

[![Build](https://github.com/MichaCo/CDT.NET/actions/workflows/ci-cd.yml/badge.svg?branch=main)](https://github.com/MichaCo/CDT.NET/actions/workflows/ci-cd.yml)
[![NuGet](https://img.shields.io/nuget/v/CDT.NET.svg)](https://www.nuget.org/packages/CDT.NET)

A C# port of the [artem-ogre/CDT](https://github.com/artem-ogre/CDT) Constrained Delaunay Triangulation library.

Fast and robust Constrained Delaunay Triangulation for .NET 8+.

## Features

- Constrained Delaunay Triangulation (CDT)
- Support for constrained edges and holes
- Multiple vertex insertion strategies (auto-optimized or as-provided)
- KD-tree spatial indexing for fast nearest-neighbor lookups
- Robust geometric predicates
- Multi-target: .NET 8 and .NET 10

## Installation

```
dotnet add package CDT.NET
```

## Usage

```csharp
using CDT;

// Create a triangulation
var cdt = new Triangulation<double>();

// Insert vertices
var vertices = new List<V2d<double>>
{
    new(0, 0),
    new(1, 0),
    new(1, 1),
    new(0, 1),
};
cdt.InsertVertices(vertices);

// Optionally insert constrained edges
var edges = new List<Edge>
{
    new(0, 1),
    new(1, 2),
    new(2, 3),
    new(3, 0),
};
cdt.InsertEdges(edges);

// Erase triangles outside the domain
cdt.EraseOuterTriangles();

// Access the result
IReadOnlyList<Triangle> triangles = cdt.Triangles;
IReadOnlyList<V2d<double>> points = cdt.Vertices;
```

## Building

```bash
dotnet build
```

## Testing

```bash
dotnet test
```

## Benchmarking

```bash
dotnet run -c Release --project benchmark/CDT.Benchmarks
```

## License

[MIT License](LICENSE)
