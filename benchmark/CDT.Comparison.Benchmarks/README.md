# CDT.Comparison.Benchmarks

Compares the performance of **CDT.NET** against other C# and native CDT/Delaunay triangulation libraries on the same input datasets used by the CDT.NET test suite.

## Libraries compared

| # | Library | NuGet / Source | CDT type | Notes |
|---|---------|---------------|----------|-------|
| 1 | **CDT.NET** (baseline) | Project ref | True CDT | This repo |
| 2 | **Triangle.NET** | `Unofficial.Triangle.NET` 0.0.1 | True CDT | Classic robust C# triangulator |
| 3 | **NetTopologySuite** | `NetTopologySuite` 2.6.0 | Conforming CDT | May insert Steiner points |
| 4 | **Poly2Tri** | `Poly2Tri.NetStandard` 1.0.2 | CDT | Sweep-line algorithm |
| 5 | **artem-ogre/CDT** | C++ via P/Invoke | True CDT | Original C++ library CDT.NET is ported from |
| 6 | **Spade** | Rust via P/Invoke | CDT | Rust `spade` crate 2.15.0 |

## Benchmark categories

| Category | Description |
|----------|-------------|
| `VerticesOnly` | Delaunay triangulation of all vertices, no constraint edges |
| `Constrained` | Full CDT — vertices + constraint edges |

The dataset used is **"Constrained Sweden"** (~2 600 vertices, ~2 600 constraint edges), the same dataset used by the upstream C++ CDT benchmark suite.

## Prerequisites

### Native libraries (artem-ogre/CDT and Spade)

The two native wrappers are built automatically when you run `dotnet build`. You need:

- **C++ wrapper** — `cmake` ≥ 3.20, a C++17 compiler (gcc/clang/MSVC), and internet access (CMake FetchContent downloads artem-ogre/CDT from GitHub)
- **Rust wrapper** — `cargo` (Rust stable toolchain), internet access to crates.io

On Linux/macOS the wrappers produce `libcdt_wrapper.so` / `libspade_wrapper.so`; on Windows they would need `cdt_wrapper.dll` / `spade_wrapper.dll` (not yet configured).

## Running the benchmarks

```bash
# From the repository root — Release configuration is required for BenchmarkDotNet
dotnet run --project benchmark/CDT.Comparison.Benchmarks -c Release
```

BenchmarkDotNet will present an interactive menu to select which benchmark class / category to run. To run all benchmarks non-interactively:

```bash
dotnet run --project benchmark/CDT.Comparison.Benchmarks -c Release -- --filter "*"
```

Results are written to `BenchmarkDotNet.Artifacts/` in the current directory.

## Known limitations

| Library | Limitation |
|---------|-----------|
| NetTopologySuite | Uses *conforming* CDT — Steiner points may be inserted, so the triangle count and layout differ from true CDT results |
| artem-ogre/CDT (C++) | Triangle count includes the three super-triangle triangles (same behaviour as CDT.NET before `EraseSuperTriangle`) |
| Spade (Rust) | `num_inner_faces()` returns only finite (inner) triangles, which is fewer than the C++ CDT count |
| Native wrappers | Linux only in the current configuration; Windows support requires separate `.dll` build scripts |
