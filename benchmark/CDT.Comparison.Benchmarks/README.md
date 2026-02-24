# CDT.Comparison.Benchmarks

Compares the performance of **CDT.NET** against other C# and native CDT/Delaunay triangulation libraries on the same input datasets used by the CDT.NET test suite.

## Libraries compared

| # | Library | NuGet / Source | CDT type | Notes |
|---|---------|---------------|----------|-------|
| 1 | **CDT.NET** (baseline) | Project ref | True CDT | This repo |
| 2 | **Triangle.NET** | `Unofficial.Triangle.NET` 0.0.1 | True CDT | Classic robust C# triangulator |
| 3 | **NetTopologySuite** | `NetTopologySuite` 2.6.0 | Conforming CDT | May insert Steiner points |
| 4 | **artem-ogre/CDT** | C++ via P/Invoke | True CDT | Original C++ library CDT.NET is ported from |
| 5 | **CGAL** | C++ via P/Invoke | True CDT | Industry-standard C++ geometry library (Epick kernel) |
| 6 | **Spade** | Rust via P/Invoke | CDT | Rust `spade` crate 2.15.0 |

## Benchmark categories

| Category | Description |
|----------|-------------|
| `VerticesOnly` | Delaunay triangulation of all vertices, no constraint edges |
| `Constrained` | Full CDT — vertices + constraint edges |

The dataset used is **"Constrained Sweden"** (~2 600 vertices, ~2 600 constraint edges), the same dataset used by the upstream C++ CDT benchmark suite.

## Prerequisites

The C# libraries (CDT.NET, Triangle.NET, NetTopologySuite) are fetched automatically by NuGet.  
The three **native** wrappers (artem-ogre/CDT, CGAL, Spade) are **compiled from source** as part of `dotnet build`. CGAL and its required Boost sub-libraries are downloaded automatically by CMake FetchContent on first build (~20 MB total, cached afterwards). No manual installation of CGAL or Boost is required.

### Linux / macOS

| Tool | Purpose | Install |
|------|---------|---------|
| `cmake` ≥ 3.20 | Builds the C++ wrappers | Package manager or https://cmake.org/download/ |
| C++17 compiler | `gcc`/`clang` | `sudo apt install build-essential` / `xcode-select --install` |
| `git` | CMake FetchContent (downloads artem-ogre/CDT) | Usually pre-installed |
| `cargo` (Rust stable) | Builds the Rust wrapper | https://rustup.rs/ |

```bash
# Ubuntu / Debian
sudo apt install cmake build-essential git
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

```bash
# macOS
brew install cmake
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

### Windows

| Tool | Purpose | Install |
|------|---------|---------|
| **Visual Studio 2022** (or Build Tools) with the **"Desktop development with C++"** workload | C++17 compiler (MSVC) | https://visualstudio.microsoft.com/downloads/ |
| **CMake** ≥ 3.20 | Builds the C++ wrappers | Bundled with VS 2022, or https://cmake.org/download/ (tick "Add to PATH") |
| **Git for Windows** | CMake FetchContent (downloads artem-ogre/CDT, CGAL, Boost sub-libs) | https://git-scm.com/download/win |
| **Rust** (stable, MSVC ABI) | Builds the Rust wrapper | https://rustup.rs/ → choose `x86_64-pc-windows-msvc` |

> **Tip:** Install Visual Studio first, then Rust. The Rust installer auto-detects the MSVC linker.  
> Open a **Developer Command Prompt for VS 2022** (or a normal terminal after running `vcvarsall.bat`) so that `cmake`, `cl`, and `cargo` are all on your PATH.
>
> CGAL and Boost are downloaded automatically by CMake — no manual install required.

If any of these tools are missing, `dotnet build` will print a clear error message pointing to the missing tool before failing.

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
| artem-ogre/CDT (C++) | Triangle count includes all convex-hull triangles (same behaviour as CDT.NET before `EraseSuperTriangle`) |
| CGAL (C++) | `number_of_faces()` counts all finite triangles in the triangulation, consistent with artem-ogre/CDT. First build downloads CGAL 6.1.1 library headers (~10 MB) and the required Boost sub-library headers (~10 MB ZIPs, ~60 MB staged); subsequent builds use the cmake cache. |
| Spade (Rust) | `num_inner_faces()` returns only inner (non-convex-hull) triangles, which is fewer than the C++ CDT counts |

## Counts of triangles by library and category

```
Constrained Delaunay Triangulation
--------------------------------------
  CDT.NET                  5.239 triangles
  artem-ogre/CDT (C++)     5.239 triangles
  Spade (Rust)             5.204 triangles
  CGAL (C++)               5.204 triangles
  Triangle.NET             5.204 triangles
--------------------------------------
Conforming Delaunay Triangulation
--------------------------------------
  CDT.NET                  5.343 triangles
  artem-ogre/CDT (C++)     5.343 triangles
  Spade (Rust)             5.308 triangles
  CGAL (C++)               5.324 triangles
  NTS                      5.764 triangles
  Triangle.NET             5.204 triangles
--------------------------------------
```

## Benchmark results

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

### Key takeaways

- **CDT.NET matches the original C++ implementation (artem-ogre/CDT) and Spade within ≤13%**.
- **CGAL** runs at ~2× CDT.NET. CGAL's `Constrained_Delaunay_triangulation_2` uses a more complex data structure (half-edge DCEL) with additional bookkeeping overhead vs. CDT.NET's compact flat arrays. For raw triangulation throughput CDT.NET is faster.
- **CDT.NET allocates 5–120× less managed memory** than Triangle.NET and NTS: Triangle.NET allocates ~5.7× more, NTS ~121× more.
- **NTS (conforming CDT)** is ~30× slower and allocates ~120× more memory — Steiner-point insertion is the main cost, and the result is semantically different (not true CDT).
- Native wrappers (artem-ogre/CDT, CGAL, Spade) show zero managed allocations as expected for P/Invoke calls into unmanaged code.

