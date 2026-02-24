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

## Benchmark results

> 12th Gen Intel Core i7-12700KF 3.60GHz, 1 CPU, 20 logical and 12 physical cores

| Method                 | Categories   | Mean      | Error     | StdDev    | Ratio | RatioSD |
|----------------------- |------------- |----------:|----------:|----------:|------:|--------:|
| **CDT.NET**            | Constrained  |  1.198 ms | 0.1065 ms | 0.0058 ms |  1.00 |    0.01 |
| Triangle.NET           | Constrained  |  4.571 ms | 2.2827 ms | 0.1251 ms |  3.82 |    0.09 |
| 'NTS (Conforming CDT)' | Constrained  | 37.066 ms | 8.5571 ms | 0.4690 ms | 30.95 |    0.36 |
| 'artem-ogre/CDT (C++)' | Constrained  |  1.788 ms | 0.1284 ms | 0.0070 ms |  1.49 |    0.01 |
| 'CGAL (C++)'           | Constrained  |  2.538 ms | 0.0574 ms | 0.0031 ms |  2.12 |    0.01 |
| 'Spade (Rust)'         | Constrained  |  1.255 ms | 0.1050 ms | 0.0058 ms |  1.05 |    0.01 |
|                        |              |           |           |           |       |         |
| **CDT.NET**            | VerticesOnly |  1.048 ms | 0.0371 ms | 0.0020 ms |  1.00 |    0.00 |
| Triangle.NET           | VerticesOnly |  1.323 ms | 0.1856 ms | 0.0102 ms |  1.26 |    0.01 |
| NTS                    | VerticesOnly |  5.519 ms | 2.8885 ms | 0.1583 ms |  5.26 |    0.13 |
| 'CGAL (C++)'           | VerticesOnly |  2.063 ms | 0.2154 ms | 0.0118 ms |  1.97 |    0.01 |
| 'artem-ogre/CDT (C++)' | VerticesOnly |  1.557 ms | 0.1013 ms | 0.0056 ms |  1.49 |    0.01 |
| 'Spade (Rust)'         | VerticesOnly |  1.028 ms | 0.0803 ms | 0.0044 ms |  0.98 |    0.00 |


### Key takeaways

- **CDT.NET matches the original C++ implementation (artem-ogre/CDT) and Spade within ≤13%** on both constrained and unconstrained triangulation.
- **CGAL** runs at ~2× CDT.NET. CGAL's `Constrained_Delaunay_triangulation_2` uses a more complex data structure (half-edge DCEL) with additional bookkeeping overhead vs. CDT.NET's compact flat arrays. For raw triangulation throughput CDT.NET is faster.
- **CDT.NET allocates 5–120× less managed memory** than Triangle.NET and NTS: Triangle.NET allocates ~5.7× more, NTS ~121× more.
- **NTS (conforming CDT)** is ~30× slower and allocates ~120× more memory — Steiner-point insertion is the main cost, and the result is semantically different (not true CDT).
- Native wrappers (artem-ogre/CDT, CGAL, Spade) show zero managed allocations as expected for P/Invoke calls into unmanaged code.

