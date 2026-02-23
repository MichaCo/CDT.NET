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

Measured on **AMD EPYC 7763 2.45 GHz, 1 CPU, 4 logical / 2 physical cores, .NET 10.0.2, Linux Ubuntu 24.04** using `[ShortRunJob]` (3 warmup + 3 iterations).  Dataset: **Constrained Sweden** (~2 600 vertices, ~2 600 constraint edges).

### Constrained CDT

| Library | Mean | Error | StdDev | Ratio | Allocated | Alloc Ratio |
|---------|-----:|------:|-------:|------:|----------:|------------:|
| **CDT.NET** *(baseline)* | 1,759 μs | 143.0 μs | 7.8 μs | 1.00 | 495 KB | 1.00 |
| artem-ogre/CDT (C++) | 1,961 μs | 85.4 μs | 4.7 μs | 1.11 | — | 0.00 |
| Spade (Rust) | 1,989 μs | 120.0 μs | 6.6 μs | 1.13 | — | 0.00 |
| CGAL (C++) | 3,659 μs | 288.2 μs | 15.8 μs | 2.08 | — | 0.00 |
| Triangle.NET | 6,405 μs | 2,193.7 μs | 120.2 μs | 3.64 | 2,817 KB | 5.70 |
| NTS (Conforming CDT) | 54,282 μs | 10,703.7 μs | 586.7 μs | 30.86 | 59,937 KB | 121.19 |

### Vertices-only (Delaunay, no constraints)

| Library | Mean | Error | StdDev | Ratio | Allocated | Alloc Ratio |
|---------|-----:|------:|-------:|------:|----------:|------------:|
| **CDT.NET** *(baseline)* | 1,584 μs | 135.3 μs | 7.4 μs | 1.00 | 322 KB | 1.00 |
| artem-ogre/CDT (C++) | 1,586 μs | 54.8 μs | 3.0 μs | 1.00 | — | 0.00 |
| Spade (Rust) | 1,613 μs | 27.3 μs | 1.5 μs | 1.02 | — | 0.00 |
| CGAL (C++) | 3,254 μs | 104.8 μs | 5.7 μs | 2.05 | — | 0.00 |
| Triangle.NET | 2,134 μs | 298.8 μs | 16.4 μs | 1.35 | 1,760 KB | 5.47 |
| NTS | 7,960 μs | 1,199.3 μs | 65.7 μs | 5.03 | 4,373 KB | 13.58 |

### Key takeaways

- **CDT.NET matches the original C++ implementation (artem-ogre/CDT) and Spade within ≤13%** on both constrained and unconstrained triangulation.
- **CGAL** runs at ~2× CDT.NET. CGAL's `Constrained_Delaunay_triangulation_2` uses a more complex data structure (half-edge DCEL) with additional bookkeeping overhead vs. CDT.NET's compact flat arrays. For raw triangulation throughput CDT.NET is faster.
- **CDT.NET allocates 5–120× less managed memory** than Triangle.NET and NTS: Triangle.NET allocates ~5.7× more, NTS ~121× more.
- **NTS (conforming CDT)** is ~30× slower and allocates ~120× more memory — Steiner-point insertion is the main cost, and the result is semantically different (not true CDT).
- Native wrappers (artem-ogre/CDT, CGAL, Spade) show zero managed allocations as expected for P/Invoke calls into unmanaged code.

