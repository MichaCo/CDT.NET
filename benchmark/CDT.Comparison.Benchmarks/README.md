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

The C# libraries (CDT.NET, Triangle.NET, NetTopologySuite, Poly2Tri) are fetched automatically by NuGet.  
The two **native** wrappers (artem-ogre/CDT and Spade) are **compiled from source** as part of `dotnet build` and require additional tooling.

### Linux / macOS

| Tool | Purpose | Install |
|------|---------|---------|
| `cmake` ≥ 3.20 | Builds the C++ wrapper | Package manager or https://cmake.org/download/ |
| C++17 compiler | `gcc`/`clang` | `sudo apt install build-essential` / `xcode-select --install` |
| `git` | CMake FetchContent (downloads artem-ogre/CDT) | Usually pre-installed |
| `cargo` (Rust stable) | Builds the Rust wrapper | https://rustup.rs/ |

```bash
# Ubuntu / Debian
sudo apt install cmake build-essential git
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

### Windows

| Tool | Purpose | Install |
|------|---------|---------|
| **Visual Studio 2022** (or Build Tools) with the **"Desktop development with C++"** workload | C++17 compiler (MSVC) | https://visualstudio.microsoft.com/downloads/ |
| **CMake** ≥ 3.20 | Builds the C++ wrapper | Bundled with VS 2022, or https://cmake.org/download/ (tick "Add to PATH") |
| **Git for Windows** | CMake FetchContent | https://git-scm.com/download/win |
| **Rust** (stable, MSVC ABI) | Builds the Rust wrapper | https://rustup.rs/ → choose `x86_64-pc-windows-msvc` |

> **Tip:** Install Visual Studio first, then Rust. The Rust installer auto-detects the MSVC linker.  
> Open a **Developer Command Prompt for VS 2022** (or a normal terminal after running `vcvarsall.bat`) so that `cmake`, `cl`, and `cargo` are all on your PATH.

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
| artem-ogre/CDT (C++) | Triangle count includes the three super-triangle triangles (same behaviour as CDT.NET before `EraseSuperTriangle`) |
| Spade (Rust) | `num_inner_faces()` returns only finite (inner) triangles, which is fewer than the C++ CDT count |
| Poly2Tri (Constrained) | Throws an internal exception on the Sweden CDT dataset; constrained benchmark shows `NA` |

## Benchmark results

Measured on **AMD EPYC 7763 2.45 GHz, 1 CPU, 4 logical / 2 physical cores, .NET 10.0.2, Linux Ubuntu 24.04** using `[ShortRunJob]` (3 warmup + 3 iterations).  Dataset: **Constrained Sweden** (~2 600 vertices, ~2 600 constraint edges).

### Constrained CDT

| Library | Mean | Ratio | Allocated | Alloc ratio |
|---------|-----:|------:|----------:|------------:|
| **CDT.NET** *(baseline)* | 1,740 μs | 1.00 | 494 KB | 1.00 |
| artem-ogre/CDT (C++) | 1,952 μs | 1.12 | — | 0.00 |
| Spade (Rust) | 1,937 μs | 1.11 | — | 0.00 |
| Triangle.NET | 6,115 μs | 3.52 | 2,817 KB | 5.70 |
| NTS (Conforming CDT) | 51,643 μs | 29.69 | 59,938 KB | 121.19 |
| Poly2Tri | ❌ error | — | — | — |

### Vertices-only (Delaunay, no constraints)

| Library | Mean | Ratio | Allocated | Alloc ratio |
|---------|-----:|------:|----------:|------------:|
| **CDT.NET** *(baseline)* | 1,604 μs | 1.00 | 322 KB | 1.00 |
| artem-ogre/CDT (C++) | 1,578 μs | 0.98 | — | 0.00 |
| Poly2Tri | 359 μs | 0.22 | 540 KB | 1.68 |
| Triangle.NET | 2,064 μs | 1.29 | 1,760 KB | 5.47 |
| Spade (Rust) | 1,606 μs | 1.00 | — | 0.00 |
| NTS | 7,876 μs | 4.91 | 4,372 KB | 13.58 |

**Key takeaways:**
- CDT.NET matches the original C++ library and Spade within ≤12% on constrained triangulation.
- CDT.NET allocates ~5–120× less memory than Triangle.NET and NTS.
- Poly2Tri is fast on unconstrained Delaunay but fails on the Sweden CDT dataset.
- NTS (conforming CDT) is ~30× slower and allocates ~120× more memory due to Steiner-point insertion.

