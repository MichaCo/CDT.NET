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
The three **native** wrappers (artem-ogre/CDT, CGAL, Spade) are **compiled from source** as part of `dotnet build` and require additional tooling.

### Linux / macOS

| Tool | Purpose | Install |
|------|---------|---------|
| `cmake` ≥ 3.20 | Builds the C++ wrappers | Package manager or https://cmake.org/download/ |
| C++17 compiler | `gcc`/`clang` | `sudo apt install build-essential` / `xcode-select --install` |
| **CGAL** | Headers + cmake config for the CGAL wrapper | `sudo apt install libcgal-dev` / `brew install cgal` |
| `git` | CMake FetchContent (downloads artem-ogre/CDT) | Usually pre-installed |
| `cargo` (Rust stable) | Builds the Rust wrapper | https://rustup.rs/ |

```bash
# Ubuntu / Debian
sudo apt install cmake build-essential git libcgal-dev
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

```bash
# macOS
brew install cmake cgal
curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh
```

### Windows

| Tool | Purpose | Install |
|------|---------|---------|
| **Visual Studio 2022** (or Build Tools) with the **"Desktop development with C++"** workload | C++17 compiler (MSVC) | https://visualstudio.microsoft.com/downloads/ |
| **CMake** ≥ 3.20 | Builds the C++ wrappers | Bundled with VS 2022, or https://cmake.org/download/ (tick "Add to PATH") |
| **CGAL** | Headers + cmake config for the CGAL wrapper | `winget install CGAL.CGAL` or `vcpkg install cgal` |
| **Git for Windows** | CMake FetchContent | https://git-scm.com/download/win |
| **Rust** (stable, MSVC ABI) | Builds the Rust wrapper | https://rustup.rs/ → choose `x86_64-pc-windows-msvc` |

> **Tip:** Install Visual Studio first, then Rust. The Rust installer auto-detects the MSVC linker.  
> Open a **Developer Command Prompt for VS 2022** (or a normal terminal after running `vcvarsall.bat`) so that `cmake`, `cl`, and `cargo` are all on your PATH.
>
> For CGAL on Windows the easiest path is **vcpkg**:
> ```powershell
> git clone https://github.com/microsoft/vcpkg
> .\vcpkg\bootstrap-vcpkg.bat
> .\vcpkg\vcpkg install cgal:x64-windows
> # Then pass -DCMAKE_TOOLCHAIN_FILE=<path_to_vcpkg>/scripts/buildsystems/vcpkg.cmake to cmake,
> # or set VCPKG_ROOT environment variable (picked up automatically by CMake 3.25+).
> ```

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
| CGAL (C++) | `number_of_faces()` counts all finite triangles in the triangulation, consistent with artem-ogre/CDT |
| Spade (Rust) | `num_inner_faces()` returns only inner (non-convex-hull) triangles, which is fewer than the C++ CDT counts |

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
| CGAL (C++) | *not yet measured* | — | — | — |

### Vertices-only (Delaunay, no constraints)

| Library | Mean | Ratio | Allocated | Alloc ratio |
|---------|-----:|------:|----------:|------------:|
| **CDT.NET** *(baseline)* | 1,604 μs | 1.00 | 322 KB | 1.00 |
| artem-ogre/CDT (C++) | 1,578 μs | 0.98 | — | 0.00 |
| Triangle.NET | 2,064 μs | 1.29 | 1,760 KB | 5.47 |
| Spade (Rust) | 1,606 μs | 1.00 | — | 0.00 |
| NTS | 7,876 μs | 4.91 | 4,372 KB | 13.58 |
| CGAL (C++) | *not yet measured* | — | — | — |

**Key takeaways (excluding CGAL which is newly added):**
- CDT.NET matches the original C++ library and Spade within ≤12% on constrained triangulation.
- CDT.NET allocates ~5–120× less memory than Triangle.NET and NTS.
- NTS (conforming CDT) is ~30× slower and allocates ~120× more memory due to Steiner-point insertion.

