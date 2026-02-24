// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using BenchmarkDotNet.Running;

var bench = new ComparisonBenchmarks();
bench.Setup();

static void Print(string label, Func<int> compute) =>
Console.WriteLine($"  {label,-22}  {compute(),6:N0} triangles");

const int lineWidth = 38;
Console.WriteLine("Constrained Delaunay Triangulation");
Console.WriteLine(new string('-', lineWidth));
Print("CDT.NET",              bench.CDT_CdtNet);
Print("artem-ogre/CDT (C++)", bench.CDT_NativeCdt);
Print("Spade (Rust)",         bench.CDT_Spade);
Print("CGAL (C++)",           bench.CDT_Cgal);
Print("Triangle.NET",         bench.CDT_TriangleNet);
Console.WriteLine(new string('-', lineWidth));

Console.WriteLine("Conforming Delaunay Triangulation");
Console.WriteLine(new string('-', lineWidth));
Print("CDT.NET",              bench.CfDT_CdtNet);
Print("artem-ogre/CDT (C++)", bench.CfDT_NativeCdt);
Print("Spade (Rust)",         bench.CfDT_Spade);
Print("CGAL (C++)",           bench.CfDT_Cgal);
Print("NTS",                  bench.CfDT_Nts);
Print("Triangle.NET",         bench.CfDT_TriangleNet);
Console.WriteLine(new string('-', lineWidth));

BenchmarkSwitcher.FromAssembly(typeof(ComparisonBenchmarks).Assembly).Run(args);
