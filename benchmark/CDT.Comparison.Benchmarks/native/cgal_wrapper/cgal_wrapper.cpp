// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

// CGAL 2D Constrained Delaunay Triangulation wrapper.
// Uses the Exact_predicates_inexact_constructions_kernel (Epick):
//   - exact geometric predicates via interval arithmetic (no GMP/MPFR needed)
//   - inexact (double) constructions

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Constrained_Delaunay_triangulation_2.h>
#include <CGAL/Constrained_triangulation_plus_2.h>
#include <CGAL/Triangulation_conformer_2.h>
#include <vector>
#include <cstdint>

typedef CGAL::Exact_predicates_inexact_constructions_kernel   K;
typedef CGAL::Constrained_Delaunay_triangulation_2<K>         CDT;
typedef CDT::Point                                            Point;
typedef CDT::Vertex_handle                                    Vertex_handle;

// For conforming CDT: wrap CDT with Exact_predicates_tag so the conformer
// can split constraint edges, then apply make_conforming_Delaunay_2.
typedef CGAL::Constrained_Delaunay_triangulation_2<
    K, CGAL::Default, CGAL::Exact_predicates_tag>             CDT_Conform_Base;
typedef CGAL::Constrained_triangulation_plus_2<CDT_Conform_Base> CDTP;

extern "C" {

/// Triangulate points with optional constraint edges using CGAL.
/// Returns the total number of faces (triangles) in the triangulation,
/// or -1 on error.
int32_t cgal_cdt(
    const double*  xs,
    const double*  ys,
    int32_t        n_verts,
    const int32_t* edge_v1,
    const int32_t* edge_v2,
    int32_t        n_edges)
{
    try
    {
        CDT cdt;

        std::vector<Vertex_handle> handles;
        handles.reserve(static_cast<size_t>(n_verts));
        for (int32_t i = 0; i < n_verts; ++i)
            handles.push_back(cdt.insert(Point(xs[i], ys[i])));

        for (int32_t i = 0; i < n_edges; ++i)
            cdt.insert_constraint(handles[edge_v1[i]], handles[edge_v2[i]]);

        return static_cast<int32_t>(cdt.number_of_faces());
    }
    catch (...)
    {
        return -1;
    }
}

/// Triangulate points with constraint edges using CGAL Conforming Delaunay.
/// Uses Constrained_triangulation_plus_2 + make_conforming_Delaunay_2, which
/// inserts Steiner points until every constrained edge is a Delaunay edge.
/// Returns the total number of faces (triangles), or -1 on error.
int32_t cgal_conform(
    const double*  xs,
    const double*  ys,
    int32_t        n_verts,
    const int32_t* edge_v1,
    const int32_t* edge_v2,
    int32_t        n_edges)
{
    try
    {
        CDTP cdt;

        std::vector<CDTP::Vertex_handle> handles;
        handles.reserve(static_cast<size_t>(n_verts));
        for (int32_t i = 0; i < n_verts; ++i)
            handles.push_back(cdt.insert(Point(xs[i], ys[i])));

        for (int32_t i = 0; i < n_edges; ++i)
            cdt.insert_constraint(handles[edge_v1[i]], handles[edge_v2[i]]);

        CGAL::make_conforming_Delaunay_2(cdt);

        return static_cast<int32_t>(cdt.number_of_faces());
    }
    catch (...)
    {
        return -1;
    }
}

} // extern "C"
