// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

#include "CDT.h"
#include <vector>
#include <cstdint>

extern "C" {

/// Triangulate points with optional constraint edges using artem-ogre/CDT.
/// Returns the number of triangles (including super-triangle triangles),
/// or -1 on error.
int32_t cdt_triangulate_d(
    const double* xs,
    const double* ys,
    int32_t       n_verts,
    const int32_t* edge_v1,
    const int32_t* edge_v2,
    int32_t        n_edges)
{
    try
    {
        CDT::Triangulation<double> cdt(
            CDT::VertexInsertionOrder::Auto,
            CDT::IntersectingConstraintEdges::TryResolve,
            0.0);

        std::vector<CDT::V2d<double>> verts;
        verts.reserve(static_cast<size_t>(n_verts));
        for (int32_t i = 0; i < n_verts; ++i)
            verts.push_back(CDT::V2d<double>(xs[i], ys[i]));
        cdt.insertVertices(verts);

        if (n_edges > 0)
        {
            std::vector<CDT::Edge> edges;
            edges.reserve(static_cast<size_t>(n_edges));
            for (int32_t i = 0; i < n_edges; ++i)
                edges.emplace_back(
                    static_cast<CDT::VertInd>(edge_v1[i]),
                    static_cast<CDT::VertInd>(edge_v2[i]));
            cdt.insertEdges(edges);
        }

        return static_cast<int32_t>(cdt.triangles.size());
    }
    catch (...)
    {
        return -1;
    }
}

/// Triangulate points with constraint edges using artem-ogre/CDT conforming mode.
/// Uses conformToEdges() which inserts Steiner points (midpoints) along each
/// constraint edge until it is represented by a chain of triangulation edges,
/// producing a Conforming Delaunay Triangulation.
/// Returns the number of triangles, or -1 on error.
int32_t cdt_conform_d(
    const double*  xs,
    const double*  ys,
    int32_t        n_verts,
    const int32_t* edge_v1,
    const int32_t* edge_v2,
    int32_t        n_edges)
{
    try
    {
        CDT::Triangulation<double> cdt(
            CDT::VertexInsertionOrder::Auto,
            CDT::IntersectingConstraintEdges::TryResolve,
            0.0);

        std::vector<CDT::V2d<double>> verts;
        verts.reserve(static_cast<size_t>(n_verts));
        for (int32_t i = 0; i < n_verts; ++i)
            verts.push_back(CDT::V2d<double>(xs[i], ys[i]));
        cdt.insertVertices(verts);

        if (n_edges > 0)
        {
            std::vector<CDT::Edge> edges;
            edges.reserve(static_cast<size_t>(n_edges));
            for (int32_t i = 0; i < n_edges; ++i)
                edges.emplace_back(
                    static_cast<CDT::VertInd>(edge_v1[i]),
                    static_cast<CDT::VertInd>(edge_v2[i]));
            cdt.conformToEdges(edges);
        }

        return static_cast<int32_t>(cdt.triangles.size());
    }
    catch (...)
    {
        return -1;
    }
}

} // extern "C"
