// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use std::panic;
use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};
use spade::handles::FixedVertexHandle;

type Cdt = ConstrainedDelaunayTriangulation<Point2<f64>>;

/// Returns true if h1 and h2 are already directly connected by a triangulation edge.
fn is_direct_edge(cdt: &Cdt, h1: FixedVertexHandle, h2: FixedVertexHandle) -> bool {
    cdt.vertex(h1).out_edges().any(|e| e.to().fix() == h2)
}

/// Returns the midpoint coordinates of the segment between h1 and h2.
fn midpoint(cdt: &Cdt, h1: FixedVertexHandle, h2: FixedVertexHandle) -> (f64, f64) {
    let p1 = cdt.vertex(h1).position();
    let p2 = cdt.vertex(h2).position();
    ((p1.x + p2.x) / 2.0, (p1.y + p2.y) / 2.0)
}

/// Recursively splits the segment (h1, h2) at its midpoint until the two
/// endpoints are already connected by a triangulation edge, then marks that
/// edge as a constraint.  This mirrors CDT.NET's ConformToEdges behaviour.
fn conform_edge(cdt: &mut Cdt, h1: FixedVertexHandle, h2: FixedVertexHandle, depth: i32) {
    if h1 == h2 || depth > 52 {
        return;
    }
    if is_direct_edge(cdt, h1, h2) {
        let _ = cdt.add_constraint(h1, h2);
        return;
    }
    let (mx, my) = midpoint(cdt, h1, h2);
    let hm = match cdt.insert(Point2::new(mx, my)) {
        Ok(h) => h,
        Err(_) => return,
    };
    if hm == h1 || hm == h2 {
        return;
    }
    conform_edge(cdt, h1, hm, depth + 1);
    conform_edge(cdt, hm, h2, depth + 1);
}

/// Triangulate points with optional constraint edges using Spade.
/// Returns the number of inner (finite) triangles, or -1 on error/panic.
///
/// # Safety
/// All pointer arguments must be valid for `n_verts` / `n_edges` reads.
#[no_mangle]
pub unsafe extern "C" fn spade_cdt(
    xs: *const f64,
    ys: *const f64,
    n_verts: i32,
    edge_v1: *const i32,
    edge_v2: *const i32,
    n_edges: i32,
) -> i32 {
    let result = panic::catch_unwind(|| {
        let n = n_verts as usize;
        let ne = n_edges as usize;

        let mut cdt = ConstrainedDelaunayTriangulation::<Point2<f64>>::new();
        let mut handles = Vec::with_capacity(n);

        for i in 0..n {
            let x = *xs.add(i);
            let y = *ys.add(i);
            match cdt.insert(Point2::new(x, y)) {
                Ok(h) => handles.push(h),
                // Duplicate vertex — reuse the previous valid handle so that
                // constraint-edge indices still stay in bounds.
                Err(_) => {
                    if let Some(&last) = handles.last() {
                        handles.push(last);
                    }
                }
            }
        }

        for i in 0..ne {
            let v1 = *edge_v1.add(i) as usize;
            let v2 = *edge_v2.add(i) as usize;
            if v1 < handles.len() && v2 < handles.len() && v1 != v2 {
                let _ = cdt.add_constraint(handles[v1], handles[v2]);
            }
        }

        cdt.num_inner_faces() as i32
    });

    result.unwrap_or(-1)
}

/// Conforming Delaunay Triangulation using Spade.
/// For each constraint edge (v1, v2) the segment is recursively split at its
/// midpoint until both endpoints are already connected by a triangulation edge,
/// at which point that edge is marked as a constraint.  This mirrors the
/// behaviour of CDT.NET's ConformToEdges / artem-ogre/CDT's conformToEdges.
/// Returns the number of inner (finite) triangles, or -1 on error/panic.
///
/// # Safety
/// All pointer arguments must be valid for `n_verts` / `n_edges` reads.
#[no_mangle]
pub unsafe extern "C" fn spade_conform(
    xs: *const f64,
    ys: *const f64,
    n_verts: i32,
    edge_v1: *const i32,
    edge_v2: *const i32,
    n_edges: i32,
) -> i32 {
    let result = panic::catch_unwind(|| {
        let n = n_verts as usize;
        let ne = n_edges as usize;

        let mut cdt = Cdt::new();
        let mut handles = Vec::with_capacity(n);

        for i in 0..n {
            let x = *xs.add(i);
            let y = *ys.add(i);
            match cdt.insert(Point2::new(x, y)) {
                Ok(h) => handles.push(h),
                Err(_) => {
                    if let Some(&last) = handles.last() {
                        handles.push(last);
                    }
                }
            }
        }

        for i in 0..ne {
            let v1 = *edge_v1.add(i) as usize;
            let v2 = *edge_v2.add(i) as usize;
            if v1 < handles.len() && v2 < handles.len() && v1 != v2 {
                conform_edge(&mut cdt, handles[v1], handles[v2], 0);
            }
        }

        cdt.num_inner_faces() as i32
    });

    result.unwrap_or(-1)
}
