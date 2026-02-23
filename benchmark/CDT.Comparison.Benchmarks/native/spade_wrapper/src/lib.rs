// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

use std::panic;
use spade::{ConstrainedDelaunayTriangulation, Point2, Triangulation};

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
