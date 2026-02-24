// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System.Numerics;
using System.Runtime.CompilerServices;
using static CDT.CdtUtils;

namespace CDT;

/// <summary>
/// Errors thrown by the triangulation.
/// </summary>
public class TriangulationException : Exception
{
    /// <inheritdoc/>
    public TriangulationException(string message) : base(message) { }
}

/// <summary>Thrown when a duplicate vertex is detected during insertion.</summary>
public sealed class DuplicateVertexException : TriangulationException
{
    /// <summary>First duplicate vertex index.</summary>
    public int V1 { get; }

    /// <summary>Second duplicate vertex index (same position as V1).</summary>
    public int V2 { get; }

    /// <inheritdoc/>
    public DuplicateVertexException(int v1, int v2)
        : base($"Duplicate vertex detected: #{v1} is a duplicate of #{v2}")
    {
        V1 = v1;
        V2 = v2;
    }
}

/// <summary>
/// Thrown when a mutating method is called after the triangulation has been finalized
/// (i.e., after any <c>Erase*</c> method has been called).
/// </summary>
public sealed class TriangulationFinalizedException : TriangulationException
{
    /// <inheritdoc/>
    public TriangulationFinalizedException()
        : base("The triangulation has already been finalized. No further modifications are allowed after calling an Erase method.")
    { }
}

/// <summary>Thrown when intersecting constraint edges are detected and <see cref="IntersectingConstraintEdges.NotAllowed"/> is set.</summary>
public sealed class IntersectingConstraintsException : TriangulationException
{
    /// <summary>First constraint edge.</summary>
    public Edge E1 { get; }

    /// <summary>Second constraint edge.</summary>
    public Edge E2 { get; }

    /// <inheritdoc/>
    public IntersectingConstraintsException(Edge e1, Edge e2)
        : base($"Intersecting constraint edges: ({e1.V1},{e1.V2}) ∩ ({e2.V1},{e2.V2})")
    {
        E1 = e1;
        E2 = e2;
    }
}

/// <summary>
/// 2D constrained Delaunay triangulation.
/// Supports both constrained and conforming modes.
/// </summary>
/// <typeparam name="T">Floating-point coordinate type (float or double).</typeparam>
public sealed class Triangulation<T>
    where T : unmanaged, IFloatingPoint<T>, IMinMaxValue<T>, IRootFunctions<T>
{
    // -------------------------------------------------------------------------
    // Public state (read-only views)
    // -------------------------------------------------------------------------

    /// <summary>All vertices in the triangulation (including super-triangle vertices while not finalized).</summary>
    public ReadOnlyMemory<V2d<T>> Vertices => new(_vertices, 0, _verticesCount);

    /// <summary>All triangles in the triangulation.</summary>
    public ReadOnlyMemory<Triangle> Triangles => new(_triangles, 0, _trianglesCount);

    /// <summary>Set of constraint (fixed) edges.</summary>
    public IReadOnlySet<Edge> FixedEdges => _fixedEdges;

    /// <summary>
    /// Stores count of overlapping boundaries for a fixed edge.
    /// Only has entries for edges that represent overlapping boundaries.
    /// </summary>
    public IReadOnlyDictionary<Edge, ushort> OverlapCount => _overlapCount;

    /// <summary>
    /// Stores the list of original edges represented by a given fixed edge.
    /// Only populated when edges were split or overlap.
    /// </summary>
    public IReadOnlyDictionary<Edge, IReadOnlyList<Edge>> PieceToOriginals => _pieceToOriginalsView;

    // -------------------------------------------------------------------------
    // Private fields
    // -------------------------------------------------------------------------

    private V2d<T>[] _vertices = [];
    private int _verticesCount;
    private Triangle[] _triangles = [];
    private int _trianglesCount;
    private readonly HashSet<Edge> _fixedEdges = new();
    private readonly Dictionary<Edge, ushort> _overlapCount = new();
    private readonly Dictionary<Edge, List<Edge>> _pieceToOriginals = new();
    private readonly IReadOnlyDictionary<Edge, IReadOnlyList<Edge>> _pieceToOriginalsView;

    private readonly VertexInsertionOrder _insertionOrder;
    private readonly IntersectingConstraintEdges _intersectingEdgesStrategy;
    private readonly T _minDistToConstraintEdge;
    private readonly T _two;
    private SuperGeometryType _superGeomType;
    private int _nTargetVerts;

    // For each vertex: one adjacent triangle index
    private int[] _vertTris = [];
    private int _vertTrisCount;

    // KD-tree for nearest-point location
    private KdTree<T>? _kdTree;

    // -------------------------------------------------------------------------
    // Construction
    // -------------------------------------------------------------------------

    /// <summary>
    /// Creates a triangulation with default settings:
    /// <see cref="VertexInsertionOrder.Auto"/>,
    /// <see cref="IntersectingConstraintEdges.NotAllowed"/>,
    /// zero minimum distance.
    /// </summary>
    public Triangulation()
        : this(VertexInsertionOrder.Auto, IntersectingConstraintEdges.NotAllowed, T.Zero)
    { }

    /// <summary>Creates a triangulation with the specified insertion order.</summary>
    public Triangulation(VertexInsertionOrder insertionOrder)
        : this(insertionOrder, IntersectingConstraintEdges.NotAllowed, T.Zero)
    { }

    /// <summary>Creates a triangulation with explicit settings.</summary>
    public Triangulation(
        VertexInsertionOrder insertionOrder,
        IntersectingConstraintEdges intersectingEdgesStrategy,
        T minDistToConstraintEdge)
    {
        _insertionOrder = insertionOrder;
        _intersectingEdgesStrategy = intersectingEdgesStrategy;
        _minDistToConstraintEdge = minDistToConstraintEdge;
        _two = T.One + T.One;
        _superGeomType = SuperGeometryType.SuperTriangle;
        _nTargetVerts = 0;
        _pieceToOriginalsView = new CovariantReadOnlyDictionary<Edge, List<Edge>, IReadOnlyList<Edge>>(_pieceToOriginals);
    }

    // -------------------------------------------------------------------------
    // Public API – vertex insertion
    // -------------------------------------------------------------------------

    /// <summary>Inserts a list of vertices into the triangulation.</summary>
    public void InsertVertices(IReadOnlyList<V2d<T>> newVertices)
    {
        if (IsFinalized) throw new TriangulationFinalizedException();
        if (newVertices.Count == 0) return;

        bool isFirstInsertion = _kdTree == null && _verticesCount == 0;

        // Pre-allocate backing arrays once we know the incoming vertex count.
        // Euler's formula: a planar triangulation of N points has ~2N triangles.
        if (isFirstInsertion)
        {
            int n = newVertices.Count;
            ArrayEnsureCapacity(ref _vertices, n + Indices.SuperTriangleVertexCount);
            ArrayEnsureCapacity(ref _vertTris, n + Indices.SuperTriangleVertexCount);
            ArrayEnsureCapacity(ref _triangles, 2 * n + 4);
        }

        // Build bounding box of new vertices
        var box = new Box2d<T>();
        box.Envelop(newVertices);

        if (isFirstInsertion)
        {
            AddSuperTriangle(box);
        }
        else if (_kdTree == null)
        {
            // Subsequent calls: initialize the KD tree with all existing vertices
            InitKdTree();
        }

        int insertStart = _verticesCount;
        foreach (var v in newVertices)
        {
            AddNewVertex(v, Indices.NoNeighbor);
        }

        if (_insertionOrder == VertexInsertionOrder.Auto && isFirstInsertion)
        {
            // Use BFS KD-tree ordering for the first bulk insertion
            // Walk-start comes from the BFS parent, not the KD tree
            InsertVertices_KDTreeBFS(insertStart, box);
        }
        else if (_insertionOrder == VertexInsertionOrder.Auto)
        {
            // Subsequent calls: randomized with KD-tree walk-start
            InsertVertices_Randomized(insertStart);
        }
        else
        {
            // AsProvided: sequential order, KD-tree walk-start
            var stack = new Stack<int>(4);
            for (int iV = insertStart; iV < _verticesCount; iV++)
            {
                InsertVertex(iV, stack);
            }
        }
    }

    // -------------------------------------------------------------------------
    // Public API – edge insertion (constrained DT)
    // -------------------------------------------------------------------------

    /// <summary>Inserts constraint edges (constrained Delaunay triangulation).</summary>
    public void InsertEdges(IReadOnlyList<Edge> edges)
    {
        if (IsFinalized) throw new TriangulationFinalizedException();
        var remaining = new List<Edge>(4);
        var tppTasks = new List<TriangulatePseudoPolygonTask>(8);
        var polyL = new List<int>(8);
        var polyR = new List<int>(8);
        var outerTris = new Dictionary<Edge, int>();
        var intersected = new List<int>(8);
        foreach (var e in edges)
        {
            remaining.Clear();
            remaining.Add(new Edge(e.V1 + _nTargetVerts, e.V2 + _nTargetVerts));
            while (remaining.Count > 0)
            {
                Edge edge = remaining[^1];
                remaining.RemoveAt(remaining.Count - 1);
                InsertEdgeIteration(edge, new Edge(e.V1 + _nTargetVerts, e.V2 + _nTargetVerts), remaining, tppTasks, polyL, polyR, outerTris, intersected);
            }
        }
    }

    // -------------------------------------------------------------------------
    // Public API – conforming DT
    // -------------------------------------------------------------------------

    /// <summary>
    /// Inserts constraint edges for a conforming Delaunay triangulation.
    /// May add new vertices (midpoints) until edges are represented.
    /// </summary>
    public void ConformToEdges(IReadOnlyList<Edge> edges)
    {
        if (IsFinalized) throw new TriangulationFinalizedException();
        var remaining = new List<ConformToEdgeTask>(8);
        var flipStack = new Stack<int>(4);
        var flippedFixed = new List<Edge>(8);
        foreach (var e in edges)
        {
            var shifted = new Edge(e.V1 + _nTargetVerts, e.V2 + _nTargetVerts);
            remaining.Clear();
            remaining.Add(new ConformToEdgeTask(shifted, new List<Edge> { shifted }, 0));
            while (remaining.Count > 0)
            {
                var task = remaining[^1];
                remaining.RemoveAt(remaining.Count - 1);
                ConformToEdgeIteration(task.Edge, task.Originals, task.Overlaps, remaining, flipStack, flippedFixed);
            }
        }
    }

    // -------------------------------------------------------------------------
    // Public API – finalization
    // -------------------------------------------------------------------------

    /// <summary>Removes the super-triangle to produce a convex hull triangulation.</summary>
    public void EraseSuperTriangle()
    {
        if (IsFinalized) throw new TriangulationFinalizedException();
        if (_superGeomType != SuperGeometryType.SuperTriangle) return;
        var toErase = new HashSet<int>();
        for (int i = 0; i < _trianglesCount; i++)
        {
            if (CdtUtils.TouchesSuperTriangle(_triangles[i]))
                toErase.Add(i);
        }
        FinalizeTriangulation(toErase);
    }

    /// <summary>Removes all outer triangles (flood-fill from super-triangle vertex until a constraint edge).</summary>
    public void EraseOuterTriangles()
    {
        if (IsFinalized) throw new TriangulationFinalizedException();
        if (_vertTris[0] == Indices.NoNeighbor)
            throw new TriangulationException("No vertex triangle data – already finalized?");
        var seeds = new Stack<int>();
        seeds.Push(_vertTris[0]);
        var toErase = GrowToBoundary(seeds);
        FinalizeTriangulation(toErase);
    }

    /// <summary>
    /// Removes outer triangles and automatically detects and removes holes
    /// using even-odd depth rule.
    /// </summary>
    public void EraseOuterTrianglesAndHoles()
    {
        if (IsFinalized) throw new TriangulationFinalizedException();
        var depths = CalculateTriangleDepths();
        var toErase = new HashSet<int>();
        for (int i = 0; i < _trianglesCount; i++)
        {
            if (depths[i] % 2 == 0) toErase.Add(i);
        }
        FinalizeTriangulation(toErase);
    }

    /// <summary>
    /// Indicates whether the triangulation has been finalized (i.e., one of the
    /// Erase methods was called). Further modification is not possible.
    /// </summary>
    public bool IsFinalized => _vertTrisCount == 0 && _verticesCount > 0;

    // -------------------------------------------------------------------------
    // Internal helpers – super-triangle setup
    // -------------------------------------------------------------------------

    private void AddSuperTriangle(Box2d<T> box)
    {
        _nTargetVerts = Indices.SuperTriangleVertexCount;
        _superGeomType = SuperGeometryType.SuperTriangle;

        T cx = (box.Min.X + box.Max.X) / _two;
        T cy = (box.Min.Y + box.Max.Y) / _two;
        T w = box.Max.X - box.Min.X;
        T h = box.Max.Y - box.Min.Y;
        T r = T.Max(w, h);
        r = T.Max(_two * r, T.One);

        // Guard against very large numbers
        while (cy <= cy - r) r = _two * r;

        T R = _two * r;
        T cos30 = ParseT("0.8660254037844386");
        T shiftX = R * cos30;

        var v1 = new V2d<T>(cx - shiftX, cy - r);
        var v2 = new V2d<T>(cx + shiftX, cy - r);
        var v3 = new V2d<T>(cx, cy + R);

        AddNewVertex(v1, 0);
        AddNewVertex(v2, 0);
        AddNewVertex(v3, 0);

        AddTriangle(new Triangle(0, 1, 2, Indices.NoNeighbor, Indices.NoNeighbor, Indices.NoNeighbor));

        if (_insertionOrder != VertexInsertionOrder.Auto)
        {
            InitKdTree();
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void AddNewVertex(V2d<T> pos, int iTri)
    {
        ArrayAdd(ref _vertices, ref _verticesCount, pos);
        ArrayAdd(ref _vertTris, ref _vertTrisCount, iTri);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int AddTriangle() => AddTriangle(new Triangle());

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int AddTriangle(Triangle t)
    {
        int idx = _trianglesCount;
        ArrayAdd(ref _triangles, ref _trianglesCount, t);
        return idx;
    }

    // -------------------------------------------------------------------------
    // Low-level array-growth helpers
    // -------------------------------------------------------------------------

    // Maximum element count for which a temporary int[] is stackalloc'd instead of heap-allocated.
    private const int StackAllocThreshold = 512;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ArrayAdd<TItem>(ref TItem[] arr, ref int count, TItem item)
    {
        if (count == arr.Length)
            Array.Resize(ref arr, arr.Length == 0 ? 4 : arr.Length * 2);
        arr[count++] = item;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ArrayEnsureCapacity<TItem>(ref TItem[] arr, int capacity)
    {
        if (arr.Length < capacity)
            Array.Resize(ref arr, capacity);
    }

    // -------------------------------------------------------------------------
    // Internal helpers – vertex insertion
    // -------------------------------------------------------------------------

    private void InsertVertex(int iVert, int walkStart, Stack<int> stack)
    {
        var (iT, iTopo) = WalkingSearchTrianglesAt(iVert, walkStart);
        if (iTopo == Indices.NoNeighbor)
            InsertVertexInsideTriangle(iVert, iT, stack);
        else
            InsertVertexOnEdge(iVert, iT, iTopo, handleFixedSplitEdge: true, stack);
        EnsureDelaunayByEdgeFlips(iVert, stack);
        TryAddVertexToLocator(iVert);
    }

    private void InsertVertex(int iVert, int walkStart)
    {
        var stack = new Stack<int>(4);
        InsertVertex(iVert, walkStart, stack);
    }

    private void InsertVertex(int iVert, Stack<int> stack)
    {
        int near = _kdTree != null
            ? _kdTree.Nearest(_vertices[iVert].X, _vertices[iVert].Y, _vertices)
            : 0;
        InsertVertex(iVert, near, stack);
    }

    private void InsertVertex(int iVert)
    {
        // Walk-start from KD-tree nearest point, or vertex 0 as fallback
        int near = _kdTree != null
            ? _kdTree.Nearest(_vertices[iVert].X, _vertices[iVert].Y, _vertices)
            : 0;
        InsertVertex(iVert, near);
    }

    private void InsertVertices_Randomized(int superGeomVertCount)
    {
        int count = _verticesCount - superGeomVertCount;
        Span<int> indices = count <= StackAllocThreshold ? stackalloc int[count] : new int[count];
        for (int i = 0; i < count; i++) { indices[i] = superGeomVertCount + i; }
        for (int i = count - 1; i > 0; i--)
        {
            int j = Random.Shared.Next(i + 1);
            (indices[i], indices[j]) = (indices[j], indices[i]);
        }
        var stack = new Stack<int>(4);
        foreach (int iV in indices) { InsertVertex(iV, stack); }
    }

    private void InsertVertices_KDTreeBFS(int superGeomVertCount, Box2d<T> box)
    {
        int vertexCount = _verticesCount - superGeomVertCount;
        if (vertexCount <= 0) { return; }

        Span<int> indices = vertexCount <= StackAllocThreshold ? stackalloc int[vertexCount] : new int[vertexCount];
        for (int i = 0; i < vertexCount; i++) { indices[i] = superGeomVertCount + i; }

        var queue = new Queue<(int lo, int hi, T boxMinX, T boxMinY, T boxMaxX, T boxMaxY, int parent)>();
        queue.Enqueue((0, vertexCount, box.Min.X, box.Min.Y, box.Max.X, box.Max.Y, 0));

        var stack = new Stack<int>(4);
        while (queue.Count > 0)
        {
            var (lo, hi, boxMinX, boxMinY, boxMaxX, boxMaxY, parent) = queue.Dequeue();
            int len = hi - lo;
            if (len == 0) { continue; }
            if (len == 1) { InsertVertex(indices[lo], parent, stack); continue; }

            int midPos = lo + len / 2;

            if (T.CreateChecked(boxMaxX - boxMinX) >= T.CreateChecked(boxMaxY - boxMinY))
            {
                NthElement(indices, lo, midPos, hi, new VertexXComparer(_vertices));
                T split = _vertices[indices[midPos]].X;
                InsertVertex(indices[midPos], parent, stack);
                if (lo < midPos) { queue.Enqueue((lo, midPos, boxMinX, boxMinY, split, boxMaxY, indices[midPos])); }
                if (midPos + 1 < hi) { queue.Enqueue((midPos + 1, hi, split, boxMinY, boxMaxX, boxMaxY, indices[midPos])); }
            }
            else
            {
                NthElement(indices, lo, midPos, hi, new VertexYComparer(_vertices));
                T split = _vertices[indices[midPos]].Y;
                InsertVertex(indices[midPos], parent, stack);
                if (lo < midPos) { queue.Enqueue((lo, midPos, boxMinX, boxMinY, boxMaxX, split, indices[midPos])); }
                if (midPos + 1 < hi) { queue.Enqueue((midPos + 1, hi, boxMinX, split, boxMaxX, boxMaxY, indices[midPos])); }
            }
        }
    }

    // -------------------------------------------------------------------------
    // nth_element — O(n) average quickselect for spatial BFS partitioning
    // -------------------------------------------------------------------------

    private readonly struct VertexXComparer : IComparer<int>
    {
        private readonly V2d<T>[] _vertices;
        public VertexXComparer(V2d<T>[] vertices) => _vertices = vertices;
        public int Compare(int a, int b) => _vertices[a].X.CompareTo(_vertices[b].X);
    }

    private readonly struct VertexYComparer : IComparer<int>
    {
        private readonly V2d<T>[] _vertices;
        public VertexYComparer(V2d<T>[] vertices) => _vertices = vertices;
        public int Compare(int a, int b) => _vertices[a].Y.CompareTo(_vertices[b].Y);
    }

    /// <summary>
    /// Rearranges <paramref name="arr"/> in [<paramref name="lo"/>, <paramref name="hi"/>)
    /// so the element at position <paramref name="nth"/> is the one that would be there
    /// after a full sort; elements before it are ≤ it and elements after are ≥ it.
    /// </summary>
    private static void NthElement<TComparer>(Span<int> arr, int lo, int nth, int hi, TComparer cmp)
        where TComparer : struct, IComparer<int>
    {
        while (lo < hi - 1)
        {
            // Pivot: middle element avoids worst-case on sorted input
            int mid = lo + (hi - lo) / 2;
            (arr[mid], arr[hi - 1]) = (arr[hi - 1], arr[mid]);
            int store = lo;
            for (int i = lo; i < hi - 1; i++)
            {
                if (cmp.Compare(arr[i], arr[hi - 1]) < 0)
                {
                    (arr[store], arr[i]) = (arr[i], arr[store]);
                    store++;
                }
            }
            (arr[store], arr[hi - 1]) = (arr[hi - 1], arr[store]);
            if (store < nth) lo = store + 1;
            else if (store > nth) hi = store;
            else return;
        }
    }

    private void InsertVertex_FlipFixedEdges(int iV, Stack<int> stack, List<Edge> flipped)
    {
        flipped.Clear();
        // Use KD-tree if available, otherwise fall back to vertex 0 (first super-triangle vertex)
        int near = _kdTree != null
            ? _kdTree.Nearest(_vertices[iV].X, _vertices[iV].Y, _vertices)
            : 0;
        var (iT, iTopo) = WalkingSearchTrianglesAt(iV, near);
        if (iTopo == Indices.NoNeighbor)
            InsertVertexInsideTriangle(iV, iT, stack);
        else
            InsertVertexOnEdge(iV, iT, iTopo, handleFixedSplitEdge: false, stack);

        int _dbgFlipIter2 = 0;
        while (stack.Count > 0)
        {
            if (++_dbgFlipIter2 > 1_000_000) throw new InvalidOperationException($"InsertVertex_FlipFixed infinite loop, iV={iV}");
            int tri = stack.Pop();
            EdgeFlipInfo(tri, iV,
                out int itopo, out int iv2, out int iv3, out int iv4,
                out int n1, out int n2, out int n3, out int n4);

            if (itopo != Indices.NoNeighbor && IsFlipNeeded(iV, iv2, iv3, iv4))
            {
                var flippedEdge = new Edge(iv2, iv4);
                if (_fixedEdges.Contains(flippedEdge))
                    flipped.Add(flippedEdge);
                FlipEdge(tri, itopo, iV, iv2, iv3, iv4, n1, n2, n3, n4);
                stack.Push(tri);
                stack.Push(itopo);
            }
        }
        TryAddVertexToLocator(iV);
    }

    private void EnsureDelaunayByEdgeFlips(int iV1, Stack<int> triStack)
    {
        int _dbgFlipIter = 0;
        while (triStack.Count > 0)
        {
            if (++_dbgFlipIter > 1_000_000) throw new InvalidOperationException($"EnsureDelaunayByEdgeFlips infinite loop, iV1={iV1}, stack size={triStack.Count}");

            int iT = triStack.Pop();
            EdgeFlipInfo(iT, iV1,
                out int iTopo, out int iV2, out int iV3, out int iV4,
                out int n1, out int n2, out int n3, out int n4);
            if (iTopo != Indices.NoNeighbor && IsFlipNeeded(iV1, iV2, iV3, iV4))
            {
                FlipEdge(iT, iTopo, iV1, iV2, iV3, iV4, n1, n2, n3, n4);
                triStack.Push(iT);
                triStack.Push(iTopo);
            }
        }
    }

    // -------------------------------------------------------------------------
    // Locate triangle containing a vertex
    // -------------------------------------------------------------------------

    private (int iT, int iTopo) WalkingSearchTrianglesAt(int iVert, int startVertex)
    {
        var v = _vertices[iVert];
        int iT = WalkTriangles(startVertex, v);
        var t = _triangles[iT];

        var loc = LocatePointTriangle(v, _vertices[t.V0], _vertices[t.V1], _vertices[t.V2]);

        if (loc == PtTriLocation.Outside)
        {
            // Walk hit a degenerate cycle; fall back to brute-force linear scan
            iT = FindTriangleLinear(v, out loc);
            t = _triangles[iT];
        }

        if (loc == PtTriLocation.OnVertex)
        {
            int iDupe = _vertices[t.V0] == v ? t.V0 : _vertices[t.V1] == v ? t.V1 : t.V2;
            throw new DuplicateVertexException(iVert - _nTargetVerts, iDupe - _nTargetVerts);
        }

        int iNeigh = CdtUtils.IsOnEdge(loc)
            ? t.GetNeighbor(CdtUtils.EdgeNeighborFromLocation(loc))
            : Indices.NoNeighbor;
        return (iT, iNeigh);
    }

    /// <summary>Brute-force O(n) fallback: scan all triangles to find the one containing <paramref name="pos"/>.</summary>
    private int FindTriangleLinear(V2d<T> pos, out PtTriLocation loc)
    {
        for (int i = 0; i < _trianglesCount; i++)
        {
            var t = _triangles[i];
            loc = LocatePointTriangle(pos, _vertices[t.V0], _vertices[t.V1], _vertices[t.V2]);
            if (loc != PtTriLocation.Outside)
            {
                return i;
            }
        }
        throw new TriangulationException($"No triangle found for point ({pos.X}, {pos.Y}).");
    }

    private int WalkTriangles(int startVertex, V2d<T> pos)
    {
        int currTri = _vertTris[startVertex];
        for (int guard = 0; guard < 1_000_000; guard++)
        {
            var t = _triangles[currTri];
            bool found = true;
            int offset = guard % 3;
            for (int i = 0; i < 3; i++)
            {
                int idx = (i + offset) % 3;
                int vStart = t.GetVertex(idx);
                int vEnd = t.GetVertex(CdtUtils.Ccw(idx));
                var loc = LocatePointLine(pos, _vertices[vStart], _vertices[vEnd]);
                int iN = t.GetNeighbor(idx);
                if (loc == PtLineLocation.Right && iN != Indices.NoNeighbor)
                {
                    currTri = iN;
                    found = false;
                    break;
                }
            }
            if (found) { return currTri; }
        }
        // Walk did not converge (very degenerate triangulation) — let caller fall back.
        return currTri;
    }

    // -------------------------------------------------------------------------
    // Insert vertex inside triangle or on edge
    // -------------------------------------------------------------------------

    private void InsertVertexInsideTriangle(int v, int iT, Stack<int> stack)
    {
        int iNewT1 = AddTriangle();
        int iNewT2 = AddTriangle();

        var t = _triangles[iT];
        int v1 = t.V0, v2 = t.V1, v3 = t.V2;
        int n1 = t.N0, n2 = t.N1, n3 = t.N2;

        _triangles[iNewT1] = new Triangle(v2, v3, v, n2, iNewT2, iT);
        _triangles[iNewT2] = new Triangle(v3, v1, v, n3, iT, iNewT1);
        _triangles[iT] = new Triangle(v1, v2, v, n1, iNewT1, iNewT2);

        SetAdjacentTriangle(v, iT);
        SetAdjacentTriangle(v3, iNewT1);
        ChangeNeighbor(n2, iT, iNewT1);
        ChangeNeighbor(n3, iT, iNewT2);

        stack.Clear();
        stack.Push(iT);
        stack.Push(iNewT1);
        stack.Push(iNewT2);
    }

    private void InsertVertexOnEdge(int v, int iT1, int iT2, bool handleFixedSplitEdge, Stack<int> stack)
    {
        int iTnew1 = AddTriangle();
        int iTnew2 = AddTriangle();

        var t1 = _triangles[iT1];
        int i1 = CdtUtils.NeighborIndex(t1, iT2);
        int v1 = t1.GetVertex(CdtUtils.OpposedVertexIndex(i1));
        int v2 = t1.GetVertex(CdtUtils.Ccw(CdtUtils.OpposedVertexIndex(i1)));
        int n1 = t1.GetNeighbor(CdtUtils.OpposedVertexIndex(i1));
        int n4 = t1.GetNeighbor(CdtUtils.Cw(CdtUtils.OpposedVertexIndex(i1)));

        var t2 = _triangles[iT2];
        int i2 = CdtUtils.NeighborIndex(t2, iT1);
        int v3 = t2.GetVertex(CdtUtils.OpposedVertexIndex(i2));
        int v4 = t2.GetVertex(CdtUtils.Ccw(CdtUtils.OpposedVertexIndex(i2)));
        int n3 = t2.GetNeighbor(CdtUtils.OpposedVertexIndex(i2));
        int n2 = t2.GetNeighbor(CdtUtils.Cw(CdtUtils.OpposedVertexIndex(i2)));

        _triangles[iT1] = new Triangle(v, v1, v2, iTnew1, n1, iT2);
        _triangles[iT2] = new Triangle(v, v2, v3, iT1, n2, iTnew2);
        _triangles[iTnew1] = new Triangle(v, v4, v1, iTnew2, n4, iT1);
        _triangles[iTnew2] = new Triangle(v, v3, v4, iT2, n3, iTnew1);

        SetAdjacentTriangle(v, iT1);
        SetAdjacentTriangle(v4, iTnew1);
        ChangeNeighbor(n4, iT1, iTnew1);
        ChangeNeighbor(n3, iT2, iTnew2);

        if (handleFixedSplitEdge)
        {
            var sharedEdge = new Edge(v2, v4);
            if (_fixedEdges.Contains(sharedEdge))
                SplitFixedEdge(sharedEdge, v);
        }

        stack.Clear();
        stack.Push(iT1);
        stack.Push(iTnew2);
        stack.Push(iT2);
        stack.Push(iTnew1);
    }

    // -------------------------------------------------------------------------
    // Edge flip
    // -------------------------------------------------------------------------

    private void EdgeFlipInfo(
        int iT, int iV1,
        out int iTopo, out int iV2, out int iV3, out int iV4,
        out int n1, out int n2, out int n3, out int n4)
    {
        var t = _triangles[iT];
        if (t.V0 == iV1)
        {
            iV2 = t.V1; iV4 = t.V2;
            n1 = t.N0; n3 = t.N2;
            iTopo = t.N1;
        }
        else if (t.V1 == iV1)
        {
            iV2 = t.V2; iV4 = t.V0;
            n1 = t.N1; n3 = t.N0;
            iTopo = t.N2;
        }
        else
        {
            iV2 = t.V0; iV4 = t.V1;
            n1 = t.N2; n3 = t.N1;
            iTopo = t.N0;
        }

        n2 = Indices.NoNeighbor;
        n4 = Indices.NoNeighbor;
        iV3 = Indices.NoVertex;

        if (iTopo == Indices.NoNeighbor) return;

        var tOpo = _triangles[iTopo];
        int oi = CdtUtils.NeighborIndex(tOpo, iT);
        int ov = CdtUtils.OpposedVertexIndex(oi);
        iV3 = tOpo.GetVertex(ov);
        // n2 and n4 use the NEIGHBOR position (oi), not the vertex position (ov)
        n2 = tOpo.GetNeighbor(CdtUtils.Ccw(oi));
        n4 = tOpo.GetNeighbor(CdtUtils.Cw(oi));
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private bool IsFlipNeeded(int iV1, int iV2, int iV3, int iV4)
    {
        // Skip HashSet lookup when there are no fixed edges (pure vertex-insertion path).
        if (_fixedEdges.Count > 0 && _fixedEdges.Contains(new Edge(iV2, iV4))) return false;

        var v1 = _vertices[iV1];
        var v2 = _vertices[iV2];
        var v3 = _vertices[iV3];
        var v4 = _vertices[iV4];

        if (_superGeomType == SuperGeometryType.SuperTriangle)
        {
            int st = Indices.SuperTriangleVertexCount;
            if (iV1 < st)
            {
                if (iV2 < st)
                    return LocatePointLine(v2, v3, v4) == LocatePointLine(v1, v3, v4);
                if (iV4 < st)
                    return LocatePointLine(v4, v2, v3) == LocatePointLine(v1, v2, v3);
                return false;
            }
            if (iV3 < st)
            {
                if (iV2 < st)
                    return LocatePointLine(v2, v1, v4) == LocatePointLine(v3, v1, v4);
                if (iV4 < st)
                    return LocatePointLine(v4, v2, v1) == LocatePointLine(v3, v2, v1);
                return false;
            }
            if (iV2 < st)
                return LocatePointLine(v2, v3, v4) == LocatePointLine(v1, v3, v4);
            if (iV4 < st)
                return LocatePointLine(v4, v2, v3) == LocatePointLine(v1, v2, v3);
        }
        return IsInCircumcircle(v1, v2, v3, v4);
    }

    private void FlipEdge(
        int iT, int iTopo,
        int v1, int v2, int v3, int v4,
        int n1, int n2, int n3, int n4)
    {
        _triangles[iT] = new Triangle(v4, v1, v3, n3, iTopo, n4);
        _triangles[iTopo] = new Triangle(v2, v3, v1, n2, iT, n1);
        ChangeNeighbor(n1, iT, iTopo);
        ChangeNeighbor(n4, iTopo, iT);
        if (!IsFinalized)
        {
            SetAdjacentTriangle(v4, iT);
            SetAdjacentTriangle(v2, iTopo);
        }
    }

    // -------------------------------------------------------------------------
    // Constraint edge insertion
    // -------------------------------------------------------------------------

    private void InsertEdgeIteration(
        Edge edge, Edge originalEdge,
        List<Edge> remaining,
        List<TriangulatePseudoPolygonTask> tppIterations,
        List<int> polyL,
        List<int> polyR,
        Dictionary<Edge, int> outerTris,
        List<int> intersected)
    {
        int iA = edge.V1, iB = edge.V2;
        if (iA == iB) return;

        if (HasEdge(iA, iB))
        {
            FixEdge(edge, originalEdge);
            return;
        }

        var a = _vertices[iA];
        var b = _vertices[iB];
        T distTol = _minDistToConstraintEdge == T.Zero
            ? T.Zero
            : _minDistToConstraintEdge * CdtUtils.Distance(a, b);

        var (iT, iVL, iVR) = IntersectedTriangle(iA, a, b, distTol);
        if (iT == Indices.NoNeighbor)
        {
            var edgePart = new Edge(iA, iVL);
            FixEdge(edgePart, originalEdge);
            remaining.Add(new Edge(iVL, iB));
            return;
        }

        polyL.Clear();
        polyR.Clear();
        outerTris.Clear();
        intersected.Clear();

        polyL.Add(iA); polyL.Add(iVL);
        polyR.Add(iA); polyR.Add(iVR);
        outerTris[new Edge(iA, iVL)] = CdtUtils.EdgeNeighbor(_triangles[iT], iA, iVL);
        outerTris[new Edge(iA, iVR)] = CdtUtils.EdgeNeighbor(_triangles[iT], iA, iVR);
        intersected.Add(iT);

        int iV = iA;
        var t = _triangles[iT];

        while (!t.ContainsVertex(iB))
        {
            int iTopo = CdtUtils.OpposedTriangle(t, iV);
            var tOpo = _triangles[iTopo];
            int iVopo = CdtUtils.OpposedVertex(tOpo, iT);

            HandleIntersectingEdgeStrategy(iVL, iVR, iA, iB, iT, iTopo, originalEdge, a, b, distTol, remaining, tppIterations, out bool @return);
            if (@return) return;

            var loc = LocatePointLine(_vertices[iVopo], a, b, distTol);
            if (loc == PtLineLocation.Left)
            {
                var e = new Edge(polyL[^1], iVopo);
                int outer = CdtUtils.EdgeNeighbor(tOpo, e.V1, e.V2);
                if (!outerTris.TryAdd(e, outer)) outerTris[e] = Indices.NoNeighbor;
                polyL.Add(iVopo);
                iV = iVL;
                iVL = iVopo;
            }
            else if (loc == PtLineLocation.Right)
            {
                var e = new Edge(polyR[^1], iVopo);
                int outer = CdtUtils.EdgeNeighbor(tOpo, e.V1, e.V2);
                if (!outerTris.TryAdd(e, outer)) outerTris[e] = Indices.NoNeighbor;
                polyR.Add(iVopo);
                iV = iVR;
                iVR = iVopo;
            }
            else // on line
            {
                iB = iVopo;
            }

            intersected.Add(iTopo);
            iT = iTopo;
            t = _triangles[iT];
        }

        outerTris[new Edge(polyL[^1], iB)] = CdtUtils.EdgeNeighbor(t, polyL[^1], iB);
        outerTris[new Edge(polyR[^1], iB)] = CdtUtils.EdgeNeighbor(t, polyR[^1], iB);
        polyL.Add(iB);
        polyR.Add(iB);

        // Ensure start/end vertices have valid non-intersected triangle
        if (_vertTris[iA] == intersected[0]) PivotVertexTriangleCW(iA);
        if (_vertTris[iB] == intersected[^1]) PivotVertexTriangleCW(iB);

        polyR.Reverse();

        // Re-use intersected triangles
        int iTL = intersected[^1]; intersected.RemoveAt(intersected.Count - 1);
        int iTR = intersected[^1]; intersected.RemoveAt(intersected.Count - 1);

        TriangulatePseudoPolygon(polyL, outerTris, iTL, iTR, intersected, tppIterations);
        TriangulatePseudoPolygon(polyR, outerTris, iTR, iTL, intersected, tppIterations);

        if (iB != edge.V2)
        {
            FixEdge(new Edge(iA, iB), originalEdge);
            remaining.Add(new Edge(iB, edge.V2));
        }
        else
        {
            FixEdge(edge, originalEdge);
        }
    }

    private void HandleIntersectingEdgeStrategy(
        int iVL, int iVR, int iA, int iB, int iT, int iTopo,
        Edge originalEdge, V2d<T> a, V2d<T> b, T distTol,
        List<Edge> remaining, List<TriangulatePseudoPolygonTask> tppIterations,
        out bool @return)
    {
        @return = false;
        var edgeLR = new Edge(iVL, iVR);
        switch (_intersectingEdgesStrategy)
        {
            case IntersectingConstraintEdges.NotAllowed:
                if (_fixedEdges.Contains(edgeLR))
                {
                    var e1 = originalEdge;
                    var e2 = edgeLR;
                    if (_pieceToOriginals.TryGetValue(e2, out var origE2) && origE2.Count > 0) e2 = origE2[0];
                    e1 = new Edge(e1.V1 - _nTargetVerts, e1.V2 - _nTargetVerts);
                    e2 = new Edge(e2.V1 - _nTargetVerts, e2.V2 - _nTargetVerts);
                    throw new IntersectingConstraintsException(e1, e2);
                }
                break;

            case IntersectingConstraintEdges.TryResolve:
                if (_fixedEdges.Contains(edgeLR))
                {
                    var newV = IntersectionPosition(_vertices[iA], _vertices[iB], _vertices[iVL], _vertices[iVR]);
                    int iNewVert = SplitFixedEdgeAt(edgeLR, newV, iT, iTopo);
                    remaining.Add(new Edge(iA, iNewVert));
                    remaining.Add(new Edge(iNewVert, iB));
                    @return = true;
                }
                break;
        }
    }

    // -------------------------------------------------------------------------
    // Conforming edge insertion
    // -------------------------------------------------------------------------

    private void ConformToEdgeIteration(
        Edge edge, List<Edge> originals, ushort overlaps,
        List<ConformToEdgeTask> remaining,
        Stack<int> flipStack,
        List<Edge> flippedFixed)
    {
        int iA = edge.V1, iB = edge.V2;
        if (iA == iB) return;

        if (HasEdge(iA, iB))
        {
            FixEdge(edge);
            if (overlaps > 0) _overlapCount[edge] = overlaps;
            if (originals.Count > 0 && edge != originals[0])
                InsertUnique(_pieceToOriginals.GetOrAdd(edge), originals);
            return;
        }

        var a = _vertices[iA];
        var b = _vertices[iB];
        T distTol = _minDistToConstraintEdge == T.Zero
            ? T.Zero
            : _minDistToConstraintEdge * CdtUtils.Distance(a, b);

        var (iT, iVleft, iVright) = IntersectedTriangle(iA, a, b, distTol);
        if (iT == Indices.NoNeighbor)
        {
            var part = new Edge(iA, iVleft);
            FixEdge(part);
            if (overlaps > 0) _overlapCount[part] = overlaps;
            InsertUnique(_pieceToOriginals.GetOrAdd(part), originals);
            remaining.Add(new ConformToEdgeTask(new Edge(iVleft, iB), originals, overlaps));
            return;
        }

        int iV = iA;
        var t = _triangles[iT];
        while (!t.ContainsVertex(iB))
        {
            int iTopo = CdtUtils.OpposedTriangle(t, iV);
            var tOpo = _triangles[iTopo];
            int iVopo = CdtUtils.OpposedVertex(tOpo, iT);
            var vOpo = _vertices[iVopo];

            HandleConformIntersecting(iVleft, iVright, iA, iB, iT, iTopo,
                originals, overlaps, remaining, out bool @return);
            if (@return) return;

            iT = iTopo;
            t = _triangles[iT];
            var loc = LocatePointLine(vOpo, a, b, distTol);
            if (loc == PtLineLocation.Left) { iV = iVleft; iVleft = iVopo; }
            else if (loc == PtLineLocation.Right) { iV = iVright; iVright = iVopo; }
            else iB = iVopo; // on line
        }

        if (iB != edge.V2)
            remaining.Add(new ConformToEdgeTask(new Edge(iB, edge.V2), originals, overlaps));

        // Insert midpoint and recurse
        int iMid = _verticesCount;
        var start = _vertices[iA];
        var end = _vertices[iB];
        AddNewVertex(new V2d<T>((start.X + end.X) / _two, (start.Y + end.Y) / _two), Indices.NoNeighbor);

        InsertVertex_FlipFixedEdges(iMid, flipStack, flippedFixed);

        remaining.Add(new ConformToEdgeTask(new Edge(iMid, iB), originals, overlaps));
        remaining.Add(new ConformToEdgeTask(new Edge(iA, iMid), originals, overlaps));

        // Re-insert flipped fixed edges
        foreach (var fe in flippedFixed)
        {
            _fixedEdges.Remove(fe);
            ushort prevOv = _overlapCount.TryGetValue(fe, out var ov) ? ov : (ushort)0;
            _overlapCount.Remove(fe);
            var prevOrig = _pieceToOriginals.TryGetValue(fe, out var po) ? po : new List<Edge> { fe };
            remaining.Add(new ConformToEdgeTask(fe, prevOrig, prevOv));
        }
    }

    private void HandleConformIntersecting(
        int iVleft, int iVright, int iA, int iB, int iT, int iTopo,
        List<Edge> originals, ushort overlaps,
        List<ConformToEdgeTask> remaining,
        out bool @return)
    {
        @return = false;
        var edgeLR = new Edge(iVleft, iVright);
        switch (_intersectingEdgesStrategy)
        {
            case IntersectingConstraintEdges.NotAllowed:
                if (_fixedEdges.Contains(edgeLR))
                {
                    var e1 = _pieceToOriginals.TryGetValue(edgeLR, out var po1) && po1.Count > 0 ? po1[0] : edgeLR;
                    throw new IntersectingConstraintsException(
                        new Edge(e1.V1 - _nTargetVerts, e1.V2 - _nTargetVerts),
                        edgeLR);
                }
                break;
            case IntersectingConstraintEdges.TryResolve:
                if (_fixedEdges.Contains(edgeLR))
                {
                    var newV = IntersectionPosition(_vertices[iA], _vertices[iB], _vertices[iVleft], _vertices[iVright]);
                    int iNewVert = SplitFixedEdgeAt(edgeLR, newV, iT, iTopo);
                    remaining.Add(new ConformToEdgeTask(new Edge(iNewVert, iB), originals, overlaps));
                    remaining.Add(new ConformToEdgeTask(new Edge(iA, iNewVert), originals, overlaps));
                    @return = true;
                }
                break;
        }
    }

    // -------------------------------------------------------------------------
    // Pseudo-polygon triangulation (for constrained edges)
    // -------------------------------------------------------------------------

    // Task: (iA, iB, iT, iParent, iInParent)
    private readonly record struct TriangulatePseudoPolygonTask(int IA, int IB, int IT, int IParent, int IInParent);

    private void TriangulatePseudoPolygon(
        List<int> poly,
        Dictionary<Edge, int> outerTris,
        int iT, int iN,
        List<int> trianglesToReuse,
        List<TriangulatePseudoPolygonTask> iterations)
    {
        iterations.Clear();
        iterations.Add(new TriangulatePseudoPolygonTask(0, poly.Count - 1, iT, iN, 0));
        while (iterations.Count > 0)
        {
            TriangulatePseudoPolygonIteration(poly, outerTris, trianglesToReuse, iterations);
        }
    }

    private void TriangulatePseudoPolygonIteration(
        List<int> poly,
        Dictionary<Edge, int> outerTris,
        List<int> trianglesToReuse,
        List<TriangulatePseudoPolygonTask> iterations)
    {
        var (iA, iB, iT, iParent, iInParent) = iterations[^1];
        iterations.RemoveAt(iterations.Count - 1);

        int iC = FindDelaunayPoint(poly, iA, iB);
        int a = poly[iA], b = poly[iB], c = poly[iC];

        // Second part (after c)
        if (iB - iC > 1)
        {
            int iNext = trianglesToReuse[^1]; trianglesToReuse.RemoveAt(trianglesToReuse.Count - 1);
            iterations.Add(new TriangulatePseudoPolygonTask(iC, iB, iNext, iT, 1));
        }
        else
        {
            var outerEdge = new Edge(b, c);
            int outerTri = outerTris[outerEdge];
            ref var tri = ref _triangles[iT];
            tri.N1 = Indices.NoNeighbor;
            if (outerTri != Indices.NoNeighbor)
            {
                tri.N1 = outerTri;
                ChangeNeighbor(outerTri, c, b, iT);
            }
            else outerTris[outerEdge] = iT;
        }

        // First part (before c)
        if (iC - iA > 1)
        {
            int iNext = trianglesToReuse[^1]; trianglesToReuse.RemoveAt(trianglesToReuse.Count - 1);
            iterations.Add(new TriangulatePseudoPolygonTask(iA, iC, iNext, iT, 2));
        }
        else
        {
            var outerEdge = new Edge(c, a);
            int outerTri = outerTris[outerEdge];
            ref var tri = ref _triangles[iT];
            tri.N2 = Indices.NoNeighbor;
            if (outerTri != Indices.NoNeighbor)
            {
                tri.N2 = outerTri;
                ChangeNeighbor(outerTri, c, a, iT);
            }
            else outerTris[outerEdge] = iT;
        }

        // Finalize triangle
        ref var parentTri = ref _triangles[iParent]; parentTri.SetNeighbor(iInParent, iT);
        ref var tFinal = ref _triangles[iT]; tFinal.N0 = iParent; tFinal.V0 = a; tFinal.V1 = b; tFinal.V2 = c;
        SetAdjacentTriangle(c, iT);
    }

    private int FindDelaunayPoint(List<int> poly, int iA, int iB)
    {
        var a = _vertices[poly[iA]];
        var b = _vertices[poly[iB]];
        int best = iA + 1;
        var bestV = _vertices[poly[best]];
        for (int i = iA + 1; i < iB; i++)
        {
            var v = _vertices[poly[i]];
            if (IsInCircumcircle(v, a, b, bestV))
            {
                best = i;
                bestV = v;
            }
        }
        return best;
    }

    // -------------------------------------------------------------------------
    // Triangle search helpers
    // -------------------------------------------------------------------------

    private (int iT, int iVleft, int iVright) IntersectedTriangle(
        int iA, V2d<T> a, V2d<T> b, T tolerance)
    {
        int startTri = _vertTris[iA];
        int iT = startTri;
        do
        {
            var t = _triangles[iT];
            int i = CdtUtils.VertexIndex(t, iA);
            int iP2 = t.GetVertex(CdtUtils.Ccw(i));
            var p2 = _vertices[iP2];
            T orientP2 = Orient2D(p2, a, b);
            var locP2 = CdtUtils.ClassifyOrientation(orientP2, tolerance);
            if (locP2 == PtLineLocation.Right)
            {
                int iP1 = t.GetVertex(CdtUtils.Cw(i));
                var p1 = _vertices[iP1];
                T orientP1 = Orient2D(p1, a, b);
                var locP1 = CdtUtils.ClassifyOrientation(orientP1, T.Zero);
                if (locP1 == PtLineLocation.OnLine)
                    return (Indices.NoNeighbor, iP1, iP1);
                if (locP1 == PtLineLocation.Left)
                {
                    if (tolerance != T.Zero)
                    {
                        T absp1 = T.Abs(orientP1), absp2 = T.Abs(orientP2);
                        T closestOrient; int iClosest;
                        if (absp1 <= absp2) { closestOrient = orientP1; iClosest = iP1; }
                        else { closestOrient = orientP2; iClosest = iP2; }
                        if (CdtUtils.ClassifyOrientation(closestOrient, tolerance) == PtLineLocation.OnLine)
                            return (Indices.NoNeighbor, iClosest, iClosest);
                    }
                    return (iT, iP1, iP2);
                }
            }
            (iT, _) = t.Next(iA);
        } while (iT != startTri);

        throw new TriangulationException("Could not find vertex triangle intersected by an edge.");
    }

    // -------------------------------------------------------------------------
    // Topology changes
    // -------------------------------------------------------------------------

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void SetAdjacentTriangle(int v, int iT)
    {
        _vertTris[v] = iT;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void ChangeNeighbor(int iT, int oldN, int newN)
    {
        if (iT == Indices.NoNeighbor) return;
        ref var t = ref _triangles[iT];
        if (t.N0 == oldN) t.N0 = newN;
        else if (t.N1 == oldN) t.N1 = newN;
        else t.N2 = newN;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void ChangeNeighbor(int iT, int va, int vb, int newN)
    {
        if (iT == Indices.NoNeighbor) return;
        ref var t = ref _triangles[iT];
        t.SetNeighbor(CdtUtils.EdgeNeighborIndex(t, va, vb), newN);
    }

    private void PivotVertexTriangleCW(int v)
    {
        int iT = _vertTris[v];
        var (iNext, _) = _triangles[iT].Next(v);
        _vertTris[v] = iNext;
    }

    // -------------------------------------------------------------------------
    // Fixed edge management
    // -------------------------------------------------------------------------

    private void FixEdge(Edge edge)
    {
        if (!_fixedEdges.Add(edge))
        {
            _overlapCount.TryGetValue(edge, out ushort cur);
            _overlapCount[edge] = (ushort)(cur + 1);
        }
    }

    private void FixEdge(Edge edge, Edge originalEdge)
    {
        FixEdge(edge);
        if (edge != originalEdge)
            InsertUnique(_pieceToOriginals.GetOrAdd(edge), originalEdge);
    }

    private void SplitFixedEdge(Edge edge, int iSplitVert)
    {
        var half1 = new Edge(edge.V1, iSplitVert);
        var half2 = new Edge(iSplitVert, edge.V2);
        _fixedEdges.Remove(edge);
        FixEdge(half1);
        FixEdge(half2);
        if (_overlapCount.TryGetValue(edge, out ushort ov))
        {
            _overlapCount.TryGetValue(half1, out ushort h1); _overlapCount[half1] = (ushort)(h1 + ov);
            _overlapCount.TryGetValue(half2, out ushort h2); _overlapCount[half2] = (ushort)(h2 + ov);
            _overlapCount.Remove(edge);
        }
        var newOrig = new List<Edge> { edge };
        if (_pieceToOriginals.TryGetValue(edge, out var originals))
        {
            newOrig = originals;
            _pieceToOriginals.Remove(edge);
        }
        InsertUnique(_pieceToOriginals.GetOrAdd(half1), newOrig);
        InsertUnique(_pieceToOriginals.GetOrAdd(half2), newOrig);
    }

    private int SplitFixedEdgeAt(Edge edge, V2d<T> splitVert, int iT, int iTopo)
    {
        int iSplit = _verticesCount;
        AddNewVertex(splitVert, Indices.NoNeighbor);
        var stack = new Stack<int>(4);
        InsertVertexOnEdge(iSplit, iT, iTopo, handleFixedSplitEdge: false, stack);
        TryAddVertexToLocator(iSplit);
        EnsureDelaunayByEdgeFlips(iSplit, stack);
        SplitFixedEdge(edge, iSplit);
        return iSplit;
    }

    // -------------------------------------------------------------------------
    // Edge query
    // -------------------------------------------------------------------------

    private bool HasEdge(int va, int vb)
    {
        int startTri = _vertTris[va];
        int iT = startTri;
        do
        {
            var t = _triangles[iT];
            if (t.ContainsVertex(vb)) return true;
            (iT, _) = t.Next(va);
        } while (iT != startTri && iT != Indices.NoNeighbor);
        return false;
    }

    // -------------------------------------------------------------------------
    // Finalization
    // -------------------------------------------------------------------------

    private HashSet<int> GrowToBoundary(Stack<int> seeds)
    {
        var traversed = new HashSet<int>();
        while (seeds.Count > 0)
        {
            int iT = seeds.Pop();
            traversed.Add(iT);
            var t = _triangles[iT];
            for (int i = 0; i < 3; i++)
            {
                int va = t.GetVertex(CdtUtils.Ccw(i));
                int vb = t.GetVertex(CdtUtils.Cw(i));
                var opEdge = new Edge(va, vb);
                if (_fixedEdges.Contains(opEdge)) continue;
                int iN = t.GetNeighbor(CdtUtils.OpposedNeighborIndex(i));
                if (iN != Indices.NoNeighbor && !traversed.Contains(iN))
                    seeds.Push(iN);
            }
        }
        return traversed;
    }

    private void FinalizeTriangulation(HashSet<int> removedTriangles)
    {
        _vertTrisCount = 0;

        if (_superGeomType == SuperGeometryType.SuperTriangle)
        {
            int shift = Indices.SuperTriangleVertexCount;
            _vertices.AsSpan(shift, _verticesCount - shift).CopyTo(_vertices);
            _verticesCount -= shift;
            RemapEdgesNoSuperTriangle(_fixedEdges);
            RemapEdgesNoSuperTriangle(_overlapCount);
            RemapEdgesNoSuperTriangle(_pieceToOriginals);
        }

        RemoveTriangles(removedTriangles);

        if (_superGeomType == SuperGeometryType.SuperTriangle)
        {
            int offset = Indices.SuperTriangleVertexCount;
            for (int i = 0; i < _trianglesCount; i++)
            {
                ref var t = ref _triangles[i];
                t.V0 -= offset; t.V1 -= offset; t.V2 -= offset;
            }
        }
    }

    private static Edge ShiftEdge(Edge e, int offset) => new(e.V1 - offset, e.V2 - offset);

    private void RemapEdgesNoSuperTriangle(HashSet<Edge> edges)
    {
        var updated = new HashSet<Edge>(edges.Count);
        foreach (var e in edges) updated.Add(ShiftEdge(e, Indices.SuperTriangleVertexCount));
        edges.Clear();
        foreach (var e in updated) edges.Add(e);
    }

    private void RemapEdgesNoSuperTriangle<TVal>(Dictionary<Edge, TVal> dict)
    {
        var updated = new Dictionary<Edge, TVal>(dict.Count);
        foreach (var kv in dict) updated[ShiftEdge(kv.Key, Indices.SuperTriangleVertexCount)] = kv.Value;
        dict.Clear();
        foreach (var kv in updated) dict[kv.Key] = kv.Value;
    }

    private void RemapEdgesNoSuperTriangle(Dictionary<Edge, List<Edge>> dict)
    {
        var updated = new Dictionary<Edge, List<Edge>>(dict.Count);
        foreach (var kv in dict)
        {
            var newKey = ShiftEdge(kv.Key, Indices.SuperTriangleVertexCount);
            var newList = new List<Edge>(kv.Value.Count);
            foreach (var e in kv.Value) newList.Add(ShiftEdge(e, Indices.SuperTriangleVertexCount));
            updated[newKey] = newList;
        }
        dict.Clear();
        foreach (var kv in updated) dict[kv.Key] = kv.Value;
    }

    private void RemoveTriangles(HashSet<int> removed)
    {
        if (removed.Count == 0) return;
        // Build a flat bool[] for O(1) indexed lookup — replaces all HashSet.Contains calls
        var isRemoved = new bool[_trianglesCount];
        foreach (int i in removed) isRemoved[i] = true;
        // Build compact mapping: old index → new index
        var mapping = new int[_trianglesCount];
        int newIdx = 0;
        for (int i = 0; i < _trianglesCount; i++)
        {
            if (isRemoved[i]) { mapping[i] = Indices.NoNeighbor; continue; }
            mapping[i] = newIdx++;
        }
        // Compact triangle list
        int write = 0;
        for (int i = 0; i < _trianglesCount; i++)
        {
            if (isRemoved[i]) continue;
            _triangles[write++] = _triangles[i];
        }
        _trianglesCount = write;
        // Re-map neighbor indices
        for (int i = 0; i < _trianglesCount; i++)
        {
            ref var t = ref _triangles[i];
            t.N0 = t.N0 == Indices.NoNeighbor ? Indices.NoNeighbor : (isRemoved[t.N0] ? Indices.NoNeighbor : mapping[t.N0]);
            t.N1 = t.N1 == Indices.NoNeighbor ? Indices.NoNeighbor : (isRemoved[t.N1] ? Indices.NoNeighbor : mapping[t.N1]);
            t.N2 = t.N2 == Indices.NoNeighbor ? Indices.NoNeighbor : (isRemoved[t.N2] ? Indices.NoNeighbor : mapping[t.N2]);
        }
    }

    // -------------------------------------------------------------------------
    // Layer depth / hole detection
    // -------------------------------------------------------------------------

    private ushort[] CalculateTriangleDepths()
    {
        var depths = new ushort[_trianglesCount];
        for (int i = 0; i < depths.Length; i++) depths[i] = ushort.MaxValue;

        // Find a triangle touching the super-triangle vertex 0
        int seedTri = _vertTrisCount > 0 ? _vertTris[0] : Indices.NoNeighbor;
        if (seedTri == Indices.NoNeighbor && _trianglesCount > 0) seedTri = 0;

        var layerSeeds = new Stack<int>();
        layerSeeds.Push(seedTri);
        ushort depth = 0;

        while (layerSeeds.Count > 0)
        {
            var nextLayer = PeelLayer(layerSeeds, depth, depths);
            layerSeeds = new Stack<int>(nextLayer.Keys);
            depth++;
        }
        return depths;
    }

    private Dictionary<int, ushort> PeelLayer(
        Stack<int> seeds, ushort layerDepth, ushort[] triDepths)
    {
        var behindBoundary = new Dictionary<int, ushort>();
        while (seeds.Count > 0)
        {
            int iT = seeds.Pop();
            triDepths[iT] = Math.Min(triDepths[iT], layerDepth);
            behindBoundary.Remove(iT);

            var t = _triangles[iT];
            for (int i = 0; i < 3; i++)
            {
                int va = t.GetVertex(CdtUtils.Ccw(i));
                int vb = t.GetVertex(CdtUtils.Cw(i));
                var opEdge = new Edge(va, vb);
                int iN = t.GetNeighbor(CdtUtils.OpposedNeighborIndex(i));
                if (iN == Indices.NoNeighbor || triDepths[iN] <= layerDepth) continue;

                if (_fixedEdges.Contains(opEdge))
                {
                    ushort nextDepth = layerDepth;
                    if (_overlapCount.TryGetValue(opEdge, out ushort ov))
                        nextDepth += ov;
                    nextDepth++;
                    if (!behindBoundary.ContainsKey(iN) || behindBoundary[iN] > nextDepth)
                        behindBoundary[iN] = nextDepth;
                }
                else
                {
                    seeds.Push(iN);
                }
            }
        }
        return behindBoundary;
    }

    // -------------------------------------------------------------------------
    // KD-tree helpers
    // -------------------------------------------------------------------------

    private void InitKdTree()
    {
        var box = new Box2d<T>();
        box.Envelop(_vertices.AsSpan(0, _verticesCount));
        _kdTree = new KdTree<T>(box.Min.X, box.Min.Y, box.Max.X, box.Max.Y);
        for (int i = 0; i < _verticesCount; i++)
            _kdTree.Insert(i, _vertices);
    }

    private void TryAddVertexToLocator(int iV)
    {
        // Only add to the locator if it's already initialized
        _kdTree?.Insert(iV, _vertices);
    }

    // -------------------------------------------------------------------------
    // Misc helpers
    // -------------------------------------------------------------------------

    private readonly record struct ConformToEdgeTask(Edge Edge, List<Edge> Originals, ushort Overlaps);

    private static void InsertUnique(List<Edge> to, Edge elem)
    {
        if (!to.Contains(elem)) to.Add(elem);
    }

    private static void InsertUnique(List<Edge> to, IEnumerable<Edge> from)
    {
        foreach (var e in from) InsertUnique(to, e);
    }

    private static T ParseT(string s)
        => T.Parse(s, System.Globalization.CultureInfo.InvariantCulture);
}

internal static class DictionaryExtensions
{
    public static TVal GetOrAdd<TKey, TVal>(this Dictionary<TKey, TVal> dict, TKey key)
        where TKey : notnull
        where TVal : new()
    {
        if (!dict.TryGetValue(key, out var val))
        {
            val = new TVal();
            dict[key] = val;
        }
        return val;
    }
}

/// <summary>
/// Provides a covariant read-only view over a <see cref="Dictionary{TKey,TInner}"/>
/// where <typeparamref name="TInner"/> is assignable to <typeparamref name="TOuter"/>.
/// </summary>
internal sealed class CovariantReadOnlyDictionary<TKey, TInner, TOuter>(Dictionary<TKey, TInner> inner)
    : IReadOnlyDictionary<TKey, TOuter>
    where TKey : notnull
    where TInner : TOuter
{
    public TOuter this[TKey key] => inner[key];
    public IEnumerable<TKey> Keys => inner.Keys;
    public IEnumerable<TOuter> Values => inner.Values.Cast<TOuter>();
    public int Count => inner.Count;
    public bool ContainsKey(TKey key) => inner.ContainsKey(key);
    public bool TryGetValue(TKey key, [System.Diagnostics.CodeAnalysis.MaybeNullWhen(false)] out TOuter value)
    {
        if (inner.TryGetValue(key, out var v)) { value = v!; return true; }
        value = default!;
        return false;
    }
    public IEnumerator<KeyValuePair<TKey, TOuter>> GetEnumerator() =>
        inner.Select(kv => new KeyValuePair<TKey, TOuter>(kv.Key, kv.Value)).GetEnumerator();
    System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator() => GetEnumerator();
}