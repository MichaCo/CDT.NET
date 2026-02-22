// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at https://mozilla.org/MPL/2.0/.

using System.Globalization;
using System.IO;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using Microsoft.Win32;

namespace CDT.Viz;

/// <summary>Custom element that renders the triangulation via DrawingContext.</summary>
internal sealed class TriangulationVisual : FrameworkElement
{
    // -----------------------------------------------------------------------
    // Rendering data – set by MainWindow, then InvalidateVisual() is called
    // -----------------------------------------------------------------------
    public List<V2d<double>> Vertices { get; set; } = [];
    public List<Triangle> Triangles { get; set; } = [];
    public HashSet<Edge> FixedEdges { get; set; } = [];
    public bool ShowSuperTriangle { get; set; }   // DontFinalize mode
    public bool ShowPoints { get; set; } = true;
    public bool ShowIndices { get; set; }

    // View transform
    public double Scale { get; set; } = 1.0;
    public Vector Translation { get; set; }

    // -----------------------------------------------------------------------
    // Colors (matching the Qt visualizer palette)
    // -----------------------------------------------------------------------
    private static readonly Pen s_triPen = MakePen(Color.FromRgb(150, 150, 150), 1.5);
    private static readonly Pen s_outerPen = MakePen(Color.FromRgb(220, 220, 220), 1.2);
    private static readonly Pen s_fixedPen = MakePen(Color.FromRgb(50, 50, 50), 2.5);
    private static readonly Pen s_pointPen = MakePen(Color.FromRgb(3, 102, 214), 5.0);
    private static readonly Brush s_indexBrush = new SolidColorBrush(Color.FromRgb(150, 0, 150));
    private static readonly Brush s_triIdxBrush = new SolidColorBrush(Color.FromRgb(0, 150, 150));
    private static readonly Typeface s_typeface = new Typeface("Segoe UI");

    static TriangulationVisual()
    {
        s_triPen.Freeze();
        s_outerPen.Freeze();
        s_fixedPen.Freeze();
        s_pointPen.Freeze();
        ((SolidColorBrush)s_indexBrush).Freeze();
        ((SolidColorBrush)s_triIdxBrush).Freeze();
    }

    private static Pen MakePen(Color c, double thickness)
    {
        var p = new Pen(new SolidColorBrush(c), thickness) { StartLineCap = PenLineCap.Round, EndLineCap = PenLineCap.Round };
        p.Brush.Freeze();
        return p;
    }

    // -----------------------------------------------------------------------
    // Scene ↔ screen coordinate helpers
    // -----------------------------------------------------------------------
    public Point SceneToScreen(V2d<double> v)
    {
        double cx = ActualWidth / 2.0;
        double cy = ActualHeight / 2.0;
        return new Point(
            Scale * v.X + cx + Translation.X,
           -Scale * v.Y + cy + Translation.Y);
    }

    public V2d<double> ScreenToScene(Point pt)
    {
        double cx = ActualWidth / 2.0;
        double cy = ActualHeight / 2.0;
        double sx = (pt.X - Translation.X - cx) / Scale;
        double sy = -(pt.Y - Translation.Y - cy) / Scale;
        return new V2d<double>(sx, sy);
    }

    // -----------------------------------------------------------------------
    // Rendering
    // -----------------------------------------------------------------------
    protected override void OnRender(DrawingContext dc)
    {
        dc.DrawRectangle(Brushes.White, null, new Rect(0, 0, ActualWidth, ActualHeight));

        if (Vertices.Count == 0) return;

        const int superCount = 3; // super-triangle uses first 3 vertices

        // --- Draw triangles ---
        foreach (var t in Triangles)
        {
            bool hasSuperVert = t.V0 < superCount || t.V1 < superCount || t.V2 < superCount;
            Pen pen = (ShowSuperTriangle && hasSuperVert) ? s_outerPen : s_triPen;

            // When not finalizing: draw outer (super) triangles in muted colour,
            // skip non-outer ones for the outer pen pass
            if (!ShowSuperTriangle && hasSuperVert) continue;

            var p0 = SceneToScreen(Vertices[t.V0]);
            var p1 = SceneToScreen(Vertices[t.V1]);
            var p2 = SceneToScreen(Vertices[t.V2]);

            var sg = new StreamGeometry();
            using (var ctx = sg.Open())
            {
                ctx.BeginFigure(p0, isFilled: false, isClosed: true);
                ctx.LineTo(p1, isStroked: true, isSmoothJoin: false);
                ctx.LineTo(p2, isStroked: true, isSmoothJoin: false);
            }
            sg.Freeze();
            dc.DrawGeometry(null, pen, sg);
        }

        // --- Draw triangle indices ---
        if (ShowIndices)
        {
            for (int i = 0; i < Triangles.Count; i++)
            {
                var t = Triangles[i];
                if (!ShowSuperTriangle && (t.V0 < superCount || t.V1 < superCount || t.V2 < superCount))
                    continue;
                var c = Centroid(Vertices[t.V0], Vertices[t.V1], Vertices[t.V2]);
                DrawLabel(dc, i.ToString(), c, s_triIdxBrush, 9);
            }
        }

        // --- Draw fixed (constraint) edges ---
        foreach (var e in FixedEdges)
        {
            if (e.V1 >= Vertices.Count || e.V2 >= Vertices.Count) continue;
            dc.DrawLine(s_fixedPen,
                SceneToScreen(Vertices[e.V1]),
                SceneToScreen(Vertices[e.V2]));
        }

        if (!ShowPoints) return;

        // --- Draw vertices ---
        foreach (var v in Vertices)
        {
            dc.DrawEllipse(s_pointPen.Brush, null, SceneToScreen(v), 3, 3);
        }

        // --- Draw vertex indices ---
        if (ShowIndices)
        {
            for (int i = 0; i < Vertices.Count; i++)
                DrawLabel(dc, i.ToString(), Vertices[i], s_indexBrush, 9);
        }
    }

    private Point Centroid(V2d<double> a, V2d<double> b, V2d<double> c)
    {
        var pa = SceneToScreen(a);
        var pb = SceneToScreen(b);
        var pc = SceneToScreen(c);
        return new Point((pa.X + pb.X + pc.X) / 3, (pa.Y + pb.Y + pc.Y) / 3);
    }

    private void DrawLabel(DrawingContext dc, string text, V2d<double> v, Brush brush, double size)
        => DrawLabel(dc, text, SceneToScreen(v), brush, size);

    private void DrawLabel(DrawingContext dc, string text, Point pt, Brush brush, double size)
    {
        var ft = new FormattedText(text, CultureInfo.InvariantCulture,
            FlowDirection.LeftToRight, s_typeface, size, brush,
            VisualTreeHelper.GetDpi(this).PixelsPerDip);
        dc.DrawText(ft, new Point(pt.X + 4, pt.Y - ft.Height / 2));
    }
}

/// <summary>Main visualizer window.</summary>
public partial class MainWindow : Window
{
    // -----------------------------------------------------------------------
    // Fields
    // -----------------------------------------------------------------------
    private List<V2d<double>> _loadedPoints = [];
    private List<Edge> _loadedEdges = [];
    private Triangulation<double>? _cdt;

    private readonly TriangulationVisual _visual = new();

    // Pan/zoom state
    private bool _isPanning;
    private Point _panStart;
    private Vector _panStartTranslation;

    // Option guards to suppress re-triangulation during init
    private bool _initializing = true;

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------
    public MainWindow()
    {
        InitializeComponent();
        Loaded += OnLoaded;
    }

    private void OnLoaded(object sender, RoutedEventArgs e)
    {
        // Add the visual element to fill the canvas border
        TriCanvas.Children.Add(_visual);
        Canvas.SetLeft(_visual, 0);
        Canvas.SetTop(_visual, 0);

        // Set size immediately so FitView works correctly before SizeChanged fires
        _visual.Width = CanvasBorder.ActualWidth;
        _visual.Height = CanvasBorder.ActualHeight;
        CanvasBorder.SizeChanged += (_, _) =>
        {
            _visual.Width = CanvasBorder.ActualWidth;
            _visual.Height = CanvasBorder.ActualHeight;
        };

        // Populate file list from inputs directory
        PopulateFileList();

        _initializing = false;

        // Auto-load the first example
        if (FileList.Items.Count > 0)
            FileList.SelectedIndex = 0;
    }

    private void PopulateFileList()
    {
        // Look for .txt files next to the exe (copied from test inputs)
        var inputsDir = Path.Combine(AppContext.BaseDirectory, "inputs");
        if (!Directory.Exists(inputsDir)) return;

        foreach (var f in Directory.GetFiles(inputsDir, "*.txt").OrderBy(x => x))
            FileList.Items.Add(Path.GetFileName(f));
    }

    // -----------------------------------------------------------------------
    // Toolbar handlers
    // -----------------------------------------------------------------------
    private void LoadFile_Click(object sender, RoutedEventArgs e)
    {
        var dlg = new OpenFileDialog
        {
            Title = "Load CDT input file",
            Filter = "Text files (*.txt)|*.txt|All files (*.*)|*.*",
        };
        if (dlg.ShowDialog() != true) return;
        LoadFromPath(dlg.FileName);
    }

    private void GenerateRandom_Click(object sender, RoutedEventArgs e)
    {
        if (!int.TryParse(RandomCount.Text, out int n) || n <= 0) n = 100;
        var rng = new Random();
        _loadedPoints = Enumerable.Range(0, n)
            .Select(_ => new V2d<double>(rng.NextDouble() * 200 - 100, rng.NextDouble() * 200 - 100))
            .ToList();
        _loadedEdges = [];
        FileList.SelectionChanged -= FileList_SelectionChanged;
        FileList.SelectedIndex = -1;
        FileList.SelectionChanged += FileList_SelectionChanged;
        UpdateLimitSliders();
        Rebuild();
        FitView();
    }

    private void SavePng_Click(object sender, RoutedEventArgs e)
    {
        var dlg = new SaveFileDialog { Filter = "PNG image|*.png", FileName = "cdt_screenshot.png" };
        if (dlg.ShowDialog() != true) return;

        int w = (int)Math.Max(_visual.ActualWidth, 1);
        int h = (int)Math.Max(_visual.ActualHeight, 1);
        var rtb = new RenderTargetBitmap(w, h, 96, 96, PixelFormats.Pbgra32);
        rtb.Render(_visual);
        var enc = new PngBitmapEncoder();
        enc.Frames.Add(BitmapFrame.Create(rtb));
        using var fs = File.OpenWrite(dlg.FileName);
        enc.Save(fs);
        StatusText.Text = $"Saved {dlg.FileName}";
    }

    private void SaveOff_Click(object sender, RoutedEventArgs e)
    {
        if (_cdt is null) return;
        var dlg = new SaveFileDialog { Filter = "OFF file|*.off", FileName = "out.off" };
        if (dlg.ShowDialog() != true) return;

        using var sw = new StreamWriter(dlg.FileName);
        sw.WriteLine("OFF");
        sw.WriteLine($"{_cdt.Vertices.Length} {_cdt.Triangles.Length} 0");
        foreach (var v in _cdt.Vertices.Span)
            sw.WriteLine(FormattableString.Invariant($"{v.X} {v.Y} 0"));
        foreach (var t in _cdt.Triangles.Span)
            sw.WriteLine($"3 {t.V0} {t.V1} {t.V2}");
        StatusText.Text = $"Saved {dlg.FileName}";
    }

    private void ResetView_Click(object sender, RoutedEventArgs e) => FitView();

    // -----------------------------------------------------------------------
    // File list
    // -----------------------------------------------------------------------
    private void FileList_SelectionChanged(object sender, SelectionChangedEventArgs e)
    {
        if (FileList.SelectedItem is not string name) return;
        var path = Path.Combine(AppContext.BaseDirectory, "inputs", name);
        if (!File.Exists(path)) return;
        LoadFromPath(path);
    }

    private void LoadFromPath(string path)
    {
        try
        {
            (_loadedPoints, _loadedEdges) = ReadInput(path);
            UpdateLimitSliders();
            Rebuild();
            FitView();
            StatusText.Text = $"Loaded: {Path.GetFileName(path)}  ({_loadedPoints.Count} pts, {_loadedEdges.Count} edges)";
        }
        catch (Exception ex)
        {
            MessageBox.Show(ex.Message, "Load error", MessageBoxButton.OK, MessageBoxImage.Warning);
        }
    }

    private static (List<V2d<double>> pts, List<Edge> edges) ReadInput(string path)
    {
        using var sr = new StreamReader(path);
        var first = sr.ReadLine()!.Trim().Split(' ', StringSplitOptions.RemoveEmptyEntries);
        int nPts = int.Parse(first[0]);
        int nEdges = int.Parse(first[1]);
        var pts = new List<V2d<double>>(nPts);
        for (int i = 0; i < nPts; i++)
        {
            var tok = sr.ReadLine()!.Trim().Split(' ', StringSplitOptions.RemoveEmptyEntries);
            pts.Add(new V2d<double>(
                double.Parse(tok[0], CultureInfo.InvariantCulture),
                double.Parse(tok[1], CultureInfo.InvariantCulture)));
        }
        var edges = new List<Edge>(nEdges);
        for (int i = 0; i < nEdges; i++)
        {
            var tok = sr.ReadLine()!.Trim().Split(' ', StringSplitOptions.RemoveEmptyEntries);
            edges.Add(new Edge(int.Parse(tok[0]), int.Parse(tok[1])));
        }
        return (pts, edges);
    }

    // -----------------------------------------------------------------------
    // Option change handlers
    // -----------------------------------------------------------------------
    private void Option_Changed(object sender, SelectionChangedEventArgs e)
    {
        if (_initializing) return;
        Rebuild();
    }

    private void Option_ChangedCb(object sender, RoutedEventArgs e)
    {
        if (_initializing) return;
        Rebuild();
    }

    private void MinDist_LostFocus(object sender, RoutedEventArgs e)
    {
        if (_initializing) return;
        Rebuild();
    }

    private void Limit_Changed(object sender, RoutedPropertyChangedEventArgs<double> e)
    {
        if (_initializing) return;
        UpdateLimitLabels();
        Rebuild();
    }

    private void Display_Changed(object sender, RoutedEventArgs e)
    {
        if (_initializing || _visual == null) return;
        _visual.ShowPoints = ShowPoints.IsChecked == true;
        _visual.ShowIndices = ShowIndices.IsChecked == true;
        _visual.InvalidateVisual();
    }

    // -----------------------------------------------------------------------
    // Core rebuild
    // -----------------------------------------------------------------------
    private void Rebuild()
    {
        if (_loadedPoints.Count == 0) return;

        // Vertex / edge limits
        int vLimit = (int)Math.Min(VertexLimit.Value, _loadedPoints.Count);
        int eLimit = (int)Math.Min(EdgeLimit.Value, _loadedEdges.Count);

        var pts = vLimit < _loadedPoints.Count ? _loadedPoints[..vLimit] : _loadedPoints;
        var edges = eLimit < _loadedEdges.Count ? _loadedEdges[..eLimit] : _loadedEdges;

        // Options
        var order = InsertionOrder.SelectedIndex == 1
            ? VertexInsertionOrder.AsProvided
            : VertexInsertionOrder.Auto;
        var strategy = IntersectingEdges.SelectedIndex == 1
            ? IntersectingConstraintEdges.NotAllowed
            : IntersectingConstraintEdges.TryResolve;
        double minDist = double.TryParse(MinDist.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out double md) ? md : 1e-6;
        bool fixDups = FixDuplicates.IsChecked == true;
        bool conforming = TriangulationType.SelectedIndex == 1;

        _cdt = new Triangulation<double>(order, strategy, minDist);

        try
        {
            var workPts = new List<V2d<double>>(pts);

            List<Edge> workEdges;
            if (fixDups)
            {
                workEdges = new List<Edge>(edges);
                CdtUtils.RemoveDuplicatesAndRemapEdges(workPts, workEdges);
            }
            else
            {
                workEdges = edges;
            }

            _cdt.InsertVertices(workPts);

            if (vLimit >= _loadedPoints.Count && workEdges.Count > 0)
            {
                if (conforming)
                    _cdt.ConformToEdges(workEdges);
                else
                    _cdt.InsertEdges(workEdges);
            }
        }
        catch (Exception ex)
        {
            StatusText.Text = $"Error: {ex.Message}";
            _cdt = null;
            return;
        }

        // Finalize
        switch (FinalizeMode.SelectedIndex)
        {
            case 1: _cdt.EraseSuperTriangle(); break;
            case 2: _cdt.EraseOuterTriangles(); break;
            case 3: _cdt.EraseOuterTrianglesAndHoles(); break;
        }

        // Push to visual
        _visual.Vertices = [.. _cdt.Vertices.Span];
        _visual.Triangles = [.. _cdt.Triangles.Span];
        _visual.FixedEdges = new HashSet<Edge>(_cdt.FixedEdges);
        _visual.ShowSuperTriangle = FinalizeMode.SelectedIndex == 0;
        _visual.ShowPoints = ShowPoints.IsChecked == true;
        _visual.ShowIndices = ShowIndices.IsChecked == true;
        _visual.InvalidateVisual();

        StatsLabel.Text = $"Vertices: {_cdt.Vertices.Length}\n" +
                          $"Triangles: {_cdt.Triangles.Length}\n" +
                          $"Fixed edges: {_cdt.FixedEdges.Count}";

        if (!TopologyVerifier.VerifyTopology(_cdt))
            StatusText.Text = "⚠ Topology error detected!";
    }

    // -----------------------------------------------------------------------
    // View fit
    // -----------------------------------------------------------------------
    private void FitView()
    {
        if (_loadedPoints.Count == 0) return;

        double minX = _loadedPoints.Min(v => v.X), maxX = _loadedPoints.Max(v => v.X);
        double minY = _loadedPoints.Min(v => v.Y), maxY = _loadedPoints.Max(v => v.Y);
        double cx = (minX + maxX) / 2, cy = (minY + maxY) / 2;
        double dx = maxX - minX, dy = maxY - minY;
        if (dx == 0) dx = 1;
        if (dy == 0) dy = 1;

        // Use CanvasBorder dimensions directly: the visual always fills it and
        // CanvasBorder.ActualWidth/Height are correct even before the visual's
        // own ActualWidth/Height have been updated by the layout pass.
        double w = Math.Max(CanvasBorder.ActualWidth, 1);
        double h = Math.Max(CanvasBorder.ActualHeight, 1);

        _visual.Scale = Math.Min(w / dx, h / dy) * 0.9;
        _visual.Translation = new Vector(-_visual.Scale * cx, _visual.Scale * cy);
        _visual.InvalidateVisual();
    }

    // -----------------------------------------------------------------------
    // Pan / zoom
    // -----------------------------------------------------------------------
    private void Canvas_MouseWheel(object sender, MouseWheelEventArgs e)
    {
        double factor = e.Delta > 0 ? 1.15 : 1.0 / 1.15;
        var pos = e.GetPosition(_visual);

        // Zoom toward mouse cursor
        double oldScale = _visual.Scale;
        _visual.Scale *= factor;

        double cx = _visual.ActualWidth / 2.0;
        double cy = _visual.ActualHeight / 2.0;
        _visual.Translation = new Vector(
            _visual.Translation.X + (pos.X - cx) * (1 - factor),
            _visual.Translation.Y + (pos.Y - cy) * (1 - factor));

        _visual.InvalidateVisual();
        UpdateCoordLabel(e.GetPosition(_visual));
    }

    private void Canvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
    {
        _isPanning = true;
        _panStart = e.GetPosition(CanvasBorder);
        _panStartTranslation = _visual.Translation;
        CanvasBorder.CaptureMouse();
    }

    private void Canvas_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
    {
        _isPanning = false;
        CanvasBorder.ReleaseMouseCapture();
    }

    private void Canvas_MouseMove(object sender, MouseEventArgs e)
    {
        if (_isPanning)
        {
            var pos = e.GetPosition(CanvasBorder);
            var delta = pos - _panStart;
            _visual.Translation = _panStartTranslation + delta;
            _visual.InvalidateVisual();
        }
        UpdateCoordLabel(e.GetPosition(_visual));
    }

    private void UpdateCoordLabel(Point screenPt)
    {
        var scene = _visual.ScreenToScene(screenPt);
        CoordLabel.Text = FormattableString.Invariant($"x={scene.X:F4}  y={scene.Y:F4}");
    }

    // -----------------------------------------------------------------------
    // Helpers
    // -----------------------------------------------------------------------
    private void UpdateLimitSliders()
    {
        int nPts = Math.Max(_loadedPoints.Count, 1);
        int nEdges = Math.Max(_loadedEdges.Count, 1);
        VertexLimit.Maximum = nPts;
        VertexLimit.Value = nPts;
        EdgeLimit.Maximum = nEdges;
        EdgeLimit.Value = nEdges;
        UpdateLimitLabels();
    }

    private void UpdateLimitLabels()
    {
        int vLim = (int)VertexLimit.Value;
        int eLim = (int)EdgeLimit.Value;
        VertexLimitLabel.Text = vLim >= _loadedPoints.Count ? "All" : vLim.ToString();
        EdgeLimitLabel.Text = eLim >= _loadedEdges.Count ? "All" : eLim.ToString();
    }
}
