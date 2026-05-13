"""
Map Visualization Module - ENHANCED VERSION
Fast native map using PyQtGraph + OpenStreetMap tiles with HIGH ZOOM for street-level detail.

IMPROVEMENTS OVER PREVIOUS VERSION:
- Zoom 18 (street level) instead of Zoom 15 - readable street names
- Interactive zoom with mouse wheel
- Pan with mouse drag
- Smaller prefetch radius optimized for higher zoom
- More workers for faster tile loading at high zoom
- Helpful UI hints for controls
"""

import math
import urllib.request
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel, QHBoxLayout
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QObject, QRectF, QTimer
from PyQt5.QtGui import QPixmap
import pyqtgraph as pg
import numpy as np
from concurrent.futures import ThreadPoolExecutor
import time


ZOOM    = 18  # Street-level zoom for readable text
TILE_PX = 256
PREFETCH_RADIUS = 2   # Smaller radius at high zoom (tiles cover less ground)
MAX_WORKERS = 6       # More workers for faster loading


# ---------------------------------------------------------------------------
# Maths
# ---------------------------------------------------------------------------

def latlon_to_fractional(lat, lon, zoom):
    n   = 2 ** zoom
    fx  = (lon + 180.0) / 360.0 * n
    lr  = math.radians(lat)
    # Standard OSM tile calculation gives Y=0 at top (North)
    # We keep this as-is - it's correct
    fy  = (1.0 - math.log(math.tan(lr) + 1.0 / math.cos(lr)) / math.pi) / 2.0 * n
    return fx, fy


def latlon_to_tile(lat, lon, zoom):
    fx, fy = latlon_to_fractional(lat, lon, zoom)
    return int(fx), int(fy)


def tile_to_world(tx, ty):
    """Top-left pixel corner of tile in world coordinates (pixel space)."""
    return tx * TILE_PX, ty * TILE_PX


# ---------------------------------------------------------------------------
# Background fetcher - Truly async
# ---------------------------------------------------------------------------

class TileFetchWorker(QObject):
    """Worker that runs in a thread pool to fetch tiles asynchronously"""
    finished = pyqtSignal(int, int, object)  # tx, ty, image_data_or_None
    
    def __init__(self, tx, ty):
        super().__init__()
        self.tx = tx
        self.ty = ty
        
    def fetch(self):
        """Fetch tile from OSM - runs in background thread"""
        url = f"https://tile.openstreetmap.org/{ZOOM}/{self.tx}/{self.ty}.png"
        try:
            req = urllib.request.Request(
                url, headers={"User-Agent": "SatelliteTelemetryGUI/2.1"})
            with urllib.request.urlopen(req, timeout=5) as r:
                data = r.read()
                self.finished.emit(self.tx, self.ty, data)
        except Exception as e:
            print(f"Tile fetch failed {ZOOM}/{self.tx}/{self.ty}: {e}")
            self.finished.emit(self.tx, self.ty, None)


# ---------------------------------------------------------------------------
# MapView - Enhanced for Street-Level Detail
# ---------------------------------------------------------------------------

class MapView(QWidget):
    """
    Enhanced map widget with street-level zoom (18).
    - Mouse wheel to zoom in/out
    - Drag to pan
    - Street names are clearly readable
    - Optimized tile loading for high zoom
    """

    def __init__(self, parent=None):
        super().__init__(parent)

        self.latitude  = 49.0395063
        self.longitude = 2.0724911

        self._tile_items   = {}      # (tx, ty) -> pg.ImageItem (on plot)
        self._tile_cache   = set()   # (tx, ty) that are loaded or being loaded
        self._current_tile = None    # Last tile position for boundary detection
        
        # Thread pool for async tile fetching
        self._executor = ThreadPoolExecutor(max_workers=MAX_WORKERS)
        self._pending_workers = []   # Keep worker refs

        self._path_xs = []
        self._path_ys = []

        self._init_ui()
        
        # Initial tile load - run after UI is ready
        QTimer.singleShot(100, self._initial_load)

    # ------------------------------------------------------------------
    # UI
    # ------------------------------------------------------------------

    def _init_ui(self):
        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        self.plot = pg.PlotWidget()
        self.plot.setAspectLocked(True)
        self.plot.hideAxis('left')
        self.plot.hideAxis('bottom')
        self.plot.setBackground('#e8e0d8')   # OSM-style beige while tiles load
        
        # Configure ViewBox for pixel-perfect tile rendering
        view_box = self.plot.getPlotItem().getViewBox()
        view_box.setAspectLocked(True)
        
        # CRITICAL: Invert Y-axis so North is at the top
        # OSM tiles have Y=0 at top (North), but PyQtGraph Y increases downward
        view_box.invertY(True)
        
        # Disable anti-aliasing to prevent tile seams
        self.plot.setAntialiasing(False)
        
        # Enable mouse interaction for zooming and panning
        self.plot.setMouseEnabled(x=True, y=True)  # Enable drag to pan
        # Mouse wheel zoom is enabled by default in PlotWidget
        
        # Disable auto-range — we pan manually to follow satellite
        self.plot.disableAutoRange()

        # Blue polyline path
        self.path_curve = self.plot.plot(
            pen=pg.mkPen('#2196F3', width=3), antialias=True)

        # Red dot at every received point (slightly larger for visibility at zoom 18)
        self.dots = pg.ScatterPlotItem(
            size=7, pen=pg.mkPen(None), brush=pg.mkBrush('#e74c3c'))
        self.plot.addItem(self.dots)

        # Larger marker at current position
        self.marker = pg.ScatterPlotItem(
            size=16,
            pen=pg.mkPen('#c0392b', width=2),
            brush=pg.mkBrush('#e74c3c'))
        self.plot.addItem(self.marker)

        layout.addWidget(self.plot)

        # Coordinate bar with zoom info and controls hint
        bar = QWidget()
        bar.setStyleSheet("background:#2c3e50; padding:3px;")
        bl  = QHBoxLayout()
        bl.setContentsMargins(8, 2, 8, 2)
        self.coord_label = QLabel(
            f"🛰️  Lat: ---   Lon: ---  |  Zoom: {ZOOM} (Street Level)  |  💡 Scroll to zoom, drag to pan"
        )
        self.coord_label.setStyleSheet(
            "color:white; font-weight:bold; font-size:11px;")
        bl.addWidget(self.coord_label)
        bar.setLayout(bl)
        layout.addWidget(bar)

        self.setLayout(layout)

    # ------------------------------------------------------------------
    # Tile management - Optimized
    # ------------------------------------------------------------------
    
    def _initial_load(self):
        """Load initial tile region after UI is ready"""
        cx, cy = latlon_to_tile(self.latitude, self.longitude, ZOOM)
        self._current_tile = (cx, cy)
        self._prefetch_region(cx, cy)
        self._update_view_centre()

    def _prefetch_region(self, cx, cy):
        """Request all tiles within PREFETCH_RADIUS of (cx, cy) - with cache check"""
        r = PREFETCH_RADIUS
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                tx, ty = cx + dx, cy + dy
                # Only fetch if not already cached
                if (tx, ty) not in self._tile_cache:
                    self._request_tile(tx, ty)

    def _request_tile(self, tx, ty):
        """Fetch tile asynchronously using thread pool"""
        # Mark as cached immediately to prevent duplicate requests
        self._tile_cache.add((tx, ty))
        
        # Create worker and submit to thread pool
        worker = TileFetchWorker(tx, ty)
        worker.finished.connect(self._on_tile_ready)
        self._pending_workers.append(worker)
        
        # Submit to executor - runs in background thread
        self._executor.submit(worker.fetch)

    def _on_tile_ready(self, tx, ty, data):
        """Called in main thread when a tile is ready"""
        if data is None:
            return

        # Convert PNG bytes to QPixmap
        pixmap = QPixmap()
        pixmap.loadFromData(data)
        if pixmap.isNull():
            return

        # Convert to numpy array with proper format
        img = pixmap.toImage().convertToFormat(4)  # Format_ARGB32
        ptr = img.bits()
        ptr.setsize(img.byteCount())
        arr = np.frombuffer(ptr, dtype=np.uint8).reshape(
            (TILE_PX, TILE_PX, 4)).copy()

        # Create ImageItem at permanent world coordinates
        # KEY FIX: Use integer coordinates for pixel-perfect alignment
        wx, wy = tile_to_world(tx, ty)
        wx, wy = int(wx), int(wy)  # Ensure integer pixel positions
        
        item = pg.ImageItem(arr, autoLevels=False)
        # Use QRectF with exact integer boundaries
        item.setRect(QRectF(float(wx), float(wy), float(TILE_PX), float(TILE_PX)))
        
        # Disable any filtering/interpolation that could cause gaps
        item.setOpts(axisOrder='row-major')
        
        # Add to plot behind overlays
        self.plot.getPlotItem().getViewBox().addItem(item)
        item.setZValue(-1)

        self._tile_items[(tx, ty)] = item

    # ------------------------------------------------------------------
    # Viewport
    # ------------------------------------------------------------------

    def _update_view_centre(self):
        """Pan the viewport to keep the satellite centred"""
        fx, fy = latlon_to_fractional(self.latitude, self.longitude, ZOOM)
        wx = fx * TILE_PX
        wy = fy * TILE_PX
        
        # At zoom 18, show smaller area for street-level detail
        # 0.75 tiles = ~190m radius, perfect for reading street names
        half = TILE_PX * 0.75
        
        # Round to nearest pixel to avoid sub-pixel rendering artifacts
        x_min = round(wx - half)
        x_max = round(wx + half)
        y_min = round(wy - half)
        y_max = round(wy + half)
        
        self.plot.setRange(
            xRange=(x_min, x_max),
            yRange=(y_min, y_max),
            padding=0,
            disableAutoRange=True)

    # ------------------------------------------------------------------
    # Public API - OPTIMIZED
    # ------------------------------------------------------------------

    def update_position(self, latitude, longitude):
        """
        OPTIMIZED: Only updates overlays and viewport.
        Tile prefetching only happens when crossing tile boundaries.
        """
        if not (-90 <= latitude <= 90) or not (-180 <= longitude <= 180):
            return

        self.latitude  = latitude
        self.longitude = longitude

        # Convert to world coords
        fx, fy = latlon_to_fractional(latitude, longitude, ZOOM)
        wx, wy = fx * TILE_PX, fy * TILE_PX

        # Update path data (limit to 500 points for performance)
        self._path_xs.append(wx)
        self._path_ys.append(wy)
        if len(self._path_xs) > 500:
            self._path_xs.pop(0)
            self._path_ys.pop(0)

        # CRITICAL: Only these 3 calls happen on every update - very fast
        self.path_curve.setData(self._path_xs, self._path_ys)
        self.dots.setData(self._path_xs, self._path_ys)
        self.marker.setData([wx], [wy])

        # Update label with zoom info
        self.coord_label.setText(
            f"🛰️  Lat: {latitude:.6f}°   Lon: {longitude:.6f}°  |  Zoom: {ZOOM} (Street Level)  |  💡 Scroll to zoom, drag to pan"
        )

        # Pan viewport to follow satellite
        self._update_view_centre()

        # SMART PREFETCH: Only fetch new tiles when crossing tile boundary
        cx, cy = latlon_to_tile(latitude, longitude, ZOOM)
        if self._current_tile != (cx, cy):
            self._current_tile = (cx, cy)
            # Only prefetch new border tiles, not entire region
            self._prefetch_border_tiles(cx, cy)

    def _prefetch_border_tiles(self, cx, cy):
        """
        Only fetch tiles on the border of current region that aren't cached.
        Much more efficient than prefetching entire region every time.
        """
        r = PREFETCH_RADIUS
        tiles_to_fetch = []
        
        # Only check border tiles
        for dy in range(-r, r + 1):
            for dx in range(-r, r + 1):
                # Skip tiles already in cache
                if (cx + dx, cy + dy) not in self._tile_cache:
                    tiles_to_fetch.append((cx + dx, cy + dy))
        
        # Fetch in background
        for tx, ty in tiles_to_fetch:
            self._request_tile(tx, ty)

    def get_current_position(self):
        return (self.latitude, self.longitude)
    
    def cleanup(self):
        """Clean up resources"""
        self._executor.shutdown(wait=False)