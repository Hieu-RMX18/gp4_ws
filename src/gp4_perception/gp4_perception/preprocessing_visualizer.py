"""Pre-processing & segmentation visualizer — inspect every image pipeline stage.

Subscribes to:
    /camera/color/image_raw  (Image)

Publishes:
    /perception/preprocessing_debug  (Image — current view)

UI:  A single OpenCV window with a **clickable tab bar** at the top.
     Click any tab to switch to that processing stage (displayed full-size).

Tabs:
    ALL        — All stages (tiled 3×3 grid)
    Original   — Raw camera image
    Blur       — Gaussian Blur (noise reduction)
    Gray       — Grayscale conversion
    CLAHE      — Contrast enhancement
    Resize     — Standardised resolution (320×240)
    Threshold  — Binary Threshold (Otsu)
    Canny      — Canny Edge Detection
    Contours   — Contour Segmentation
    Red Mask   — HSV red-colour isolation (for red_box detection)

Trackbars:  Canny Low / Canny High, H_Lo / H_Hi / S_Min / V_Min (realtime)
Keyboard:   q / ESC to quit

Entry point: ros2 run gp4_perception preprocessing_visualizer
"""

from __future__ import annotations

import logging
import sys

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image

_LOGGER = logging.getLogger(__name__)

_WINDOW_NAME = "GP4 Preprocessing Pipeline"

# ---------------------------------------------------------------------------
#  Tab definitions  (order matters — left to right in the tab bar)
# ---------------------------------------------------------------------------
MODE_ALL = 0
MODE_ORIGINAL = 1
MODE_BLUR = 2
MODE_GRAY = 3
MODE_CLAHE = 4
MODE_RESIZED = 5
MODE_THRESHOLD = 6
MODE_CANNY = 7
MODE_CONTOURS = 8
MODE_RED_MASK = 9

_TAB_LABELS: list[tuple[int, str]] = [
    (MODE_ALL, "ALL"),
    (MODE_ORIGINAL, "Original"),
    (MODE_BLUR, "Blur"),
    (MODE_GRAY, "Gray"),
    (MODE_CLAHE, "CLAHE"),
    (MODE_RESIZED, "Resize"),
    (MODE_THRESHOLD, "Threshold"),
    (MODE_CANNY, "Canny"),
    (MODE_CONTOURS, "Contours"),
    (MODE_RED_MASK, "Red Mask"),
]

# Tab bar appearance.
_TAB_H = 36          # tab bar height in pixels
_TAB_FONT = cv2.FONT_HERSHEY_SIMPLEX
_TAB_FONT_SCALE = 0.48
_TAB_FONT_THICK = 1

# Colours (BGR).
_TAB_BG = (40, 40, 40)
_TAB_ACTIVE_BG = (200, 120, 0)   # orange-ish highlight for active tab
_TAB_HOVER_BG = (80, 80, 80)
_TAB_TEXT = (220, 220, 220)
_TAB_ACTIVE_TEXT = (255, 255, 255)
_TAB_BORDER = (100, 100, 100)

# Standardised resolution for the resize step.
_STD_W, _STD_H = 320, 240


# ---------------------------------------------------------------------------
#  Helpers
# ---------------------------------------------------------------------------

def _put_label(img: np.ndarray, text: str) -> np.ndarray:
    """Draw a label with dark background at top-left corner."""
    out = img.copy()
    font = cv2.FONT_HERSHEY_SIMPLEX
    scale = 0.50
    thickness = 1
    (tw, th), baseline = cv2.getTextSize(text, font, scale, thickness)
    cv2.rectangle(out, (0, 0), (tw + 12, th + baseline + 12), (0, 0, 0), -1)
    cv2.putText(out, text, (6, th + 6), font, scale, (0, 255, 200), thickness, cv2.LINE_AA)
    return out


def _ensure_bgr(img: np.ndarray) -> np.ndarray:
    """Ensure image is 3-channel BGR for display / tiling."""
    if len(img.shape) == 2:
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img


# ---------------------------------------------------------------------------
#  Tab bar renderer + hit-test
# ---------------------------------------------------------------------------

class _TabBar:
    """Manages a row of clickable tabs drawn onto the output image."""

    def __init__(self) -> None:
        self._rects: list[tuple[int, int, int, int, int]] = []  # (x1, y1, x2, y2, mode)
        self._hover_mode: int | None = None

    def render(self, width: int, active_mode: int) -> np.ndarray:
        """Return a BGR image (height=_TAB_H, width=width) with tabs drawn."""
        bar = np.full((_TAB_H, width, 3), _TAB_BG, dtype=np.uint8)
        self._rects.clear()

        n = len(_TAB_LABELS)
        pad_x = 4            # gap between tabs
        total_pad = pad_x * (n + 1)
        tab_w = (width - total_pad) // n

        x = pad_x
        y_top = 4
        y_bot = _TAB_H - 4

        for mode, label in _TAB_LABELS:
            x2 = x + tab_w

            # Pick fill colour.
            if mode == active_mode:
                fill = _TAB_ACTIVE_BG
                txt_col = _TAB_ACTIVE_TEXT
            elif mode == self._hover_mode:
                fill = _TAB_HOVER_BG
                txt_col = _TAB_TEXT
            else:
                fill = _TAB_BG
                txt_col = _TAB_TEXT

            # Draw rounded-ish tab background.
            cv2.rectangle(bar, (x, y_top), (x2, y_bot), fill, -1)
            cv2.rectangle(bar, (x, y_top), (x2, y_bot), _TAB_BORDER, 1)

            # Underline active tab.
            if mode == active_mode:
                cv2.line(bar, (x, y_bot), (x2, y_bot), (0, 200, 255), 2)

            # Centre the label text.
            (tw, th), _ = cv2.getTextSize(label, _TAB_FONT, _TAB_FONT_SCALE, _TAB_FONT_THICK)
            tx = x + (tab_w - tw) // 2
            ty = y_top + (y_bot - y_top + th) // 2
            cv2.putText(bar, label, (tx, ty), _TAB_FONT, _TAB_FONT_SCALE, txt_col, _TAB_FONT_THICK, cv2.LINE_AA)

            self._rects.append((x, y_top, x2, y_bot, mode))
            x = x2 + pad_x

        return bar

    def hit_test(self, mx: int, my: int) -> int | None:
        """Return the mode whose tab was clicked, or None."""
        for x1, y1, x2, y2, mode in self._rects:
            if x1 <= mx <= x2 and y1 <= my <= y2:
                return mode
        return None

    def set_hover(self, mx: int, my: int) -> None:
        """Update hover highlight (call on mouse move)."""
        self._hover_mode = self.hit_test(mx, my)


# ---------------------------------------------------------------------------
#  ROS 2 Node
# ---------------------------------------------------------------------------

class PreprocessingVisualizer(Node):
    """Visualise image pre-processing and segmentation stages with tab UI."""

    def __init__(self) -> None:
        super().__init__("preprocessing_visualizer")

        self._bridge = CvBridge()
        self._mode: int = MODE_ALL
        self._tab_bar = _TabBar()

        # Canny defaults.
        self._canny_low: int = 50
        self._canny_high: int = 150

        # HSV red-mask defaults (red wraps around H=0/180).
        self._h_lo: int = 10    # upper bound of low-red range (0..H_Lo)
        self._h_hi: int = 170   # lower bound of high-red range (H_Hi..180)
        self._s_min: int = 80   # minimum saturation
        self._v_min: int = 60   # minimum value

        # --- OpenCV window ---
        cv2.namedWindow(_WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(_WINDOW_NAME, 1280, 720)
        cv2.createTrackbar("Canny Low", _WINDOW_NAME, self._canny_low, 255, self._on_canny_low)
        cv2.createTrackbar("Canny High", _WINDOW_NAME, self._canny_high, 255, self._on_canny_high)
        cv2.createTrackbar("H_Lo", _WINDOW_NAME, self._h_lo, 180, self._on_h_lo)
        cv2.createTrackbar("H_Hi", _WINDOW_NAME, self._h_hi, 180, self._on_h_hi)
        cv2.createTrackbar("S_Min", _WINDOW_NAME, self._s_min, 255, self._on_s_min)
        cv2.createTrackbar("V_Min", _WINDOW_NAME, self._v_min, 255, self._on_v_min)
        cv2.setMouseCallback(_WINDOW_NAME, self._on_mouse)

        # Low-latency QoS — same as other perception nodes.
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.create_subscription(Image, "/camera/color/image_raw", self._on_image, qos)

        self._debug_pub = self.create_publisher(
            Image, "/perception/preprocessing_debug", 10
        )

        self.get_logger().info(
            "PreprocessingVisualizer started — click tabs to switch view, q/ESC to quit"
        )

    # ---- mouse callback ----
    def _on_mouse(self, event: int, x: int, y: int, flags: int, param) -> None:
        if event == cv2.EVENT_LBUTTONDOWN:
            hit = self._tab_bar.hit_test(x, y)
            if hit is not None:
                self._mode = hit
                self.get_logger().info(f"Switched to: {dict(_TAB_LABELS).get(hit, '?')}")
        elif event == cv2.EVENT_MOUSEMOVE:
            self._tab_bar.set_hover(x, y)

    # ---- trackbar callbacks ----
    def _on_canny_low(self, val: int) -> None:
        self._canny_low = val

    def _on_canny_high(self, val: int) -> None:
        self._canny_high = val

    def _on_h_lo(self, val: int) -> None:
        self._h_lo = val

    def _on_h_hi(self, val: int) -> None:
        self._h_hi = val

    def _on_s_min(self, val: int) -> None:
        self._s_min = val

    def _on_v_min(self, val: int) -> None:
        self._v_min = val

    # ---- processing pipeline ----
    def _compute_stages(self, bgr: np.ndarray) -> dict[int, np.ndarray]:
        """Run all pre-processing stages and return {mode: image} dict."""
        stages: dict[int, np.ndarray] = {}

        # 1. Original
        stages[MODE_ORIGINAL] = _put_label(bgr, "Original")

        # 2. Gaussian Blur (noise reduction)
        blurred = cv2.GaussianBlur(bgr, (5, 5), 0)
        stages[MODE_BLUR] = _put_label(blurred, "Gaussian Blur (5x5)")

        # 3. Grayscale conversion
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        stages[MODE_GRAY] = _put_label(_ensure_bgr(gray), "Grayscale")

        # 4. CLAHE contrast enhancement (on grayscale)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        clahe_img = clahe.apply(gray)
        stages[MODE_CLAHE] = _put_label(_ensure_bgr(clahe_img), "CLAHE Contrast")

        # 5. Resized (standardised resolution)
        resized = cv2.resize(bgr, (_STD_W, _STD_H), interpolation=cv2.INTER_AREA)
        resized_display = resized.copy()
        cv2.putText(
            resized_display, f"{_STD_W}x{_STD_H}",
            (resized_display.shape[1] - 90, resized_display.shape[0] - 10),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA,
        )
        # Scale back up so it fills the view area like other stages.
        resized_show = cv2.resize(
            resized_display, (bgr.shape[1], bgr.shape[0]),
            interpolation=cv2.INTER_NEAREST,
        )
        stages[MODE_RESIZED] = _put_label(resized_show, f"Resized {_STD_W}x{_STD_H}")

        # 6. Binary Threshold (Otsu on CLAHE-enhanced grayscale)
        _, binary = cv2.threshold(clahe_img, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        stages[MODE_THRESHOLD] = _put_label(_ensure_bgr(binary), "Binary Threshold (Otsu)")

        # 7. Canny Edge Detection (on blurred grayscale)
        blurred_gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        canny = cv2.Canny(blurred_gray, self._canny_low, self._canny_high)
        stages[MODE_CANNY] = _put_label(
            _ensure_bgr(canny), f"Canny ({self._canny_low}/{self._canny_high})"
        )

        # 8. Contour Segmentation — find contours on binary, draw on original
        contour_vis = bgr.copy()
        contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        min_area = 500
        valid_contours = [c for c in contours if cv2.contourArea(c) > min_area]
        cv2.drawContours(contour_vis, valid_contours, -1, (0, 255, 0), 2)
        for cnt in valid_contours:
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(contour_vis, (x, y), (x + w, y + h), (255, 0, 255), 2)
            area = cv2.contourArea(cnt)
            cv2.putText(
                contour_vis, f"A:{area:.0f}",
                (x, y - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1,
            )
        stages[MODE_CONTOURS] = _put_label(
            contour_vis, f"Contours ({len(valid_contours)} objects)"
        )

        # 9. Red Mask — HSV threshold to isolate red pixels (for red_box)
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        # Red hue wraps around 0/180: range1 = [0, H_Lo], range2 = [H_Hi, 180]
        mask_lo = cv2.inRange(
            hsv,
            np.array([0, self._s_min, self._v_min]),
            np.array([self._h_lo, 255, 255]),
        )
        mask_hi = cv2.inRange(
            hsv,
            np.array([self._h_hi, self._s_min, self._v_min]),
            np.array([180, 255, 255]),
        )
        red_mask = cv2.bitwise_or(mask_lo, mask_hi)
        # Clean up with morphological ops.
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        # Build display: red regions highlighted on original.
        red_overlay = bgr.copy()
        red_overlay[red_mask > 0] = (255, 255, 255)  # red → white
        # Count red pixels.
        n_red = int(np.count_nonzero(red_mask))
        total = red_mask.shape[0] * red_mask.shape[1]
        pct = 100.0 * n_red / total if total > 0 else 0.0
        stages[MODE_RED_MASK] = _put_label(
            red_overlay,
            f"Red Mask  H:[0-{self._h_lo}]+[{self._h_hi}-180]  S>={self._s_min}  V>={self._v_min}  ({pct:.1f}%)",
        )

        return stages

    def _build_tiled(self, stages: dict[int, np.ndarray]) -> np.ndarray:
        """Arrange 9 stages in a 3×3 grid."""
        ref = stages[MODE_ORIGINAL]
        tile_h = ref.shape[0] // 3
        tile_w = ref.shape[1] // 3

        tiles: list[np.ndarray] = []
        for mode in range(MODE_ORIGINAL, MODE_RED_MASK + 1):
            img = stages.get(mode, np.zeros((tile_h, tile_w, 3), dtype=np.uint8))
            tiles.append(cv2.resize(img, (tile_w, tile_h), interpolation=cv2.INTER_AREA))

        row1 = np.hstack(tiles[0:3])
        row2 = np.hstack(tiles[3:6])
        row3 = np.hstack(tiles[6:9])
        return np.vstack([row1, row2, row3])

    # ---- ROS callback ----
    def _on_image(self, msg: Image) -> None:
        """Process each incoming camera frame."""
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().warn(f"CvBridge failed: {exc}")
            return

        stages = self._compute_stages(bgr)

        # Select view based on current tab.
        if self._mode == MODE_ALL:
            content = self._build_tiled(stages)
        else:
            content = stages.get(self._mode, stages[MODE_ORIGINAL])

        # Render tab bar at the width of the content image.
        tab_bar_img = self._tab_bar.render(content.shape[1], self._mode)

        # Composite: tab bar on top + content below.
        display = np.vstack([tab_bar_img, content])

        cv2.imshow(_WINDOW_NAME, display)

        # Keyboard: q / ESC to quit.
        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            self.get_logger().info("Quit requested — shutting down")
            rclpy.shutdown()
            return

        # Publish debug image.
        try:
            debug_msg = self._bridge.cv2_to_imgmsg(display, encoding="bgr8")
            debug_msg.header = msg.header
            self._debug_pub.publish(debug_msg)
        except Exception as exc:
            self.get_logger().warn(f"Failed to publish debug image: {exc}")


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node = PreprocessingVisualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
