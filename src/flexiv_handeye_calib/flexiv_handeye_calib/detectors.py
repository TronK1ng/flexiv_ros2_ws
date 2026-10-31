"""Factory so the nodes do not care which target is on the table."""

from __future__ import annotations

from .chessboard import ChessboardDetector
from .tag_detector import TagDetector

CHESSBOARD_ALIASES = {"chessboard", "checkerboard", "chess", "board", "checker"}
APRILTAG_ALIASES = {"apriltag", "aruco", "tag", "apriltags"}


def make_detector(
    target_type: str = "chessboard",
    *,
    tag_family: str = "25h9",
    tag_size: float = 0.026,
    board_cols: int = 8,
    board_rows: int = 11,
    square_size: float = 0.006,
    board_auto: bool = True,
):
    kind = (target_type or "apriltag").strip().lower()
    if kind in CHESSBOARD_ALIASES:
        return ChessboardDetector(
            cols=board_cols,
            rows=board_rows,
            square_size=square_size,
            auto_pattern=board_auto,
        )
    if kind in APRILTAG_ALIASES:
        return TagDetector(tag_family, tag_size)
    raise ValueError(
        f"unknown target_type '{target_type}', expected one of "
        f"{sorted(CHESSBOARD_ALIASES | APRILTAG_ALIASES)}"
    )
