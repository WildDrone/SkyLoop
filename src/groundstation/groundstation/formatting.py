"""Small pure formatting helpers shared by the GUI.

Extracted to remove copy-pasted inline formatting. All functions are pure and
byte-for-byte reproduce the expressions they replace (see test_formatting.py).
"""


def format_mmss(seconds: float) -> str:
    """Format a duration as ``M:SS`` (minutes not zero-padded)."""
    return f"{int(seconds // 60)}:{int(seconds % 60):02d}"


def format_hms(seconds: float) -> str:
    """Format a duration as ``HH:MM:SS`` (all zero-padded)."""
    return (f"{int(seconds // 3600):02d}:"
            f"{int((seconds % 3600) // 60):02d}:"
            f"{int(seconds % 60):02d}")


def battery_color(level: float) -> str:
    """Hex color for a battery level (green > 50 > orange > 20 > red)."""
    return '#4caf50' if level > 50 else '#ff9800' if level > 20 else '#f44336'


def battery_status_color(level: float) -> str:
    """Named color for a battery level (green > 50 > orange > 20 > red)."""
    return 'green' if level > 50 else 'orange' if level > 20 else 'red'
