"""Leaflet map arrow marker for drone position display.

Renders and updates a directional arrow on a NiceGUI Leaflet map via the
JavaScript helpers in ``static/arrows.js``.
"""

from nicegui import ui


class Arrow:
    """Arrow marker for drone position display on map."""

    def __init__(self, map_ui, id: str, lat: float, lng: float, heading: float,
                 drones_arrows: dict, color: str = '#FF6B6B'):
        """Initialize an arrow on the given map.

        :param map_ui: The NiceGUI Leaflet map instance.
        :param id: Unique identifier for the arrow (namespace).
        :param lat: Latitude of the arrow's initial position.
        :param lng: Longitude of the arrow's initial position.
        :param heading: Initial heading of the arrow (in degrees).
        :param drones_arrows: Dict to check for duplicate arrows.
        :param color: Primary color for the arrow (hex format).
        """
        self.map_ui = map_ui
        self.id = id
        self.lat = lat
        self.lng = lng
        self.heading = heading
        self.color = color
        # Generate darker shade for 3D effect
        self.dark_color = self._darken_color(color)

        if id in drones_arrows:
            raise ValueError(f"Arrow with id '{id}' already exists.")

    def _darken_color(self, hex_color: str) -> str:
        """Generate a darker shade of the given hex color."""
        # Remove # if present
        hex_color = hex_color.lstrip('#')
        # Convert to RGB
        r = int(hex_color[0:2], 16)
        g = int(hex_color[2:4], 16)
        b = int(hex_color[4:6], 16)
        # Darken by 30%
        factor = 0.7
        r = int(r * factor)
        g = int(g * factor)
        b = int(b * factor)
        return f'#{r:02x}{g:02x}{b:02x}'

    def _place_arrow(self):
        """Place the arrow on the map."""
        ui.run_javascript(
            f"place_arrow({self.map_ui.id}, {self.lat}, {self.lng}, {self.heading}, '{self.id}', '{self.color}', '{self.dark_color}')"
        )

    def update(self, lat: float, lng: float, heading: float):
        """Update the position and heading of the arrow.

        :param lat: New latitude.
        :param lng: New longitude.
        :param heading: New heading (in degrees).
        """
        self.lat = lat
        self.lng = lng
        self.heading = heading
        ui.run_javascript(
            f"update_arrow_test('{self.id}', {self.lat}, {self.lng}, {self.heading})"
        )

    def destroy(self):
        """Remove the arrow from the map."""
        ui.run_javascript(f"delete_arrow('{self.id}')")
