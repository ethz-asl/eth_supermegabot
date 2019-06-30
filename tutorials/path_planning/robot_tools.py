import numpy as np
import polygon_tools as poly

class Robot2D(object):

    def __init__(self, pos=poly.Point(0.0, 0.0), heading=0.0, footprint=poly.PointList([poly.Point(0.0, 0.0)])):
        self.R = np.eye(2)

        self.position = pos
        self.footprint = footprint
        self.heading = heading
        self._set_heading_transformation()

    def _set_heading_transformation(self):
        ct, st = np.cos(self.heading), np.sin(self.heading)
        self.R = np.array([[ct, -st], [st, ct]])

    def set_heading(self, heading):
        self.heading = heading
        self._set_heading_transformation()

    def set_position(self, pos):
        self.position = pos

    def set_footprint(self, footprint):
        self.footprint = footprint

    def get_current_polygon(self):
        out_poly = poly.Polygon([poly.Point(*(np.matmul(self.R, p)+self.position)) for p in self.footprint])
        return out_poly


