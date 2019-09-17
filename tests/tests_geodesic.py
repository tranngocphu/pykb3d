"""
In order to validate the computation formula return the correct value, I use
the comparision between the well-known package/libraries and the develop code.

Author: @tanthml

"""
import unittest

from geopy.point import Point
from geopy.distance import great_circle
from geographiclib.geodesic import Geodesic

from pykb3d.Util import deg2rad, rad2deg
from pykb3d.Geodesic import gc_dist, true_course


class TestGeodesic(unittest.TestCase):

    def setUp(self) -> None:
        self.lat1 = 36+59.87/60
        self.lon1 = 76+28.74/60
        self.lat2 = 36+52.5/60
        self.lon2 = 76+20.1/60

    def test_gc_dist(self):
        actual = rad2deg(
            gc_dist(
                lat1=deg2rad(self.lat1), lon1=deg2rad(self.lon1),
                lat2=deg2rad(self.lat2), lon2=deg2rad(self.lon2)
            )
        ) * 60.0  # nautical miles

        expectation = great_circle(
            Point(latitude=self.lat1, longitude=self.lon1),
            Point(latitude=self.lat2, longitude=self.lon2)
        ).nm

        self.assertAlmostEqual(actual, expectation, places=1)

    def test_true_course(self):
        actual = rad2deg(
            true_course(
                lat1=deg2rad(self.lat1), lon1=deg2rad(self.lon1),
                lat2=deg2rad(self.lat2), lon2=deg2rad(self.lon2)
            )
        )  # degree

        '''
        initial azimuth (bearing) is given by azi1 
        (angle value xxx degrees clockwise from north)
        '''
        expectation = Geodesic.WGS84.Inverse(
            self.lat1, self.lon1, self.lat2,self.lon2
        )['azi1']

        # convert to positive value
        actual = (360.0 + actual) if actual < 0 else actual
        expectation = (360.0 + expectation) if expectation < 0 else expectation

        self.assertAlmostEqual(actual, expectation, places=0)


if __name__ == '__main__':
    unittest.main()
