"""Run with:  python -m unittest discover -s tests -v"""
import math, os, sys, unittest
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "path-planning-scripts"))
import terrain_mission_planner as tmp


class PlannerTests(unittest.TestCase):
    def test_clearance_is_never_violated(self):
        dem, ac = tmp.SyntheticDEM(), tmp.Aircraft()
        a, b = (29.2183, 79.5130), (29.2450, 79.5400)
        leg = tmp.plan_leg(dem, a, b, ac, clearance=60.0)
        pts = leg.waypoints
        total = tmp.haversine(a, b)
        for i in range(0, 401):
            t = i / 400
            p = tmp.lerp_latlon(a, b, t)
            d = t * total
            # interpolate flown altitude along waypoint chain
            acc, alt = 0.0, pts[-1][1]
            for (p1, a1), (p2, a2) in zip(pts, pts[1:]):
                seg = tmp.haversine(p1, p2)
                if acc <= d <= acc + seg + 1e-6:
                    f = (d - acc) / seg if seg else 0
                    alt = a1 + f * (a2 - a1)
                    break
                acc += seg
            agl = alt + leg.home_elev - dem.elevation(*p)
            self.assertGreater(agl, 20.0, f"clearance too small at {t:.2f}: {agl:.1f} m")

    def test_gradient_limits(self):
        leg = tmp.plan_leg(tmp.SyntheticDEM(), (29.2183, 79.5130), (29.2450, 79.5400), tmp.Aircraft())
        for (p1, a1), (p2, a2) in zip(leg.waypoints, leg.waypoints[1:]):
            ang = math.degrees(math.atan2(a2 - a1, tmp.haversine(p1, p2)))
            self.assertLess(ang, 20.5)
            self.assertGreater(ang, -15.5)

    def test_spray_area_and_lines(self):
        poly = [(29.2200, 79.5100), (29.2200, 79.5108), (29.2206, 79.5108), (29.2206, 79.5100)]
        segs, area = tmp.boustrophedon(poly, swath=4.0)
        self.assertGreater(area, 4000)
        self.assertGreater(len(segs), 10)


if __name__ == "__main__":
    unittest.main()
