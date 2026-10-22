"""Offline checks of stop constraints and the saved loop's continuity."""
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import unittest

import carla

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('bus_route', ROOT/'tools/map/generate_town05_bus_route.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


class BusRouteTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.xodr = Path('/home/a/CARLA/0.9.16/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05.xodr')
        cls.native = carla.Map('Town05', cls.xodr.read_text())
        cls.map_path = Path('/home/a/autoware_data/maps/Town05/lanelet2_map.osm')
        cls.roads, cls.crossings = module.read_polygons(cls.map_path)
        cls.route = json.loads((ROOT/'scenarios/bus/town05_bus_route.json').read_text())

    def test_ten_distinct_stops_in_outer_urban_lanes(self):
        stops = self.route['stops']
        self.assertEqual(len(stops), 10)
        self.assertEqual(len({s['validation']['lanelet_id'] for s in stops}), 10)
        for s in stops:
            self.assertNotIn(s['road_id'], (12,34,35,36,37,38))
            w = self.native.get_waypoint_xodr(s['road_id'], s['lane_id'], s['s_m'])
            module.validate_stop(w, self.roads, self.crossings)

    def test_inner_lane_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'outermost'):
            module.validate_stop(self.native.get_waypoint_xodr(44,-1,30),self.roads,self.crossings)

    def test_junction_is_rejected(self):
        junction = next(w for w in self.native.generate_waypoints(5) if w.is_junction)
        with self.assertRaisesRegex(ValueError, 'non-junction'):
            module.validate_stop(junction,self.roads,self.crossings)

    def test_crosswalk_approach_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'crosswalk|junction'):
            module.validate_stop(self.native.get_waypoint_xodr(8,2,2),self.roads,self.crossings)

    def test_saved_loop_has_no_reverse_samples_or_open_seams(self):
        segments=self.route['segments']
        for i, seg in enumerate(segments):
            self.assertEqual(seg['to'],segments[(i+1)%len(segments)]['from'])
            self.assertEqual(seg['points'][-1]['carla'],segments[(i+1)%len(segments)]['points'][0]['carla'])
            for a,b in zip(seg['points'],seg['points'][1:]):
                x,y=a['carla'],b['carla'];angle=math.radians(x['yaw_deg'])
                along=(y['x']-x['x'])*math.cos(angle)+(y['y']-x['y'])*math.sin(angle)
                self.assertGreaterEqual(along,-.1)
                self.assertGreaterEqual(b['route_distance_m'],a['route_distance_m'])
        total=sum(s['distance_to_next_stop_m'] for s in self.route['stops'])
        self.assertAlmostEqual(total,self.route['length_m'],places=2)

    def test_generated_data_matches_current_map_and_spec(self):
        paths={'spec':ROOT/'config/carla/bus/town05_route_spec.json', 'xodr':self.xodr,
               'lanelet_map':self.map_path}
        for key,path in paths.items():
            self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),self.route['source_sha256'][key])


if __name__ == '__main__':
    unittest.main()
