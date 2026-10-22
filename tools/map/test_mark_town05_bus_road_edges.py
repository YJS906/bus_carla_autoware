import unittest
import xml.etree.ElementTree as ET
from mark_town05_bus_road_edges import classify


class RoadEdgeClassification(unittest.TestCase):
    def setUp(self):
        self.roads={8:ET.fromstring('''<road><lanes><laneSection>
          <left><lane id="1" type="driving"/><lane id="2" type="driving"/>
            <lane id="3" type="shoulder"/></left>
          <right><lane id="-1" type="driving"/><lane id="-2" type="driving"/>
            <lane id="-3" type="shoulder"/></right>
          </laneSection></lanes></road>''')}

    def test_outer_edge_both_directions(self):
        for lane in (2,-2):
            self.assertEqual(classify([(8,0,lane)],'right',self.roads),'edge')

    def test_same_direction_lane_is_not_edge(self):
        for lane in (2,-2):
            self.assertEqual(classify([(8,0,lane)],'left',self.roads),'internal')
        for lane in (1,-1):
            self.assertEqual(classify([(8,0,lane)],'right',self.roads),'internal')

    def test_opposite_direction_is_protected(self):
        for lane in (1,-1):
            self.assertEqual(classify([(8,0,lane)],'left',self.roads),'divider')


if __name__=='__main__': unittest.main()
