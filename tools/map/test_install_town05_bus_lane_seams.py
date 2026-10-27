"""Integration checks against the installed native map (CARLA Python required)."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from install_town05_bus_lane_seams import candidate, BOUNDARIES

MAP = Path('/home/a/autoware_data/maps/Town05Bus/lanelet2_map.osm')
XODR = Path('/home/a/CARLA/0.9.16/CarlaUE4/Content/Carla/Maps/OpenDrive/Town05_Opt.xodr')


@unittest.skipUnless(MAP.exists() and XODR.exists(), 'Native Town05 fixture required')
class NativeSeamTest(unittest.TestCase):
    def setUp(self):
        root = ET.parse(MAP).getroot()
        for way_id in BOUNDARIES:
            way = root.find(f"way[@id='{way_id}']")
            for tag in way.findall('tag'):
                way.remove(tag)
        self.text = ET.tostring(root, encoding='unicode')
        self.source = XODR.read_text()

    def test_only_verified_way_tags_change_and_install_is_idempotent(self):
        updated, proof = candidate(self.text, self.source)
        self.assertEqual({p['way_id'] for p in proof}, set(BOUNDARIES))
        for row in proof:
            self.assertGreater(row['samples'], 60)
            self.assertLess(row['max_boundary_error_m'], .05)
        root = ET.fromstring(updated)
        for way_id in BOUNDARIES:
            way = root.find(f"way[@id='{way_id}']")
            self.assertEqual(way.find("tag[@k='lane_change']").get('v'), 'yes')
            for tag in way.findall('tag'):
                way.remove(tag)
        self.assertEqual(ET.tostring(root, encoding='unicode'), self.text)
        self.assertEqual(candidate(updated, self.source)[0], updated)

    def test_solid_or_prohibited_native_marking_is_rejected(self):
        for road in (25, 8):
            for attribute, value in [('type','solid'), ('laneChange','none')]:
                root = ET.fromstring(self.source)
                root.find(f"road[@id='{road}']/lanes/laneSection/left/lane[@id='1']/roadMark").set(attribute,value)
                with self.assertRaisesRegex(ValueError, 'explicitly allow'):
                    candidate(self.text, ET.tostring(root, encoding='unicode'))

    def test_existing_user_tags_are_preserved(self):
        root = ET.fromstring(self.text)
        ET.SubElement(root.find("way[@id='6522']"),'tag',k='lane_change',v='no')
        with self.assertRaisesRegex(ValueError, 'Preserve existing'):
            candidate(ET.tostring(root, encoding='unicode'), self.source)

    def test_extended_unpainted_seam_is_rejected(self):
        root = ET.fromstring(self.source)
        marks = root.findall("road[@id='8']/lanes/laneSection/left/lane[@id='1']/roadMark")
        marks[1].set('sOffset', '1.0')
        with self.assertRaisesRegex(ValueError, 'explicitly allow'):
            candidate(self.text, ET.tostring(root, encoding='unicode'))


if __name__=='__main__':unittest.main()
