import unittest
import xml.etree.ElementTree as ET
from install_town05_bus_speeds import patch


class SpeedTagsTest(unittest.TestCase):
    def test_only_road_speed_tags_change_and_repeated_install_is_noop(self):
        source = '<osm><node id="1"/><way id="2"/>'
        for i, road in enumerate(('36', '37', '0', '13')):
            source += (f'<relation id="{i}"><member type="way" ref="2" role="left"/>'
                       '<tag k="type" v="lanelet"/><tag k="subtype" v="road"/>'
                       f'<tag k="source:opendrive:road_id" v="{road}"/>'
                       '<tag k="speed_limit" v="20"/></relation>')
        source += '<relation id="9"><tag k="type" v="regulatory_element"/></relation></osm>'
        updated, counts = patch(source)
        self.assertEqual(counts, {'urban_30': 2, 'ring_50': 2})
        self.assertEqual(patch(updated)[0], updated)
        old, new = ET.fromstring(source), ET.fromstring(updated)
        for a, b in zip(old.findall('relation'), new.findall('relation')):
            if a.get('id') == '9':
                self.assertEqual(ET.tostring(a), ET.tostring(b))
                continue
            speed = b.find("tag[@k='speed_limit']")
            self.assertEqual(speed.get('v'), '50' if int(b.get('id')) < 2 else '30')
            a.remove(a.find("tag[@k='speed_limit']"));b.remove(speed)
            # ElementTree whitespace differs, but every geometry member and remaining tag stays.
            self.assertEqual([dict(e.attrib) for e in a], [dict(e.attrib) for e in b])


if __name__ == '__main__':
    unittest.main()
