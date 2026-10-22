#!/usr/bin/env python3
"""Conservatively recover native Town05 legal lane changes, entirely offline.

Only tags on previously unclassified, shared road boundaries are added. We do
not rebuild/split geometry: a boundary spanning disconnected native lanes, variable markings, a junction,
or one-way permission stays non-changeable. Connected road sections are allowed
only when every section has the same explicit two-way lane-change permission. This
is intentionally narrower than the simulator's complete road network.
"""
from __future__ import annotations

import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import carla
from shapely.geometry import LineString

SOURCE = "carla_town05_lane_change_v1"
OWNED_TAGS = {"type": "line_thin", "subtype": "dashed", "lane_change": "yes",
              "carla:lane_marking_source": SOURCE}


def tags(element):
    return {t.get("k"): t.get("v") for t in element.findall("tag")}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    Path(path).write_text(json.dumps(value, indent=2, sort_keys=True) + "\n")


def constant_broken_both(lane):
    """Never infer permission just from a dashed appearance or sampled points."""
    marks = sorted(lane.findall("roadMark"), key=lambda m: float(m.get("sOffset")))
    return bool(marks) and float(marks[0].get("sOffset")) == 0 and all(
        mark.get("type") == "broken" and mark.get("laneChange") == "both"
        for mark in marks)


def same_direction_adjacent(a, b):
    return a[:2] == b[:2] and a[2] * b[2] > 0 and abs(a[2] - b[2]) == 1


def native_marking_lane(section, lane_a, lane_b):
    # An OpenDRIVE lane's roadMark describes its OUTER boundary. The inner of
    # two same-direction lanes therefore owns their shared boundary.
    inner = lane_a if abs(lane_a) < abs(lane_b) else lane_b
    return next((lane for lane in section.findall("./*/lane")
                 if int(lane.get("id")) == inner), None)


def match_lane(left, right, native):
    if math.dist(left[0], right[0]) + math.dist(left[-1], right[-1]) > \
            math.dist(left[0], right[-1]) + math.dist(left[-1], right[0]):
        right = list(reversed(right))
    ll, rr = LineString(left), LineString(right)
    length = max(ll.length, rr.length)
    count = max(4, math.ceil(length / 2))
    keys, max_error = [], 0.0
    for i in range(count + 1):
        # Endpoints coincide with adjoining roads; sample 10 cm inward. Require
        # the ENTIRE matched native laneSection to have a uniform legal mark,
        # so this endpoint inset cannot overlook a short solid segment.
        fraction = min(1 - min(.1 / length, .1), max(min(.1 / length, .1), i / count))
        a, b = ll.interpolate(fraction, normalized=True), rr.interpolate(fraction, normalized=True)
        x, y, z = (a.x+b.x)/2, (a.y+b.y)/2, (a.z+b.z)/2
        wp = native.get_waypoint(carla.Location(x=x, y=-y, z=z),
                                 project_to_road=True, lane_type=carla.LaneType.Driving)
        if wp is None:
            return None, "no_native_lane", max_error
        error = math.hypot(wp.transform.location.x-x, wp.transform.location.y+y)
        max_error = max(max_error, error)
        if error > .65 or abs(wp.transform.location.z-z) > .75:
            return None, "geometry_mismatch", max_error
        # Confirm travel direction, not just coincident geometry (especially
        # relevant to opposite-direction lanelets and junction overlaps).
        lower = max(0.0, fraction-.001)
        upper = min(1.0, fraction+.001)
        p0, p1 = ll.interpolate(lower, normalized=True), ll.interpolate(upper, normalized=True)
        dx, dy = p1.x-p0.x, p1.y-p0.y
        f = wp.transform.get_forward_vector()
        cosine = (dx*f.x-dy*f.y) / max(math.hypot(dx, dy), 1e-12)
        if cosine < .94:
            return None, "direction_mismatch", max_error
        if wp.is_junction:
            return None, "junction", max_error
        key = (wp.road_id, wp.section_id, wp.lane_id)
        if not keys or keys[-1] != key:
            keys.append(key)
    return keys, None, max_error


def connected_native_sequence(keys, roads):
    """Prove each sampled transition using explicit OpenDRIVE road/lane links."""
    for a, b in zip(keys, keys[1:]):
        road_a, section_a, lane_a = a
        road_b, section_b, lane_b = b
        direction = "successor" if lane_a < 0 else "predecessor"
        sections = roads[road_a].findall("./lanes/laneSection")
        lane = next(l for l in sections[section_a].findall("./*/lane")
                    if int(l.get("id")) == lane_a)
        link = lane.find("./link/" + direction)
        if link is None or int(link.get("id")) != lane_b:
            return False
        if road_a == road_b:
            if section_b != section_a + (1 if lane_a < 0 else -1):
                return False
        else:
            # Reject skipped sections, junction links and reversed road contacts.
            if section_a != (len(sections)-1 if lane_a < 0 else 0):
                return False
            road_link = roads[road_a].find("./link/" + direction)
            if road_link is None or road_link.get("elementType") != "road" or \
                    int(road_link.get("elementId")) != road_b:
                return False
            contact = "start" if lane_b < 0 else "end"
            target_sections = roads[road_b].findall("./lanes/laneSection")
            if road_link.get("contactPoint") != contact or \
                    section_b != (0 if lane_b < 0 else len(target_sections)-1):
                return False
    return True


def repair(map_path, xodr_path, output, manifest=None, output_manifest=None):
    tree = ET.parse(map_path)
    root = tree.getroot()
    xroot = ET.parse(xodr_path).getroot()
    native = carla.Map("Town05", Path(xodr_path).read_text())
    nodes = {n.get("id"): tuple(float(tags(n).get(k, 0)) for k in ("local_x", "local_y", "ele"))
             for n in root.findall("node")}
    ways = {w.get("id"): w for w in root.findall("way")}
    roads = {int(road.get("id")): road for road in xroot.findall("road")}
    sections = {(int(road.get("id")), i): section
                for road in xroot.findall("road")
                for i, section in enumerate(road.findall("./lanes/laneSection"))}
    uses, matches = defaultdict(list), {}
    for rel in root.findall("relation"):
        if tags(rel).get("type") != "lanelet" or tags(rel).get("subtype") != "road":
            continue
        bounds = {m.get("role"): m.get("ref") for m in rel.findall("member")
                  if m.get("role") in ("left", "right")}
        points = [[nodes[nd.get("ref")] for nd in ways[bounds[role]].findall("nd")]
                  for role in ("left", "right")]
        if any(len(p) < 2 for p in points):
            raise ValueError("Degenerate road lanelet " + rel.get("id"))
        matches[rel.get("id")] = match_lane(*points, native)
        for role, way_id in bounds.items():
            uses[way_id].append((rel.get("id"), role))
    records = []
    for way_id, lanes in uses.items():
        record = {"way_id": int(way_id), "lanelets": [int(l[0]) for l in lanes]}
        reason = None
        if len(lanes) != 2 or {l[1] for l in lanes} != {"left", "right"}:
            reason = "not_shared_same_direction_boundary"
        else:
            match_a, reason_a, error_a = matches[lanes[0][0]]
            match_b, reason_b, error_b = matches[lanes[1][0]]
            record["native_lanes"] = [match_a, match_b]
            record["max_center_match_error_m"] = max(error_a, error_b)
            reason = reason_a or reason_b
            if not reason and (len(match_a) != len(match_b) or not all(
                    same_direction_adjacent(a, b) for a, b in zip(match_a, match_b))):
                reason = "not_same_native_road_adjacent_lanes"
            if not reason and not all(connected_native_sequence(keys, roads)
                                      for keys in (match_a, match_b)):
                reason = "disconnected_native_sections"
            if not reason:
                record["markings"] = []
                for a, b in zip(match_a, match_b):
                    mark_lane = native_marking_lane(sections[a[:2]], a[2], b[2])
                    record["markings"].append({"road_section": a[:2],
                        "road_marks": [dict(m.attrib) for m in mark_lane.findall("roadMark")]})
                    if not constant_broken_both(mark_lane):
                        reason = "not_uniform_broken_both_in_native_section"
        current = tags(ways[way_id])
        owned = current.get("carla:lane_marking_source") == SOURCE
        if owned and (reason or any(current.get(k) != v for k, v in OWNED_TAGS.items())):
            raise ValueError(f"Previously generated boundary {way_id} changed or no longer matches native source; preserve and review it")
        if not reason and not owned and any(k in current for k in ("type", "subtype", "lane_change", "lane_change:left", "lane_change:right")):
            reason = "existing_user_boundary_tags_preserved"
        if reason:
            record.update(allowed=False, reason=reason)
        else:
            if not owned:
                for key, value in OWNED_TAGS.items():
                    ET.SubElement(ways[way_id], "tag", k=key, v=value)
            record.update(allowed=True, reason="native_constant_broken_both", already_tagged=owned)
        records.append(record)
    if str(Path(map_path).resolve()) == str(Path(output).resolve()):
        raise ValueError("Write a candidate, never modify input in place")
    tree.write(output, encoding="UTF-8", xml_declaration=True)
    result = {"schema": 2, "matcher_version": 2, "source": SOURCE, "input_map_sha256": sha(map_path),
              "output_map_sha256": sha(output), "xodr_sha256": sha(xodr_path),
              "road_lanelets": len(matches), "road_boundaries": len(uses),
              "allowed_boundaries": sum(r["allowed"] for r in records),
              "added_boundaries": sum(r["allowed"] and not r["already_tagged"] for r in records),
              "reasons": dict(Counter(r["reason"] for r in records)), "boundaries": records,
              "conservative_policy": "Unsplit mixed/ambiguous native sections remain forbidden; existing user tags are never overwritten."}
    if manifest:
        if not output_manifest:
            raise ValueError("--output-manifest is required with --manifest")
        data = json.loads(Path(manifest).read_text())
        light_ids = {int(r.get("id")) for r in root.findall("relation")
                     if tags(r).get("type") == "regulatory_element" and tags(r).get("subtype") == "traffic_light"}
        if data.get("map_name") != "Town05" or {g["group_id"] for g in data.get("groups", [])} != light_ids:
            raise ValueError("Traffic signal groups do not match candidate map")
        result["input_manifest_map_sha256"] = data.get("map_sha256")
        data["map_sha256"] = result["output_map_sha256"]
        write_json(output_manifest, data)
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    for name in ("map", "xodr", "output", "report"):
        p.add_argument("--"+name, required=True, type=Path)
    p.add_argument("--manifest", type=Path)
    p.add_argument("--output-manifest", type=Path)
    args = p.parse_args()
    result = repair(args.map, args.xodr, args.output, args.manifest, args.output_manifest)
    write_json(args.report, result)
    print(json.dumps({k: v for k, v in result.items() if k != "boundaries"}, indent=2))


if __name__ == "__main__":
    main()
