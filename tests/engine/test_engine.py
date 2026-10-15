"""Pure-Python tests: RuView packet parsing and the presence engine (no HA needed)."""
import json
import pathlib
import sys

sys.path.insert(0, str(pathlib.Path(__file__).parents[2] / "custom_components" / "real2sim_home"))
import engine  # noqa: E402
import ruview  # noqa: E402

ROOM = json.loads((pathlib.Path(__file__).parents[1] / "homeassistant" / "sample_room.json").read_text())


def test_vitals_round_trip():
    p = ruview.parse(ruview.build_vitals(4, heart=71.25, breathing=13.5, fall=True, n_persons=2))
    assert isinstance(p, ruview.Vitals)
    assert (p.node_id, p.heart_bpm, p.breathing_bpm, p.fall, p.presence, p.n_persons) == (4, 71.25, 13.5, True, True, 2)
    assert ruview.parse(b"\x00\x01") is None
    assert ruview.parse(ruview.build_vitals(1, heart=0, breathing=0)).heart_bpm is None


def test_hub_counts_csi_rate():
    t = [100.0]
    hub = ruview.RuViewHub(clock=lambda: t[0])
    for i in range(60):
        hub.handle(ruview.build_csi(2, i))
        t[0] += 0.1
    assert hub.nodes[2].csi_frames == 60 and 9 < hub.nodes[2].csi_rate < 11


def _snap(**kw):
    room = engine.Room("living_room", "Living room", engine.validate_room_map(ROOM), "area1")
    base = dict(rooms=[room], sensors=[], nodes=[], radar=[], persons=[], now=1000.0)
    base.update(kw)
    return engine.Snapshot(**base)


def test_empty_room_is_free():
    out = engine.fuse(_snap())
    assert out["rooms"][0]["occupied"] is False and out["people"] == []


def test_ruview_vitals_go_to_the_person_in_the_room():
    node = engine.Node(1, "living_room", (0.3, 0.3), presence=True, heart=64, breathing=12, n_persons=1, vitals_age=1)
    out = engine.fuse(_snap(nodes=[node]))
    (p,) = out["people"]
    assert p["heart_rate"] == 64 and p["breathing_rate"] == 12 and p["heart_rate_source"] == "ruview:1"
    assert p["pose"] == "sitting" and p["on"] == "sofa_1"    # still + a sofa in the room -> on the sofa


def test_radar_position_and_pose_from_furniture():
    tgt = [engine.RadarTarget("living_room", 2.4, 0.7), engine.RadarTarget("living_room", 1.0, 3.0, speed=0.6)]
    person = engine.Person("person.ana", "Ana", room_id="living_room", heart=80, heart_age=10, heart_source="sensor.ana_watch_heart_rate")
    out = engine.fuse(_snap(radar=tgt, persons=[person]))
    assert len(out["people"]) == 2
    ana = next(p for p in out["people"] if p["id"] == "person.ana")
    assert (ana["x"], ana["y"], ana["pose"]) == (1.0, 3.0, "walking") or (ana["x"], ana["y"], ana["pose"]) == (2.4, 0.7, "sitting")
    assert ana["heart_rate"] == 80
    poses = sorted(p["pose"] for p in out["people"])
    assert poses == ["sitting", "walking"]


def test_fall_flag_marks_fallen():
    node = engine.Node(1, "living_room", None, presence=True, fall=True, heart=90, vitals_age=1, last_fall_age=1)
    out = engine.fuse(_snap(nodes=[node]))
    assert out["rooms"][0]["fall"] and out["people"][0]["pose"] == "fallen"


def test_motion_sensor_hold_and_markers():
    s = [engine.Sensor("binary_sensor.lr_motion", "living_room", "motion", "Motion", "off", on=False, last_changed=990.0),
         engine.Sensor("sensor.lr_temp", "living_room", "temperature", "Temp", "21.46", "°C"),
         engine.Sensor("camera.lr", "living_room", "camera", "Cam", "idle", pos=(4.9, 3.9))]
    out = engine.fuse(_snap(sensors=s))
    r = out["rooms"][0]
    assert r["occupied"] and r["temperature"] == 21.5 and r["count"] == 1
    cams = [m for m in out["markers"] if m["kind"] == "camera"]
    assert cams[0]["placed"] and cams[0]["x"] == 4.9
    assert all(0 <= m["x"] <= 5 and 0 <= m["y"] <= 4 for m in out["markers"])


def test_named_person_located_by_room_sensor_without_other_sources():
    p = engine.Person("person.bo", "Bo", room_id="living_room", activity="walking")
    out = engine.fuse(_snap(persons=[p]))
    assert out["people"][0]["name"] == "Bo" and out["people"][0]["pose"] == "walking"


def test_validate_and_layout():
    import pytest
    with pytest.raises(ValueError):
        engine.validate_room_map({"format": "x"})
    m = engine.validate_room_map({"format": "r2s-roommap", "name": "Küche 2", "walls": [{"a": [0, 0], "b": [3, 0]}, {"a": [3, 0], "b": [3, 2]}]})
    assert m["id"] == "k_che_2" and m["bounds"] == [0, 0, 3, 2] and len(m["floor"]) == 4
    rooms = [engine.Room("a", "A", {"bounds": [0, 0, 5, 4]}), engine.Room("b", "B", {"bounds": [0, 0, 3, 3]})]
    lay = engine.auto_layout(rooms, {})
    assert lay["a"]["x"] == 0 and lay["b"]["x"] == 5.6
