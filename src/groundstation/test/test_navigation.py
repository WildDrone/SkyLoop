"""Golden tests pinning navigation math to values captured before the refactor.

Run: ``python -m groundstation.test_navigation`` or ``pytest test_navigation.py``.
These lock the pure formulas so the mission_controller/perpetual_monitor dedup
provably keeps identical output.
"""

from groundstation import navigation as nav


def test_golden():
    approx = lambda a, b: abs(a - b) < 1e-9

    assert approx(nav.haversine_distance(55.0, 10.0, 55.01, 10.01), 1281.8358618969785)
    assert approx(nav.haversine_distance(0, 0, 0, 1), 111194.92664455874)
    assert approx(nav.calculate_bearing(55.0, 10.0, 55.01, 10.01), 29.83038851325074)
    assert approx(nav.calculate_bearing(0, 0, 0, 1), 90.0)

    # mission_controller variant: 5s + climb + 5s + horizontal
    assert nav.estimate_travel_time(300, 50, 15, 4,
                                    wait_after_takeoff=5.0, wait_after_climb=5.0) == 42.5
    assert nav.estimate_travel_time(300, 50, 0, 4,
                                    wait_after_takeoff=5.0, wait_after_climb=5.0) == float('inf')
    # calculator variant: no waits
    assert nav.estimate_travel_time(300) == 32.5
    assert nav.estimate_travel_time(300, 80, 10, 5) == 46.0

    assert nav.calculate_relay_countdown(600, 120) == 420.0
    assert nav.calculate_relay_countdown(100, 120) == 0.0

    assert nav.calculate_drones_needed(3600, 900, 120) == 6
    assert nav.calculate_drones_needed(3600, 200, 120) == float('inf')
    assert nav.calculate_drones_needed(3600, 0, 120) == 0


if __name__ == '__main__':
    test_golden()
    print('navigation golden tests passed')
