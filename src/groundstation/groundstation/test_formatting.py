"""Golden tests: helpers must byte-match the inline expressions they replace."""

from groundstation import formatting as fmt


def test_golden():
    for s in [0, 5, 59, 60, 61, 125, 599, 3600, 3661, 7325.7]:
        assert fmt.format_mmss(s) == f"{int(s // 60)}:{int(s % 60):02d}"
        assert fmt.format_hms(s) == (f"{int(s // 3600):02d}:"
                                     f"{int((s % 3600) // 60):02d}:{int(s % 60):02d}")
    # spot values
    assert fmt.format_mmss(125) == "2:05"
    assert fmt.format_mmss(59) == "0:59"
    assert fmt.format_hms(3661) == "01:01:01"

    for lvl in [100, 51, 50, 21, 20, 0]:
        assert fmt.battery_color(lvl) == ('#4caf50' if lvl > 50 else '#ff9800' if lvl > 20 else '#f44336')
        assert fmt.battery_status_color(lvl) == ('green' if lvl > 50 else 'orange' if lvl > 20 else 'red')


if __name__ == '__main__':
    test_golden()
    print('formatting golden tests passed')
