from src.gridmap import GridMap


def corridor_map():
    """Ban do hanh lang: 9 khoi, hanh lang rong 6 m, nhieu goc vuong. Tra ve (map, cac diem giao)."""
    m = GridMap(50.0, 0.5)
    m.add_border(1.0)
    for (x0, x1) in [(6, 16), (22, 32), (38, 46)]:
        for (y0, y1) in [(6, 16), (22, 32), (38, 46)]:
            m.add_rect(x0, y0, x1, y1)
    junctions = [(3, 3), (19, 19), (35, 19), (35, 35), (19, 35), (3, 35)]
    return m, junctions



def open_map():
    """Ban do rong co duong cong: vat can tron/elip, khong gian thoang. Tra ve (map, cac diem giao)."""
    m = GridMap(50.0, 0.5)
    m.add_border(1.0)
    m.add_circle(15, 15, 6)
    m.add_circle(35, 12, 4)
    m.add_ellipse(25, 30, 9, 4, angle=0.5)
    m.add_circle(8, 38, 5)
    m.add_circle(42, 36, 5)
    junctions = [(4, 4), (25, 8), (45, 20), (30, 42), (12, 26), (4, 46)]
    return m, junctions