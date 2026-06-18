from window_cleaner.gazebo_dirt_cleaner import SimDirtDot, dirt_under_robot_footprint


def test_dirt_under_robot_footprint_uses_rectangular_cleaning_area():
    dots = [
        SimDirtDot("inside", 10.0, 20.0),
        SimDirtDot("outside_x", 26.0, 20.0),
        SimDirtDot("outside_y", 10.0, 36.0),
    ]

    cleaned = dirt_under_robot_footprint(10.0, 20.0, dots, half_width_cm=15.0, half_height_cm=15.0)

    assert [dot.name for dot in cleaned] == ["inside"]
