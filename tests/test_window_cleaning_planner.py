from window_cleaner.window_cleaning_planner import (
    CameraProjectionConfig,
    DirtMemoryMap,
    LawnMowerScanPlanner,
    MapPoint,
    PixelDetection,
    project_pixel_detection_to_window,
)


def test_dirt_memory_merges_near_observations_and_keeps_far_points():
    memory = DirtMemoryMap(merge_radius_cm=5.0)

    memory.add_observation(MapPoint(10.0, 20.0))
    merged = memory.add_observation(MapPoint(12.0, 22.0))
    memory.add_observation(MapPoint(40.0, 80.0))

    assert len(memory.points) == 2
    assert merged.observations == 2
    assert round(merged.x_cm, 1) == 11.0
    assert round(merged.y_cm, 1) == 21.0


def test_lawnmower_scan_covers_window_with_alternating_waypoints():
    planner = LawnMowerScanPlanner(80.0, 160.0, margin_cm=15.0, stripe_spacing_cm=25.0)

    assert planner.waypoints == [
        MapPoint(15.0, 15.0),
        MapPoint(15.0, 145.0),
        MapPoint(40.0, 145.0),
        MapPoint(40.0, 15.0),
        MapPoint(65.0, 15.0),
        MapPoint(65.0, 145.0),
    ]


def test_project_pixel_detection_to_window_uses_robot_pose_and_camera_offsets():
    point = project_pixel_detection_to_window(
        PixelDetection(x_px=420.0, y_px=140.0),
        robot_pose=MapPoint(30.0, 50.0),
        calibration=CameraProjectionConfig(
            image_width_px=640,
            image_height_px=480,
            cm_per_pixel_x=0.1,
            cm_per_pixel_y=0.2,
            camera_offset_x_cm=2.0,
            camera_offset_y_cm=3.0,
        ),
    )

    assert point == MapPoint(42.0, 73.0)



def test_heading_to_target_uses_window_up_as_zero():
    from window_cleaner.window_cleaning_planner_node import heading_to_target

    assert heading_to_target(MapPoint(0.0, 0.0), MapPoint(0.0, 10.0)) == 0.0
    assert round(heading_to_target(MapPoint(0.0, 0.0), MapPoint(10.0, 0.0)), 6) == round(1.5707963267948966, 6)
