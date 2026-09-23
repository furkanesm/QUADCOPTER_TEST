import os
import sys
import pytest
import cv2
import numpy as np

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from plan_route_from_mask import validate_coordinates, find_target_point, extract_mask_layer, apply_connection_mask

def create_synthetic_image(grid, rgb_channels):
    """
    Creates an image from grid where all values are duplicated to channels.
    """
    h, w = grid.shape
    if rgb_channels:
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[:,:,0] = grid
        img[:,:,1] = grid
        img[:,:,2] = grid
    else:
        img = grid.copy()
    return img

def test_extract_mask_layer():
    # 0, 1, 2, 3 valid. 4 is invalid
    grid = np.array([[0, 1], [2, 3]], dtype=np.uint8)
    img = create_synthetic_image(grid, True)
    res = extract_mask_layer(img)
    assert np.array_equal(res, grid)

def test_extract_mask_layer_invalid_value():
    grid = np.array([[0, 1], [4, 3]], dtype=np.uint8)
    img = create_synthetic_image(grid, True)
    with pytest.raises(SystemExit):
        extract_mask_layer(img) # should sys.exit(1)

def test_extract_mask_layer_mismatched_channels():
    img = np.zeros((2, 2, 3), dtype=np.uint8)
    img[:,:,0] = 0
    img[:,:,1] = 1
    with pytest.raises(SystemExit):
        extract_mask_layer(img)

def test_find_target_point_multiple_targets():
    grid = np.array([
        [0, 3, 0],
        [0, 0, 0],
        [0, 3, 0]
    ], dtype=np.uint8)
    with pytest.raises(SystemExit):
        find_target_point(grid)

def test_find_target_point_success():
    grid = np.array([
        [0, 0, 0],
        [0, 3, 3],
        [0, 3, 3]
    ], dtype=np.uint8)
    pt = find_target_point(grid)
    # Centroid of the block 
    assert pt == (1, 1) or pt == (2, 2) or pt == (1, 2) or pt == (2, 1) or pt == (1, 1) # OpenCV centroid might be float rounded to int
    
    assert pt in [(1,1), (2,2), (1,2), (2,1)]

def test_apply_connection_mask_rejects_obstacle():
    # grid: 1 is free, 2 is obstacle, 0 is unknown
    grid = np.array([
        [1, 0, 2],
        [1, 0, 2],
        [1, 0, 2]
    ], dtype=np.uint8)
    conn_img = np.zeros((3,3), dtype=np.uint8)
    conn_img[0, 2] = 1 # touches OBSTACLE (2)
    with pytest.raises(SystemExit):
        apply_connection_mask(grid, conn_img)

def test_apply_connection_mask_applies_successfully():
    grid = np.array([
        [1, 0, 2],
        [1, 0, 2],
        [1, 0, 2]
    ], dtype=np.uint8)
    conn_img = np.zeros((3,3), dtype=np.uint8)
    conn_img[1, 1] = 1 # touches Unknown (0)
    
    new_grid, added = apply_connection_mask(grid.copy(), conn_img)
    assert new_grid[1, 1] == 1 # Now FREE
    assert added[1, 1] == True
    assert new_grid[0, 2] == 2 # Obstacle still obstacle

def test_full_synthetic_routing(tmp_path):
    # Test through main using CLI args mock
    out_dir = str(tmp_path)
    
    # 5x5 Grid
    # S = Start
    # T = Target(3)
    # O = Obstacle(2)
    # U = Unknown(0)
    # F = Free(1)
    # y=0: F F O U U
    # y=1: S F O U T
    # y=2: F F O F F
    # y=3: O F F F F
    # y=4: O O O O U
    
    grid = np.array([
        [1, 1, 2, 0, 0],
        [1, 1, 2, 0, 3], # start at (0,1), target at (4,1)
        [1, 1, 2, 1, 1],
        [2, 1, 1, 1, 1],
        [2, 2, 2, 2, 0]
    ], dtype=np.uint8)
    
    rgb = create_synthetic_image(grid * 40, True) # Just visual
    mask = create_synthetic_image(grid, True)
    
    rgb_file = os.path.join(out_dir, "rgb.png")
    mask_file = os.path.join(out_dir, "mask.png")
    cv2.imwrite(rgb_file, rgb)
    cv2.imwrite(mask_file, mask)
    
    from plan_route_from_mask import main
    import sys
    
    sys.argv = ["plan_route_from_mask.py", "--image", rgb_file, "--mask", mask_file, "--start", "0", "1", "--output-dir", out_dir]
    
    # Should path down and around obstacle?
    # At (0,1), path to (4,1).
    # O at x=2 blocks straight. Must go via x=1..x=3 at y=3.
    # U at (3,0), (4,0), (3,1) acts as obstacle.
    
    try:
        main()
    except SystemExit as e:
        assert e.code == 0 or e.code == None
        
    import json
    out_json = os.path.join(out_dir, "route_output.json")
    with open(out_json, "r") as f:
        data = json.load(f)
        
    assert data["status"] == "SUCCESS"
    assert len(data["waypoints"]) > 0
    # Coordinates should not touch obstacle
    for wp in data["waypoints"]:
        # In grid pixel space
        px, py = wp["x"] - 0.5, wp["y"] - 0.5 # since +0.5 was added in GridPlanner scaling
        px, py = int(px), int(py)
        assert grid[py, px] != 2
        assert grid[py, px] != 0 # Should not walk on unknown since it's an obstacle

def test_full_synthetic_routing_no_path(tmp_path):
    # Test through main using CLI args mock
    out_dir = str(tmp_path)
    
    grid = np.array([
        [1, 1, 2, 0, 0],
        [1, 1, 2, 0, 3], # start at (0,1), target at (4,1)
        [1, 1, 2, 1, 1],
        [1, 1, 2, 1, 1],
        [1, 1, 2, 1, 1]
    ], dtype=np.uint8)
    # Blocked completely by column x=2 being obstacle=2
    
    rgb = create_synthetic_image(grid * 40, True) # Just visual
    mask = create_synthetic_image(grid, True)
    
    rgb_file = os.path.join(out_dir, "rgb.png")
    mask_file = os.path.join(out_dir, "mask.png")
    cv2.imwrite(rgb_file, rgb)
    cv2.imwrite(mask_file, mask)
    
    from plan_route_from_mask import main
    import sys
    
    sys.argv = ["plan_route_from_mask.py", "--image", rgb_file, "--mask", mask_file, "--start", "0", "1", "--output-dir", out_dir]
    
    main()
        
    import json
    out_json = os.path.join(out_dir, "route_output.json")
    with open(out_json, "r") as f:
        data = json.load(f)
        
    assert data["status"] == "NO_PATH"
    assert len(data["waypoints"]) == 0
