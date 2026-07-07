import numpy as np


CONE_SPECS = {
    "large_orange": {
        "aliases": {"large_orange", "orange", "big_orange", "大号橘色桩桶", "橘色"},
        "size": np.array([0.35, 0.35, 0.70], dtype=float),
    },
    "small_red": {
        "aliases": {"small_red", "red", "小号红色桩桶", "红色"},
        "size": np.array([0.20, 0.20, 0.30], dtype=float),
    },
    "small_yellow": {
        "aliases": {"small_yellow", "yellow", "小号黄色桩桶", "黄色"},
        "size": np.array([0.20, 0.20, 0.30], dtype=float),
    },
    "small_blue": {
        "aliases": {"small_blue", "blue", "小号蓝色桩桶", "蓝色"},
        "size": np.array([0.20, 0.20, 0.30], dtype=float),
    },
}

DEFAULT_CONE_TYPE = "small_blue"


def front(car_position, car_heading, cone_position):
    """Return True when the cone is in front of the vehicle/LiDAR."""

    delta = cone_position[:2] - car_position[:2]
    x_forward = delta[0] * np.cos(car_heading) + delta[1] * np.sin(car_heading)
    return bool(x_forward > 0.0)


def angle_judge(car_position, car_heading, cone_position, fov_deg=120.0):
    """Return True when the cone is inside the horizontal LiDAR FOV.
    判断锥桶是否在120°角度内
    """

    dx = cone_position[0] - car_position[0]
    dy = cone_position[1] - car_position[1]
    angle_to_cone = np.arctan2(dy, dx)
    relative_angle = angle_to_cone - car_heading
    relative_angle = (relative_angle + np.pi) % (2 * np.pi) - np.pi
    half_fov = np.radians(fov_deg / 2.0)
    return bool(abs(relative_angle) <= half_fov)


def distance_judge(car_position, cone_position):
    """Return True when the cone is inside the valid LiDAR range.
    距离判断
    np.linalg.norm() 计算两个点之间的欧几里得距离
    因为车辆高度置0，不需要考虑z轴高度，距离计算只需要考虑xy坐标
    """
    car_position = car_position[:,:2]
    cone_position = cone_position[:,:2]
    dist = np.linalg.norm(cone_position - car_position)
    return bool(1.5 <= dist <= 50.0)


def _cone_size_from_kind(kind):
    """Return cone dimensions [width, depth, height] in meters."""
    if kind is None:
        return CONE_SPECS[DEFAULT_CONE_TYPE]["size"].copy()

    normalized = str(kind).strip().lower()
    for spec in CONE_SPECS.values():
        aliases = {alias.lower() for alias in spec["aliases"]}
        if normalized in aliases:
            return spec["size"].copy()

    raise ValueError(f"Unknown cone type/color: {kind!r}")


def _parse_obstacle_box(obstacle):
    """Normalize a cone obstacle to an axis-aligned bounding box."""
    if isinstance(obstacle, dict):
        if "bbox_min" in obstacle and "bbox_max" in obstacle:
            bbox_min = np.asarray(obstacle["bbox_min"], dtype=float)[:3]
            bbox_max = np.asarray(obstacle["bbox_max"], dtype=float)[:3]
            return np.minimum(bbox_min, bbox_max), np.maximum(bbox_min, bbox_max)

        position = obstacle.get("position", obstacle.get("center"))
        if position is None:
            raise ValueError("Cone obstacle requires 'position' or 'center'.")

        size = obstacle.get("size", obstacle.get("dimensions"))
        if size is None:
            kind = obstacle.get("type", obstacle.get("name", obstacle.get("color")))
            size = _cone_size_from_kind(kind)
    else:
        values = np.asarray(obstacle, dtype=float)
        if values.shape[0] == 3:
            position = values
            size = _cone_size_from_kind(DEFAULT_CONE_TYPE)
        elif values.shape[0] >= 6:
            position = values[:3]
            size = values[3:6]
        else:
            raise ValueError(
                "Cone obstacle must be a dict, [x, y, z], or "
                "[x, y, z, width, depth, height]."
            )

    position = np.asarray(position, dtype=float)[:3]
    size = np.asarray(size, dtype=float)[:3]
    if position.shape[0] < 3 or size.shape[0] < 3:
        raise ValueError("Cone obstacle position and size must contain 3 values.")
    if np.any(size <= 0.0):
        raise ValueError("Cone obstacle size values must be positive.")

    half_xy = size[:2] / 2.0
    bbox_min = np.array(
        [position[0] - half_xy[0], position[1] - half_xy[1], position[2]],
        dtype=float,
    )
    bbox_max = np.array(
        [position[0] + half_xy[0], position[1] + half_xy[1], position[2] + size[2]],
        dtype=float,
    )
    return bbox_min, bbox_max


def _segment_intersects_aabb(start, end, bbox_min, bbox_max, eps=1e-9):
    """Slab test for an open line segment intersecting an AABB."""
    direction = end - start
    t_min = 0.0
    t_max = 1.0

    for axis in range(3):
        if abs(direction[axis]) <= eps:
            if start[axis] < bbox_min[axis] or start[axis] > bbox_max[axis]:
                return False
            continue

        inv_dir = 1.0 / direction[axis]
        t1 = (bbox_min[axis] - start[axis]) * inv_dir
        t2 = (bbox_max[axis] - start[axis]) * inv_dir
        if t1 > t2:
            t1, t2 = t2, t1
        t_min = max(t_min, t1)
        t_max = min(t_max, t2)
        if t_min > t_max:
            return False

    return t_max > eps and t_min < 1.0 - eps


def occlusion_judge(car_position, cone_position, obstacles, eps=1e-9):
    """
    Judge whether the line of sight from LiDAR to cone is blocked.

    Obstacles are modeled from the cone size table as vertical axis-aligned
    boxes. Returns True when any box intersects the open segment from
    car_position to cone_position.
    """
    if not obstacles:
        return False

    car_position = np.asarray(car_position, dtype=float)[:3]
    cone_position = np.asarray(cone_position, dtype=float)[:3]
    ray = cone_position - car_position
    ray_len_sq = float(np.dot(ray, ray))

    if ray_len_sq <= eps:
        return False

    for obstacle in obstacles:
        bbox_min, bbox_max = _parse_obstacle_box(obstacle)
        obstacle_position = np.array(
            [
                (bbox_min[0] + bbox_max[0]) / 2.0,
                (bbox_min[1] + bbox_max[1]) / 2.0,
                bbox_min[2],
            ],
            dtype=float,
        )
        if np.allclose(obstacle_position, cone_position, atol=eps):
            continue

        if _segment_intersects_aabb(car_position, cone_position, bbox_min, bbox_max, eps):
            return True

    return False


def judge(car_position, car_heading, cone_position, obstacles=None):
    """Return True when a cone can be scanned by the LiDAR."""
    if not front(car_position, car_heading, cone_position):
        return False
    if not angle_judge(car_position, car_heading, cone_position):
        return False
    if not distance_judge(car_position, cone_position):
        return False
    if occlusion_judge(car_position, cone_position, obstacles):
        return False
    return True


## 生成 锥桶反射点云

def point_make_and_map(cone_position):
    """Placeholder for point-cloud generation, kept for the current skeleton."""
    return np.asarray(cone_position, dtype=float)


## 生成 模拟地面 

def plane_make():





def main(): 





    car_position = np.array([0.0, 0.0, 0.0])

    car_heading = 0.0

    cone_position = np.array([5.0, 0.0, 0.0])

    car_position = np.asarray(car_position, dtype=float)
    cone_position = np.asarray(cone_position, dtype=float)

    map = np.zeros((100, 100), dtype=int)

    obstacles = [{"position": np.array([2.5, 0.0, 0.0]), "color": "yellow"}]
    # "position": cone_position ,"color": "yellow"
    if(judge(car_position, car_heading, cone_position, obstacles)):
        point_make_and_map(cone_position)



if __name__ == "__main__":
    main()
