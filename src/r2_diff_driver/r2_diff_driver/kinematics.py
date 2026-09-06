"""Differential-drive kinematics helpers."""


def clamp(value, low, high):
    return max(low, min(high, value))


def twist_to_wheel_speeds(linear_x, angular_z, track_width):
    """Return (left_wheel_mps, right_wheel_mps) for a differential model."""
    # 差速运动学：只根据车体速度 (v, w) 和轮距反推左右轮应有多快。
    # 这是"运动学"(kinematics)而非"动力学"(dynamics)：
    #   运动学只管几何/几何关系——"轮子该转多快车才会这么走"，
    #   完全忽略力、质量、加速度响应、轮胎打滑、路面阻力这些因素。
    #
    # 对现在的需求够用，因为目标是把 R2 当作低速差速车(如 0.5 m/s)。
    # 只有当你要做精准轨迹跟踪、高速、或分析侧滑/加速度响应时，
    # 才需要在外面再叠一层"动力学"(车辆模型+控制律)。这里无需引入。
    half_track = track_width / 2.0
    left_mps = linear_x - angular_z * half_track
    right_mps = linear_x + angular_z * half_track
    return left_mps, right_mps


def wheel_speeds_to_twist(left_mps, right_mps, track_width):
    """Return (linear_x, angular_z) from wheel speeds."""
    linear_x = (left_mps + right_mps) / 2.0
    angular_z = (right_mps - left_mps) / track_width
    return linear_x, angular_z
