
import numpy as np
from scipy.interpolate import CubicSpline
from evo.core.trajectory import PoseTrajectory3D

def calculate_yaw_from_points(x, y):
    """Calculate yaw based on the tangent of the trajectory."""
    dx = np.diff(x)
    dy = np.diff(y)
    yaw = np.arctan2(dy, dx) * 180 / np.pi  # Convert radians to degrees
    yaw = np.append(yaw, yaw[-1])  # Repeat the last yaw for array alignment
    return yaw

def calculate_pitch_from_points(x, y, z):
    """Calculate pitch based on the slope of the trajectory."""
    dx = np.diff(x)
    dy = np.diff(y)
    dz = np.diff(z)
    horizontal_distance = np.sqrt(dx**2 + dy**2)  # Distance in the horizontal plane
    pitch = np.arctan2(dz, horizontal_distance) * 180 / np.pi  # Convert radians to degrees
    pitch = np.append(pitch, pitch[-1])  # Repeat the last pitch for array alignment
    return -pitch


def trajectory_generation(points, use_yaw_input, use_pitch_input, sampling_rate) -> PoseTrajectory3D:
    if len(points) < 2:
        print("At least two points are required to generate a trajectory.")
        return

    # Extract x, y, z, yaw, roll, pitch, and velocity from points
    x, y, z, yaw, roll, pitch, velocity = zip(*points)

    distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2 + np.diff(z)**2)  # Approximate distances
    time_intervals = distances / np.minimum(velocity[:-1], velocity[1:])  # Use the smaller velocity at each segment
    t = np.insert(np.cumsum(time_intervals), 0, 0)  # Cumulative time

    # Smooth interpolation with cubic splines (zero velocity constraint)
    spline_x = CubicSpline(t, x, bc_type=((1, 0.0), (1, 0.0)))
    spline_y = CubicSpline(t, y, bc_type=((1, 0.0), (1, 0.0)))
    spline_z = CubicSpline(t, z, bc_type=((1, 0.0), (1, 0.0)))
    spline_roll = CubicSpline(t, roll, bc_type=((1, 0.0), (1, 0.0)))

    if use_yaw_input:
        spline_yaw = CubicSpline(t, yaw, bc_type=((1, 0.0), (1, 0.0)))
    if use_pitch_input:
        spline_pitch = CubicSpline(t, pitch, bc_type=((1, 0.0), (1, 0.0)))

    # Sample at 10Hz
    total_time = t[-1]
    num_samples = max(int(sampling_rate * total_time), 2)  # Ensure at least two samples
    t_smooth = np.linspace(0, total_time, num_samples)
    smooth_x = spline_x(t_smooth)
    smooth_y = spline_y(t_smooth)
    smooth_z = spline_z(t_smooth)
    smooth_roll = spline_roll(t_smooth)

    if use_yaw_input:
        smooth_yaw = spline_yaw(t_smooth)
    else:
        smooth_yaw = calculate_yaw_from_points(smooth_x, smooth_y)

    if use_pitch_input:
        smooth_pitch = spline_pitch(t_smooth)
    else:
        smooth_pitch = calculate_pitch_from_points(smooth_x, smooth_y, smooth_z)

    # Calculate velocities
    #velocity_x = spline_x.derivative()(t_smooth)
    #velocity_y = spline_y.derivative()(t_smooth)
    #velocity_z = spline_z.derivative()(t_smooth)
    #velocities = np.sqrt(velocity_x**2 + velocity_y**2 + velocity_z**2)

    traj = create_evo_trajecory(smooth_x, smooth_y, smooth_z, smooth_yaw, smooth_pitch, smooth_roll, t_smooth)
    
    traj.meta["base_points"] = points

    return traj


def create_evo_trajecory(x,y,z,yaw,pitch,roll,t) -> PoseTrajectory3D:
    
    # Prepare SE3 poses and timestamps
    poses = []
    for i in range(len(t)):
        # Translation vector
        translation = np.array([x[i], y[i], z[i]])

        # Compute rotation matrix from yaw, pitch, and roll
        yaw_, pitch_, roll_ = (
            np.radians(yaw[i]),
            np.radians(pitch[i]),
            np.radians(roll[i]),
        )
        R_yaw = np.array([
            [np.cos(yaw_), -np.sin(yaw_), 0],
            [np.sin(yaw_),  np.cos(yaw_), 0],
            [0,            0,           1],
        ])
        R_pitch = np.array([
            [np.cos(pitch_), 0, np.sin(pitch_)],
            [0,             1, 0           ],
            [-np.sin(pitch_), 0, np.cos(pitch_)],
        ])
        R_roll = np.array([
            [1, 0,            0           ],
            [0, np.cos(roll_), -np.sin(roll_)],
            [0, np.sin(roll_),  np.cos(roll_)],
        ])
        rotation_matrix = R_yaw @ R_pitch @ R_roll

        # Combine translation and rotation into SE3 matrix
        se3_pose = np.eye(4)
        se3_pose[:3, :3] = rotation_matrix
        se3_pose[:3, 3] = translation

        poses.append(se3_pose)

    # Create PoseTrajectory3D
    trajectory = PoseTrajectory3D(poses_se3=poses, timestamps=t)

    return trajectory