import copy
import json

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.figure import Figure
from scipy.optimize import least_squares, minimize
import matplotlib as mpl

mpl.rcParams['figure.dpi'] = 300


def jacobian(input_x, odometry):
    """
    Calculates the jacobian for the current input, distances and odometry inorder to calculates the Non Linear
    Least Squares
    @param input_x: The input of the function, initial x, y, and theta parameters
    @param distances: The UWB range measurements
    @param odometry_data: The associated odometry to the range measurements
    @return: An array for the residuals calculated as the difference between the range measurements and the range
     from the offset odometry data
    """
    # x[0] = x_start
    # x[1] = y_start
    # x[2] = z_start
    # x[3] = theta

    x_g, y_g, z_g, theta = input_x

    c = np.cos(theta)
    s = np.sin(theta)

    xyz_T = np.array([x_g, y_g, z_g])

    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    anchor_pos = odometry[:, 1:4]

    xyz_odom = odometry[:, 4:7]
    theta_odom = odometry[:, 7]

    o_c = np.cos(theta_odom + theta)
    o_s = np.sin(theta_odom + theta)

    o_R = np.zeros((len(odometry), 3, 3))
    o_R[:, 0, 0] = o_c
    o_R[:, 0, 1] = -o_s
    o_R[:, 1, 0] = o_s
    o_R[:, 1, 1] = o_c
    o_R[:, 2, 2] = 1

    xyz_tag = odometry[:, 4:7]

    odom = np.einsum('Ni,Bi ->BN', R, xyz_odom)
    tag = np.einsum('BNi,Bi ->BN', o_R, xyz_tag)
    xyz_jac = xyz_T - anchor_pos + odom + tag

    x = x_g - anchor_pos[:, 0]
    y = y_g - anchor_pos[:, 1]
    T = np.array([[y, x], [x, -y]])

    R = np.array([c, s])
    Ro = np.array([o_c, o_s])

    xy_odom = xyz_odom[:, :2]

    odom = np.einsum("N,NiB->Bi", R, T)
    complete = np.einsum("BN,BN->B", odom, xy_odom)

    odom = np.einsum("NB,NiB->Bi", Ro, T)
    xy_tag = xyz_tag[:, :2]
    complete2 = np.einsum("BN,BN->B", odom, xy_tag)

    theta_jac = complete + complete2

    return np.hstack([xyz_jac, theta_jac[:, None]]) * 2


def LSQ(initial_state, odometry, random_scale=1.):
    odometry_copy = odometry

    # np.random.normal(np.zeros(4), [5, 5, 0, np.pi / 2])

    random_odometry = np.random.normal(np.zeros(odometry.shape[1]),
                                       np.array([20 / 1000, 5 / 1000, 5 / 1000, 0, 5 / 1000, 5 / 1000, 0,
                                                 np.pi / 1000]) * random_scale,
                                       odometry.shape)

    odometry_copy += random_odometry
    res = least_squares(trilateration_function, np.zeros(4), args=(odometry_copy,),
                        loss='soft_l1',
                        f_scale=10.0,
                        jac='3-point',
                        # bounds=([-np.inf, -np.inf, -np.inf, -np.pi],
                        #         [np.inf, np.inf, np.inf, np.pi]),
                        # jac=jacobian
                        )

    local_mininum = res.x

    return np.array([local_mininum[0], local_mininum[1], local_mininum[2], local_mininum[3] % (2 * np.pi)])


def trilateration_function(input_x, odometry):
    """
    Calculates the residuals for the current input, distances and odometry inorder to calculates the Non Linear
    Least Squares
    @param input_x: The input of the function, initial x, y, and theta parameters
    @param distances: The UWB range measurements
    @param odometry_data: The associated odometry to the range measurements
    @return: An array for the residuals calculated as the difference between the range measurements and the range
    from the offset odometry data
    """
    # x[0] = x_start
    # x[1] = y_start
    # x[2] = z_start
    # x[3] = theta

    x_g, y_g, z_g, theta = input_x

    c = np.cos(theta)
    s = np.sin(theta)

    xyz_T = np.array([x_g, y_g, z_g])

    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    anchor_range = odometry[:, 0]
    anchor_pos = odometry[:, 1:4]

    xyz_odom = odometry[:, 4:7]
    theta_odom = odometry[:, 7]

    o_c = np.cos(theta_odom)
    o_s = np.sin(theta_odom)

    o_R = np.zeros((len(odometry), 3, 3))
    o_R[:, 0, 0] = o_c
    o_R[:, 0, 1] = -o_s
    o_R[:, 1, 0] = o_s
    o_R[:, 1, 1] = o_c
    o_R[:, 2, 2] = 1

    xyz_tag = np.zeros_like(xyz_odom)

    local_xyz_tag = np.einsum('BNi,Bi ->BN', o_R, xyz_tag) + xyz_odom

    global_xyz_tag = np.einsum('Ni,Bi ->BN', R, local_xyz_tag) + xyz_T

    residuals = np.linalg.norm(global_xyz_tag - anchor_pos, axis=1) ** 2 - anchor_range ** 2

    return residuals


class Robot:
    def __init__(self, xy: np.ndarray = None, theta: float = None, odometry=False):

        self.p = np.array([0, 0, 0], dtype=float)

        self.theta = 0.

        if theta is not None:
            self.theta = theta
        if xy is not None:
            self.p[:2] = xy

        self.odometry = odometry

        if self.odometry:
            self.p_o = np.zeros_like(self.p)
            self.theta_o = 0

    def rotation(self):
        c = np.cos(self.theta)
        s = np.sin(self.theta)

        return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])

    def rotation_odom(self):
        c = np.cos(self.theta_o)
        s = np.sin(self.theta_o)

        return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])

    def step(self, v, w, dt):
        rotM = self.rotation()

        rotation = rotM @ np.array([v, 0., 0.]) * dt
        rot = w * dt

        self.p += rotation
        self.theta += rot

        if self.odometry:
            rotM = self.rotation_odom()
            rotation = rotM @ np.array([v, 0., 0.]) * dt

            self.p_o += rotation
            self.theta_o += rot


def gen_random_robot(bounds=10, target=False):
    return Robot(np.random.uniform(-bounds, bounds, 2), np.random.uniform(-np.pi, np.pi), odometry=target)


def angle_diff(actual, target):
    diff = actual - target
    return np.arctan2(np.sin(diff), np.cos(diff))


def plot_result(result, odometry):
    xyz_odom = odometry[:, 4:7]
    theta_odom = odometry[:, 7]

    o_c = np.cos(theta_odom)
    o_s = np.sin(theta_odom)

    o_R = np.zeros((len(odometry), 3, 3))
    o_R[:, 0, 0] = o_c
    o_R[:, 0, 1] = -o_s
    o_R[:, 1, 0] = o_s
    o_R[:, 1, 1] = o_c
    o_R[:, 2, 2] = 1

    xyz_tag = np.zeros_like(xyz_odom)

    local_xyz_tag = np.einsum('BNi,Bi ->BN', o_R, xyz_tag) + xyz_odom

    x_g, y_g, z_g, theta = result

    c = np.cos(theta)
    s = np.sin(theta)

    xyz_T = np.array([x_g, y_g, z_g])

    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])

    # local_xyz_tag = np.zeros_like(local_xyz_tag)
    # local_xyz_tag[:, 0] = 10
    # local_xyz_tag[0, 0] = 0

    global_xyz_tag = np.einsum('Ni,Bi ->BN', R, local_xyz_tag) + xyz_T
    # global_xyz_tag = xyz_odom

    plt.plot(global_xyz_tag[:, 0], global_xyz_tag[:, 1], c='g')


def generate_output(N_ROBOTS=1, T=10, plot=False, random_scale=1.):
    BOUNDS = 20

    target_robot = gen_random_robot(BOUNDS, target=True)

    robots = [gen_random_robot(BOUNDS) for _ in range(N_ROBOTS)]

    dt = 0.1

    RANDOM_V = 2
    RANDOM_W = 2

    data = [
        [np.copy(r.p)] for r in robots
    ]

    target_robot_data = [
        np.copy(target_robot.p)
    ]

    initial_state = np.array([*target_robot.p, target_robot.theta])

    odometry_data = []

    for t_i in range(int(T // dt)):
        for i, robot in enumerate(robots):
            robot.step(np.random.uniform(-RANDOM_V / 2, RANDOM_V), np.random.uniform(-RANDOM_W, RANDOM_W), dt)

            data[i].append(np.copy(robot.p))

        target_robot.step(np.random.uniform(-RANDOM_V / 2, RANDOM_V), np.random.uniform(-RANDOM_W, RANDOM_W), dt)
        target_robot_data.append(np.copy(target_robot.p))

        random_robot = robots[np.random.randint(len(robots))]

        odometry_pose = np.copy(target_robot.p_o)
        anchor_pose = np.copy(random_robot.p)
        r = np.linalg.norm(random_robot.p - target_robot.p)

        odometry_data.append([r, *anchor_pose, *odometry_pose, target_robot.theta_o])

    data = np.array(data)

    odometry_data = np.array(odometry_data)
    target_robot_data = np.array(target_robot_data)

    output = LSQ(initial_state, odometry_data, random_scale)

    if plot:
        for i in range(data.shape[0]):
            plt.plot(data[i, :, 0], data[i, :, 1], c='b')

        for i in range(target_robot_data.shape[0]):
            plt.plot(target_robot_data[:, 0], target_robot_data[:, 1], c='r')

        # plot_result(output, odometry_data)
        # plot_result(initial_state, odometry_data)

        print(initial_state, output)
        print(output)

        plt.gca().set_xlabel('x (m)')
        plt.gca().set_ylabel('y (m)')

        plt.tight_layout()
        plt.gca().set_aspect('equal')
        plt.show()

    return initial_state, output


def generate_monte_carlo(noise_levels=False):
    MAX_N_ROBOTS = 5
    MAX_T = 10
    GENERATIONS = 1000

    monte_carlo_data = dict()

    if noise_levels:
        Tmax = [MAX_T]
        scaleMax = [0.001, 0.1, 1, 10, 100]
    else:
        Tmax = range(1, MAX_T + 1)
        scaleMax = [1]

    for N_ROBOTS in range(1, MAX_N_ROBOTS + 1):
        for_n_robot = []

        for T in Tmax:
            for scale in scaleMax:
                print(N_ROBOTS, T, scale)

                data = []

                for _ in range(GENERATIONS):
                    target, actual = generate_output(N_ROBOTS, T, random_scale=scale)

                    data.append([*actual, *target])

                data = np.array(data)

                pose_diff = data[:, 0:3] - data[:, 4: 7]
                theta_diff = angle_diff(data[:, 3], data[:, 7])

                differences = np.concatenate([pose_diff[:, :2], theta_diff[:, np.newaxis]], axis=1)

                for_n_robot.append(differences.tolist())

        monte_carlo_data[N_ROBOTS] = for_n_robot

    with open(f'save_data{"_noise" if noise_levels else ""}.json', 'w') as f:
        json.dump(monte_carlo_data, f)


def plot_monete_carlo():
    with open('save_data.json', 'r') as f:
        data = json.load(f)

    for nr, Ts in data.items():
        for t in Ts:
            diffs = np.array(t)

            plt.scatter(diffs[:, 0], diffs[:, 1], label=str(t))

        plt.title(str(nr))
        plt.gca().set_aspect('equal')
        plt.show()


def plot_violins(noise_levels=False):
    with open(f'save_data{"_noise" if noise_levels else ""}.json', 'r') as f:
        data = json.load(f)

    poses_fig: Figure = plt.figure()
    poses_ax = poses_fig.add_subplot()

    angles_fig: Figure = plt.figure()
    angles_ax = angles_fig.add_subplot()

    min_t = 1
    max_t = 1

    for nr, Ts in data.items():
        positions = []

        means = []
        errors = []
        min_quantile = []
        max_quantile = []

        angle_errors = []
        angle_min_quantile = []
        angle_max_quantile = []

        for t in Ts:
            diffs = np.array(t)

            angles = diffs[:, 2]

            range_diffs = np.linalg.norm(diffs[:, :2], axis=1)

            # means.append(range_diffs.mean())
            means.append(np.median(range_diffs))
            errors.append(np.std(range_diffs))
            min_quantile.append(np.quantile(range_diffs, q=0.25))
            max_quantile.append(np.quantile(range_diffs, q=0.75))

            angles = np.abs(angles)
            angle_errors.append(np.median(angles))
            angle_min_quantile.append(np.quantile(angles, q=0.25))
            angle_max_quantile.append(np.quantile(angles, q=0.75))

            positions.append(range_diffs)

        # plt.violinplot(positions, showmedians=True, showextrema=False, points=1000)
        # plt.boxplot(positions)

        x = np.array(range(len(means))) + 1

        if noise_levels:
            x = np.array([0.01, 0.1, 1, 10, 100])

        min_t = min(x)
        max_t = max(x)

        poses_ax.plot(x, means, label=f'N={nr}')
        poses_ax.fill_between(x, min_quantile, max_quantile, alpha=.3)
        # plt.errorbar(x, means, yerr=np.log(errors), lolims=0)

        angles_ax.plot(x, angle_errors, label=f'N={nr}')
        angles_ax.fill_between(x, angle_min_quantile, angle_max_quantile, alpha=0.3)

    poses_ax.set_ylabel('Median Position Error (m)')
    if noise_levels:
        poses_ax.set_xlabel('Noise Magnitude Scale')
    else:
        poses_ax.set_xlabel('Time Horizon (s)')
    poses_ax.legend()
    poses_ax.set_xlim([min_t, max_t])
    poses_ax.set_yscale('log')

    if noise_levels:
        poses_ax.set_xscale('log')
    poses_fig.tight_layout()
    poses_fig.savefig(f'monte_carlo_poses{"_noise" if noise_levels else ""}.png')

    angles_ax.set_ylabel('Absolute Median Angle Error (rad)')
    if noise_levels:
        angles_ax.set_xlabel('Noise Magnitude Scale')
    else:
        angles_ax.set_xlabel('Time Horizon (s)')
    angles_ax.set_xlim([min_t, max_t])
    angles_ax.legend()
    angles_ax.set_yscale('log')
    if noise_levels:
        angles_ax.set_xscale('log')
    angles_fig.tight_layout()
    angles_fig.savefig(f'monte_carlo_angles{"_noise" if noise_levels else ""}.png')

    plt.show()


if __name__ == '__main__':
    # np.random.seed(0)

    # for i in range(5):
    # generate_output(10, 10, True)

    generate_monte_carlo()

    # plot_monete_carlo()

    plot_violins()

    # generate_output(10, 10, True, random_scale=1)
