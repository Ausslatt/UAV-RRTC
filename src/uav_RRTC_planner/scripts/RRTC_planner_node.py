#!/usr/bin/env python2

import math
import random
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Transform, Vector3
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint
from tf.transformations import quaternion_from_euler, euler_from_quaternion

# ((xmin, xmax), (ymin, ymax), (zmin, zmax))
# inflate obstacles by PAD to compensate MAV dimensions.
X_PAD = 1.0
Y_PAD = 1.0
Z_PAD = 1.0

OBSTACLES = [
    #  walls
    
    ((-20.0- X_PAD, -3.0 + X_PAD),    (-11.0 - Y_PAD, -5.0 + Y_PAD),    (0 - Z_PAD, 7.0 + Z_PAD)),   # wall_1
    ((4.0 - X_PAD, 10.0 + X_PAD),       (-39.5 - Y_PAD, -4.5 + Y_PAD),    (0 - Z_PAD, 7.0 + Z_PAD)),   # wall_2
    ((10.0 - X_PAD, 40.0 + X_PAD),       (-5.5 - Y_PAD, 3.5 + Y_PAD),    (0 - Z_PAD, 2.0 + Z_PAD)),   # wall_3
    #  towers
    ((-7.5 - X_PAD, -4.5 + X_PAD),     (1.0 - Y_PAD, 4.0 + Y_PAD),       (0 - Z_PAD, 3.0 + Z_PAD)),   # tower_0
    ((5.5 - X_PAD, 8.5 + X_PAD),      (12.5 - Y_PAD, 15.5 + Y_PAD),       (0.0 - Z_PAD, 7.5 + Z_PAD)),   # tower_1
    ((5.5 - X_PAD, 8.5 + X_PAD),     (3.5 - Y_PAD, 6.5 + Y_PAD),       (0.0 - Z_PAD, 4.0 + Z_PAD)),   # tower_3
    
    # mud patch near goal
    ((-20.1, -12.1),   (-26.0, -16.0),   (-0.1, 0.1)),   # mud_box

    # grey walls
    ((-19.75 - X_PAD, -12.25 + X_PAD), (-16.1 - Y_PAD, -15.9 + Y_PAD ),   (0.0 - Z_PAD, 2.8 + Z_PAD)),    # grey_wall
    ((-15.9 - X_PAD, -8.4 + X_PAD),    (-19.8 - Y_PAD, -19.6 + Y_PAD),   (0.0 - Z_PAD, 2.8 + Z_PAD)),    # grey_wall_0
    ((15.9 - X_PAD, -8.4 + X_PAD),   (-19.8 - Y_PAD, -19.6 + Y_PAD),   (0.0 - Z_PAD, 2.8 + Z_PAD)),    # grey_wall_0
]

# Sampling bounds 
SAMPLE_BOUNDS = {
    'min_x': -20.0,
    'max_x':  40.0,
    'min_y': -40.0,
    'max_y':  20.0,
    'min_z':   0.1,
    'max_z':   15.0,
}


class Vertex(object):
    def __init__(self, x, y, z, yaw, parent=None):
	# a vertex of our tree
        self.x = x
        self.y = y
        self.z = z
        self.yaw = yaw
        self.parent = parent


def vertex_position(v):
    return v.x, v.y, v.z


def calc_dist(v, w):
    dx = v.x - w.x
    dy = v.y - w.y
    dz = v.z - w.z
    return math.sqrt(dx * dx + dy * dy + dz * dz)


def state_in_obstacle(x, y, z):
    """
    Check if the point (x, y, z) is inside any obstacle.
    """
    for (xrange, yrange, zrange) in OBSTACLES:
        xmin, xmax = xrange
        ymin, ymax = yrange
        zmin, zmax = zrange

        if (x >= xmin and x <= xmax and
            y >= ymin and y <= ymax and
            z >= zmin and z <= zmax):
            return True

    return False


def collision_free(v, w, step=0.10):
    """
    Check if the straight-line edge between vertices v and w is collision free.
    """
    x1, y1, z1 = vertex_position(v)
    x2, y2, z2 = vertex_position(w)

    # quick reject if endpoints are inside obstacles
    if state_in_obstacle(x1, y1, z1):
        return False
    if state_in_obstacle(x2, y2, z2):
        return False

    dx = x2 - x1
    dy = y2 - y1
    dz = z2 - z1

    dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    if dist == 0.0:
        return True

    steps = int(dist / step)
    if steps < 1:
        steps = 1

    for i in range(1, steps):
        t = float(i) / float(steps)
        x = x1 + t * dx
        y = y1 + t * dy
        z = z1 + t * dz
        if state_in_obstacle(x, y, z):
            return False

    return True


class RRTTree(object):
    """
    A tree is a root and list of vertices
    """
    def __init__(self, root):
        self.root = root
        self.vertices = [root]


class RRTConnectPlanner(object):
    """
    3D RRT-Connect planner with obstacles defined via OBSTACLES.

    """

    def __init__(self, start, goal,
                 max_iters=1000,
                 step_size=1.0,
                 goal_tolerance=0.5,
                 goal_bias=0.2):
        self.start = start
        self.goal = goal
        self.max_iters = max_iters
        self.step_size = step_size
        self.goal_tolerance = goal_tolerance
        self.goal_bias = goal_bias

    def plan(self):
        """
        Returns: list of (x, y, z, yaw) waypoints from start to goal,
                 or [] if no path was found.
        """
        T_start = RRTTree(self.start)
        T_goal = RRTTree(self.goal)

        T_a = T_start
        T_b = T_goal

        for k in range(self.max_iters):
            # goal-biased sampling
            if random.random() < self.goal_bias:
                q_rand = self.goal
            else:
                q_rand = self.sample_free()

            q_new, status = self.extend(T_a, q_rand)

            if status != 'trapped':
                q_connect, connected = self.connect(T_b, q_new)

                if connected:
                    # Figure out which side is start vs goal for path reconstruction
                    if T_a is T_start:
                        path_vertices = self.build_path(T_start, T_goal,
                                                        from_start_side=q_new,
                                                        from_goal_side=q_connect)
                    else:
                        # Trees are swapped
                        path_vertices = self.build_path(T_goal, T_start,
                                                        from_start_side=q_connect,
                                                        from_goal_side=q_new)

                    # Convert vertices to (x,y,z,yaw)
                    path = [(v.x, v.y, v.z, v.yaw) for v in path_vertices]
                    return path

            # swap the trees
            T_a, T_b = T_b, T_a

        # no path found
        return []

    def sample_free(self):
        """
        Sample a collision-free state inside sampling bounds.
        """
        while True:
            x = random.uniform(SAMPLE_BOUNDS['min_x'], SAMPLE_BOUNDS['max_x'])
            y = random.uniform(SAMPLE_BOUNDS['min_y'], SAMPLE_BOUNDS['max_y'])
            z = random.uniform(SAMPLE_BOUNDS['min_z'], SAMPLE_BOUNDS['max_z'])
            if state_in_obstacle(x, y, z):
                continue

            yaw = random.uniform(-math.pi, math.pi)
            return Vertex(x, y, z, yaw)

    def nearest(self, tree, q_target):
        """
        Find nearest vertex in 'tree' to q_target.
        """
        best = None
        best_dist = float('inf')
        for v in tree.vertices:
            d = calc_dist(v, q_target)
            if d < best_dist:
                best = v
                best_dist = d
        return best

    def steer(self, q_from, q_to):

        dx = q_to.x - q_from.x
        dy = q_to.y - q_from.y
        dz = q_to.z - q_from.z

        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        if dist <= self.step_size and dist > 0.0:
            # go all the way to q_to
            x = q_to.x
            y = q_to.y
            z = q_to.z
        elif dist == 0.0:
            x, y, z = q_from.x, q_from.y, q_from.z
        else:
            r = self.step_size / dist
            x = q_from.x + r * dx
            y = q_from.y + r * dy
            z = q_from.z + r * dz

        # yaw isn't important for planning (trajectory uses constant yaw),
        # keep q_from's yaw for continuity.
        return Vertex(x, y, z, q_from.yaw)

    def extend(self, tree, q_target):
        """
        Try to grow 'tree' towards q_target by one step.
        Returns (q_new, status) where status is:
          'advanced', 'reached', or 'trapped'.
        """
        q_near = self.nearest(tree, q_target)
        if q_near is None:
            return None, 'trapped'

        q_new = self.steer(q_near, q_target)

        if not collision_free(q_near, q_new):
            return None, 'trapped'

        q_new.parent = q_near
        tree.vertices.append(q_new)

        d = calc_dist(q_new, q_target)
        if d <= self.goal_tolerance:
            return q_new, 'reached'
        else:
            return q_new, 'advanced'

    def connect(self, tree, q_target):
        """
        Keep extending 'tree' towards q_target until trapped or reached.
        Returns (last_added_vertex, connected_bool).
        """
        q_new = None
        while True:
            q_new, status = self.extend(tree, q_target)
            if status == 'trapped':
                return q_new, False
            if status == 'reached':
                return q_new, True
                
    def build_path(self, start_tree, goal_tree, from_start_side, from_goal_side):
        """
        Reconstruct a full path from start_tree.root to goal_tree.root
        through the connection vertices.
        """
        # path from start root to connection (in start_tree)
        path_start = []
        v = from_start_side
        while v is not None:
            path_start.append(v)
            v = v.parent
        path_start.reverse()  # [start_root ... from_start_side]

        # path from connection (in goal_tree) to goal root
        path_goal = []
        v = from_goal_side
        while v is not None:
            path_goal.append(v)
            v = v.parent
        # path_goal is [from_goal_side ... goal_root], which is
        # the direction we actually want

        # Avoid duplicating the connection if they are effectively the same point
        if path_start and path_goal:
            v_s = path_start[-1]
            v_g = path_goal[0]
            if calc_dist(v_s, v_g) < 1e-3:
                path_goal = path_goal[1:]

        return path_start + path_goal


class RRTCPlannerNode(object):


    def __init__(self):
        rospy.init_node('RRTC_planner')

        self.mav_name = rospy.get_param('~mav_name', 'firefly')
        self.world_frame = rospy.get_param('~world_frame', 'world')
	
	# this is overridden
        self.goal_x = rospy.get_param('~goal_x', -5.0)
        self.goal_y = rospy.get_param('~goal_y', -30.0)
        self.goal_z = rospy.get_param('~goal_z', 1.0)
        self.goal_yaw = rospy.get_param('~goal_yaw', 0.0)

        # Reuse parameters but interpret them for RRT-Connect
        self.n_iters = rospy.get_param('~n_iters', 2000)        # max iterations
        self.rrt_step_size = rospy.get_param('~step_size', 1.0)
        self.rrt_goal_tol = rospy.get_param('~goal_tolerance', 1.0)

        self.current_pose = None
        self.path_sent = False

        # Ground-truth odom from RotorS
        odom_topic = '/{}/ground_truth/odometry'.format(self.mav_name)
        rospy.Subscriber(odom_topic, Odometry, self.odom_cb)

        traj_topic = '/{}/command/trajectory'.format(self.mav_name)
        self.traj_pub = rospy.Publisher(traj_topic,
                                        MultiDOFJointTrajectory,
                                        queue_size=1,
                                        latch=True)

        rospy.loginfo("RRT-Connect planner waiting for odometry on %s", odom_topic)

    def densify_path(self, path, max_seg_len=1.0):

        if not path or len(path) < 2:
            return path

        const_yaw = path[0][3]

        dense = []
        x_prev, y_prev, z_prev, _ = path[0]
        dense.append((x_prev, y_prev, z_prev, const_yaw))

        for i in range(1, len(path)):
            x, y, z, _ = path[i]
            dx = x - x_prev
            dy = y - y_prev
            dz = z - z_prev
            dist = math.sqrt(dx*dx + dy*dy + dz*dz)

            steps = max(1, int(dist / max_seg_len))

            for s in range(1, steps + 1):
                t = float(s) / float(steps)
                xi = x_prev + t * dx
                yi = y_prev + t * dy
                zi = z_prev + t * dz
                dense.append((xi, yi, zi, const_yaw))

            x_prev, y_prev, z_prev = x, y, z

        return dense

    def odom_cb(self, msg):
        self.current_pose = msg.pose.pose
        if not self.path_sent:
            self.plan_and_send()

    def plan_and_send(self):
        if self.current_pose is None:
            rospy.logwarn("No odometry yet, cannot plan.")
            return

        p = self.current_pose.position
        q = self.current_pose.orientation

        roll, pitch, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

        start = Vertex(p.x, p.y, p.z, yaw)
        goal = Vertex(self.goal_x, self.goal_y, self.goal_z, self.goal_yaw)

        planner = RRTConnectPlanner(start, goal,
                                    max_iters=self.n_iters,
                                    step_size=self.rrt_step_size,
                                    goal_tolerance=self.rrt_goal_tol,
                                    goal_bias=0.2)

        rospy.loginfo("RRT-Connect planning from (%.2f, %.2f, %.2f, yaw=%.2f) to "
                      "(%.2f, %.2f, %.2f, yaw=%.2f)",
                      p.x, p.y, p.z, yaw,
                      self.goal_x, self.goal_y, self.goal_z, self.goal_yaw)

        path = planner.plan()

        rospy.loginfo("RRT-Connect planned path with %d waypoints", len(path) if path else 0)

        if not path or len(path) < 2:
            rospy.logwarn("RRT-Connect did not find a valid path")
            return

        traj_msg = self.path_to_trajectory(path)
        self.traj_pub.publish(traj_msg)
        self.path_sent = True
        rospy.loginfo("Published trajectory with %d points", len(traj_msg.points))

    def path_to_trajectory(self, path):
        """
        path: list of (x, y, z, yaw)
        Turn the RRT-Connect path into a slow, smooth MultiDOFJointTrajectory
        that the RotorS Lee controller can actually track.
        """
        traj = MultiDOFJointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.header.frame_id = self.world_frame
        traj.joint_names = ['base_link']

        if not path or len(path) < 2:
            return traj

        # 1) Densify path and keep yaw constant
        path = self.densify_path(path, max_seg_len=1.0)

        v_des = 1.0      # desired translational speed (m/s)
        min_dt = 0.2     # minimum time between points [s]

        time_from_start = rospy.Duration(0.0)
        prev_xyz = None

        for i, (x, y, z, yaw) in enumerate(path):
            trans = Transform()
            trans.translation = Vector3(x, y, z)

            qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, yaw)
            trans.rotation.x = qx
            trans.rotation.y = qy
            trans.rotation.z = qz
            trans.rotation.w = qw

            pt = MultiDOFJointTrajectoryPoint()
            pt.transforms.append(trans)

            if i == 0:
                time_from_start = rospy.Duration(0.0)
            else:
                dx = x - prev_xyz[0]
                dy = y - prev_xyz[1]
                dz = z - prev_xyz[2]
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)

                if v_des > 0.0:
                    dt = dist / v_des
                else:
                    dt = min_dt

                if dt < min_dt:
                    dt = min_dt

                time_from_start += rospy.Duration(dt)

            pt.time_from_start = time_from_start
            traj.points.append(pt)

            prev_xyz = (x, y, z)

        # 2) Add a  hover at the goal 
        if traj.points:
            last = traj.points[-1]
            hover = MultiDOFJointTrajectoryPoint()
            hover.transforms.extend(last.transforms)
            hover.time_from_start = time_from_start + rospy.Duration(3.0)
            traj.points.append(hover)

        return traj


if __name__ == "__main__":
    node = RRTCPlannerNode()
    rospy.spin()
