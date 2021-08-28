"""Class for motion primitives, computed given a certain pre-defined 
    state lattice, or loaded from file if already computed."""
import numpy as np
from misc import loadmat
from os.path import splitext
import pickle
import matplotlib.pyplot as plt
import casadi as ca
from copy import deepcopy


class MotionPrimitives:
    def __init__(self, filename=''):
        """Motion primitves class
        
        If a filename is provided, saved motion primitives are loaded.
        """
        if len(filename) > 0:
            self.load(filename)

    def load(self, filename):
        """Load the motion primitives from file if they have been
            computed already"""
        ext = splitext(filename)[-1]
        if ext == '.mat':
            mprims = loadmat(filename)['mprims']

            self.mprims = []
            for i in range(len(mprims)):
                mi = []
                for j in range(len(mprims[0])):
                    mi_element = {'x': mprims[i][j].x, 'y': mprims[i][j].y,
                                  'u': mprims[i][j].u, 'th': mprims[i][j].th,
                                  'T': mprims[i][j].T, 'ds': mprims[i][j].ds}
                    mi.append(mi_element)
                self.mprims.append(mi)
            self.mprims = np.array(self.mprims)

            self.th = np.array([mi[0].th[0] for mi in mprims])
        elif ext == '.pickle':
            with open(filename, 'rb') as f:
                (self.mprims, self.th) = pickle.load(f)
        else:
            raise Exception('Unknown file type, only .mat and .pickle supported')

    def with_start_orientation_index(self, theta):
        """Iterator for index to motion primitives with a given starting orientation"""
        tol = 1e-5
        k = np.argwhere(np.abs((theta - self.th) % (2 * np.pi)) < tol)[0][0]
        return iter([(k, j) for j in range(len(self.mprims[k]))])

    def with_end_orientation_index(self, theta):
        """Iterator for index to motion primitives with a given end orientation"""
        tol = 1e-5
        for i, m_i in enumerate(self.mprims):
            for j, m_ij in enumerate(m_i):
                if np.abs((m_ij["th"][-1] - theta) % (2 * np.pi)) < tol:
                    yield (i, j)

    def with_start_orientation(self, theta):
        """Iterator for motion primitives with a given starting orientation"""
        tol = 1e-5
        k = np.argwhere(np.abs((theta - self.th) % (2 * np.pi)) < tol)[0][0]
        return iter(self.mprims[k])

    def with_end_orientation(self, theta):
        """Iterator for motion primitives with a given end orientation"""
        tol = 1e-5
        for m_i in self.mprims:
            for m_ij in m_i:
                if np.abs((m_ij["th"][-1] - theta) % (2 * np.pi)) < tol:
                    yield m_ij

    def save(self, filename):
        """Save the motion primitives in a file to avoid re-computing
            them next time they are used"""
        with open(filename, 'wb') as f:
            pickle.dump((self.mprims, self.th), f)

    def plot(self, *args, **kwargs):
        """Plot motion primitives"""
        for mp in self.mprims:
            for mpi in mp:
                plt.plot(mpi['x'], mpi['y'], *args, **kwargs)

    def plan_to_path(self, x0, plan):
        """Convert a plan to a path
        Compute a path p and direction information for a plan. 
        
        arguments:
            x0 -- Initial state (index)
            plan -- Plan

        output:
            p -- path
            sp -- The output sp describes which parts of
                the path that are driving forward and which parts that are driving
                backwards. The matrix has the form
                [[u_1, start_1, end_1],
                 [u_2, start_2, end_2],
                 ...]
                where u_i is 1 for forward and -1 for reverse. Start_i and
                end_i are indices into the path p. 
                
                For example, to plot the k:th segment in blue when driving forward 
                and red when reversing, you can write

                segment_range = range(sp[k, 1], sp[k, 2])
                plt.plot(p[0, segment_range], p[1, segment_range], "b" if sp[k, 0] == 1 else "r")
        """
        return self.control_to_path(x0, plan["control"])

    def control_to_path(self, x0, control):
        """Convert a control description to a path
        Compute a path p and direction information for a control trajectory (plan["control"]). 
        
        arguments:
            x0 -- Initial state (index)
            control -- Control actions (typically plan["control"])

        output:
            p -- path
            sp -- The output sp describes which parts of
                the path that are driving forward and which parts that are driving
                backwards. The matrix has the form
                [[u_1, start_1, end_1],
                 [u_2, start_2, end_2],
                 ...]
                where u_i is 1 for forward and -1 for reverse. Start_i and
                end_i are indices into the path p. 
                
                For example, to plot the k:th segment in blue when driving forward 
                and red when reversing, you can write

                segment_range = range(sp[k, 1], sp[k, 2])
                plt.plot(p[0, segment_range], p[1, segment_range], "b" if sp[k, 0] == 1 else "r")
        """
        p = np.array([x0])

        u_current = control[0][2]
        switch_points = []
        curr_segment = [u_current, 0]

        for ui in control:
            mpi = self.mprims[ui[0], ui[1]]
            if ui[2] == 1:  # Forward driving
                traj = np.column_stack((p[-1][0:2] + np.column_stack((mpi['x'], mpi['y'])), mpi['th']))
            else:  # Reverse driving
                traj = np.column_stack((p[-1][0:2] + np.column_stack((np.flip(mpi["x"]) - mpi["x"][-1], 
                                                                      np.flip(mpi["y"]) - mpi["y"][-1])), 
                                                                      mpi["th"]))

            if ui[2] != u_current:
                curr_segment.append(p.shape[0])
                switch_points.append(curr_segment)
                u_current = ui[2]
                curr_segment = [u_current, p.shape[0]]
            p = np.vstack((p, traj))
 
        curr_segment.append(p.shape[0])
        switch_points.append(curr_segment)

        return p, np.array(switch_points)

    def generate_primitives(self, theta_init, state_0, L=1.5, v=15,
                            u_max=np.pi / 4, print_level=3):
        """Generate set of motion primitives using optimization
        
        arguments:
            theta_init -- List of orientations in the lattice
            state_0 -- List with state discretization
            L -- Wheel base (default: 1.5 m)
            v -- Velocity (default: 15 m/s)
            u_max -- Maximum steer angle (default: pi/4)

        Example state discretization:
            x_vec = np.array([3, 2, 3, 3, 3, 1, 3, 3, 3, 2, 3])
            y_vec = np.array([2, 2, 2, 1, 1, 0, -1, -1, -2, -2, -2])
            th_vec = np.array([0, np.pi / 4, np.pi / 2, 0, np.pi / 4, 0, -np.pi / 4, 
                                0, -np.pi / 2, -np.pi / 4, 0])
            state_0 = np.column_stack((x_vec, y_vec, th_vec))
        """
        self.mprims = compute_motion_primitives(theta_init, state_0, L, v,
                                                u_max, print_level)
        self.th = np.array([mi[0]['th'][0] for mi in self.mprims])


def compute_motion_primitives(theta_init, state_0, L, v, u_max, print_level=3):
    """Compute motion primitives with minimum path length for the 
        kinematic car model using CasADi"""
    def compute_mprim(state_i, lattice, L, v, u_max, print_level):
        """Compute the motion primitive using optimization with the tool CasADi
            using direct collocation for discretization of the continuous-time
            motion equations"""
        
        # Parameters for collocation
        N = 75; # Number of elements
        nx = 3; # Degree of state vector
        Nc = 3; # Degree of interpolation polynomials
        
        mprim = []
        
        # Formulate the optimization problem for minimum path length using CasADi
        for state_f in lattice:
            # Define optimization variables and motion equations
            x = ca.MX.sym('x', nx)
            u = ca.MX.sym('u')

            F = ca.Function('f', [x, u],
                            [v * ca.cos(x[2]), v * ca.sin(x[2]), v * ca.tan(u) / L])

            # Use the opti interface in CasADi
            opti = ca.Opti()
            X = opti.variable(nx, N + 1)
            pos_x = X[0, :]
            pos_y = X[1, :]
            ang_th = X[2, :]
            U = opti.variable(N, 1)
            T = opti.variable(1)
            # Set the element length (with final time T unknown, and thus an 
            # optimization variable)
            dt = T / N

            # Set initial guess values of variables
            opti.set_initial(T, 0.1)
            opti.set_initial(U, 0.0 * np.ones(N))
            opti.set_initial(pos_x, np.linspace(state_i[0], state_f[0], N + 1))
            opti.set_initial(pos_y, np.linspace(state_i[1], state_f[1], N + 1))

            # Define collocation parameters
            tau = ca.collocation_points(Nc, 'radau')
            C, _ = ca.collocation_interpolators(tau)

            # Formulate collocation constraints
            Xc_vec = []
            for k in range(N): # Loop over elements
                Xc = opti.variable(nx, Nc)
                Xc_vec.append(Xc)
                X_kc = ca.horzcat(X[:, k], Xc)
                for j in range(Nc):
                    # Make sure that the motion equations are satisfied at
                    # all collocation points
                    fo = F(Xc[:, j], U[k])
                    opti.subject_to(X_kc @ C[j + 1] == dt * ca.vertcat(fo[0], fo[1], fo[2]))

                # Continuity constraints for states between elements
                opti.subject_to(X_kc[:, Nc] == X[:, k + 1])

            # Input constraints
            for k in range(N):
                opti.subject_to(U[k] <= u_max)
                opti.subject_to(-u_max <= U[k])

            # Initial and terminal constraints
            opti.subject_to(T >= 0.001)
            opti.subject_to(X[:, 0] == state_i)
            opti.subject_to(X[:, -1] == state_f)

            # Formulate the cost function
            alpha = 1e-2
            opti.minimize(T + alpha * ca.sumsqr(U))

            # Choose solver ipopt and solve the problem
            opti.solver('ipopt', {'expand': True},
                        {'tol': 10**-8, 'print_level': print_level})
            sol = opti.solve()

            # Extract solution trajectories and store them in the mprim variable
            pos_x_opt = sol.value(pos_x)
            pos_y_opt = sol.value(pos_y)
            ang_th_opt = sol.value(ang_th)
            u_opt = sol.value(U)
            T_opt = sol.value(T)
            mprim.append({'x': pos_x_opt, 'y': pos_y_opt, 'th': ang_th_opt,
                          'u': u_opt, 'T': T_opt, 'ds': T_opt * v})
        return np.array(mprim)
    
    N = len(theta_init)
    dth = 2 * np.pi / N

    # Generate the motion primitives
    mprims = []
    state_i = np.array([0, 0, theta_init[0]])
    mprims.append(compute_mprim(state_i, state_0, L, v, u_max, print_level))

    # The remaining primitive sets are computed based on the two previous sets
    xy_pi_4 = np.sqrt(2) * (
        state_0[:, 0:2] @ np.array([[np.cos(dth), np.sin(dth)],
                                    [-np.sin(dth), np.cos(dth)]]))
    state_pi_4 = np.column_stack((xy_pi_4, state_0[:, 2] + np.pi / 4))
    state_i = np.array([0, 0, theta_init[1]])
    mprims.append(compute_mprim(state_i, state_pi_4, L, v, u_max, print_level))

    for i in range(2, len(theta_init)):
        if i % 2 == 1:
            # Not multiple of pi/2
            mprims.append(deepcopy(mprims[1]))
            th = dth * i - np.pi / 4
        else:
            # Multiple of pi/2
            mprims.append(deepcopy(mprims[0]))
            th = dth * i

        #  Rotate the previous set of primitives
        for j in range(len(mprims[i])):
            mp = mprims[i][j]
            xy_vec_rot = np.array(
                [[np.cos(th), -np.sin(th)],
                 [np.sin(th), np.cos(th)]]) @ np.vstack((mp['x'], mp['y']))
            mprims[i][j]['x'] = xy_vec_rot[0]
            mprims[i][j]['y'] = xy_vec_rot[1]
            mprims[i][j]['th'] += th
            # Fix orientation to interval [-pi,pi]
            mprims[i][j]['th'] -= (mprims[i][j]['th'] > np.pi) * np.pi * 2
            mprims[i][j]['th'] += (mprims[i][j]['th'] < -np.pi) * np.pi * 2

    return np.array(mprims)
