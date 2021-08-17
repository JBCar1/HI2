"""Support files for implementing and simulating controllers
for simple vehicles"""
from abc import ABC, abstractmethod
import numpy as np
import matplotlib.pyplot as plt
# from scipy.interpolate import interp1d
# from scipy.optimize import brenth
# import warnings
import time


class Timer:
    """Simple timer class with tic/toc functionality"""
    t0 = 0.0
    dt = 0.0

    def tic(self):
        """Start timing"""
        self.t0 = time.time()

    def toc(self):
        """Return time since last toc"""
        self.dt = time.time() - self.t0
        return self.dt


class EulerForward:
    """Simple Euler forward integrator class"""
    t = 0
    y = 0
    Ts = -1

    def __init__(self, f):
        self.f = f

    def set_Ts(self, Ts):
        self.Ts = Ts
        return self

    def set_initial_value(self, y, t):
        self.t = t
        self.y = np.array(y)

    def integrate(self, Tend):
        while self.t < Tend:
            if self.t + self.Ts < Tend:
                dt = self.Ts
            else:
                dt = Tend - self.t

            self.y = self.y + self.f(self.t, self.y) * dt
            self.t = self.t + dt

    def successful(self):
        return True


class RungeKutta:
    """Simple Runge-Kutta integrator class"""
    t = 0
    y = 0
    Ts = -1

    def __init__(self, f):
        self.f = f

    def set_Ts(self, Ts):
        self.Ts = Ts
        return self

    def set_initial_value(self, y, t):
        self.t = t
        self.y = y

    def integrate(self, Tend):
        while self.t < Tend:
            if self.t + self.Ts < Tend:
                dt = self.Ts
            else:
                dt = Tend - self.t

            k1 = self.f(self.t, self.y)
            k2 = self.f(self.t, self.y + dt / 2 * k1)
            k3 = self.f(self.t, self.y + dt / 2 * k2)
            k4 = self.f(self.t, self.y + dt / 3 * k3)

            self.y = self.y + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4)
            self.t = self.t + dt

    def successful(self):
        return True


class VehicleModelBase(ABC):
    """Abstract base class for vehicle models

    Derived classes must implement the method
        dw/dt = dx(w, u)
    where w is the state and u the control inputs. Defined in the class as
        def dx(self, w, u)
    and returns a numpy array with the time derivatives of the states.
    """

    _u = None
    _controller = None

    def __init__(self, Ts):
        # self.integrator = ode(lambda t, w: self.dx(t, w, self._u)).set_integrator('vode', method='bdf')
        # self.integrator = EulerForward(lambda t, w: self.dx(t, w, self._u)).set_Ts(Ts)
        self.integrator = RungeKutta(lambda t, w: self.dx(t, w, self._u)).set_Ts(Ts)

    @property
    def Ts(self):
        return self.integrator.Ts

    @Ts.setter
    def Ts(self, Ts):
        self.integrator.set_Ts(Ts)

    def set_attributes(self, opts):
        for ki in opts:
            setattr(self, ki, opts[ki])
        return self

    def set_state(self, w0, t0=0.0):
        self.integrator.set_initial_value(w0, t0)

    @abstractmethod
    def dx(self, t, w, u):
        pass

    @property
    def controller(self):
        return self._controller

    @controller.setter
    def controller(self, controller):
        if not isinstance(controller, ControllerBase):
            raise RuntimeError(
                'Controller has to be derived from class ControllerBase')
        self._controller = controller

    def simulate_step(self, Tend):
        self.integrator.integrate(Tend)
        return self.integrator.successful()

    def get_state(self):
        """Return current time and state for the vehicle."""
        return self.integrator.t, self.integrator.y

    @property
    def t(self):
        """Returns current time."""
        return self.integrator.t

    @property
    def w(self):
        """Returns vehicle state."""
        return self.integrator.y

    def simulate(self, w0, T, dt, t0=0.0):
        """Simulate vehicle and controller.

        Simulates the vehicle and controller until specified stop time
        or when the controller stops, i.e., when the controller method run
        returns false.

        Inputs:
          w0 - Initial state
          T  - Stop time
          dt - Controller sampling period
          t0 - Start time (default: 0.0)

        Outputs:
          t - Simulation time vector (sampled by dt)
          w - state trajectories
          u - control signals
        """
        if not isinstance(self._controller, ControllerBase):
            raise RuntimeError('Controller has to be set before simulating, call set_controller first.')

        self._controller.u_time = 0.0

        self.set_state(w0, t0)
        sim = []
        t = []
        u = []
        successful = True
        while ((successful and self.t < T) and self._controller.run(self.t, self.w)):
            self._u = self._controller.u_timed(self.t, self.w)
            u.append(self._u)
            sim.append(self.w)
            t.append(self.t)
            successful = self.simulate_step(self.t + dt)

        u.append(self._controller.u(self.t, self.w))
        sim.append(self.w)
        t.append(self.t)

        return np.array(t), np.array(sim), np.array(u)

    # def viz_simulate(self, fig, w0, T, dt, t0=0.0, sleep_time=-1):
    #     """Simulate and animate vehicle and controller.
    #
    #     Simulates the vehicle and controller until specified stop time
    #     or when the controller stops, i.e., when the controller method run
    #     returns false.
    #
    #     Inputs:
    #     fig - handle to animation window
    #     w0 - Initial state
    #     T  - Stop time
    #     dt - Controller sampling period
    #     t0 - Start time (default: 0.0)
    #     sleep_time - Time in seconds to wait after each control event to slow down simulation
    #
    #     Outputs:
    #     t - Simulation time vector (sampled by dt)
    #     w - state trajectories
    #     u - control signals
    #     """
    #     if not isinstance(self._controller, ControllerBase):
    #         raise RuntimeError('Controller has to be set before simulating, ' +
    #                         'call set_controller first.')
    #
    #     self._controller.u_time = 0.0
    #
    #     self.set_state(w0, t0)
    #     sim = []
    #     t = []
    #     u = []
    #     successful = True
    #     car_pos = plt.plot(w0[0], w0[1], 'ro')
    #     t_last = time.time()
    #     t_0 = t_last
    #     while successful and self.t < T and self._controller.run(self.t, self.w):
    #         self._u = self._controller.u_timed(self.t, self.w)
    #         u.append(self._u)
    #         sim.append(self.w)
    #         t.append(self.t)
    #         successful = self.simulate_step(self.t + dt)
    #         car_pos[0].set_data([self.w[0], self.w[1]])
    #         if fig.number not in plt.get_fignums():
    #             print('Figure closed, exiting')
    #             return
    #         fig.canvas.draw()
    #         fig.canvas.flush_events()
    #         if sleep_time > 0:
    #             time.sleep(sleep_time)
    #
    #     u.append(self._controller.u(self.t, self.w))
    #     sim.append(self.w)
    #     t.append(self.t)
    #
    #     return (np.array(t), np.array(sim), np.array(u))


class SingleTrackModel(VehicleModelBase):
    L = 2
    amax = np.inf
    amin = -np.inf
    steer_limit = np.pi / 3

    def __init__(self, Ts=0.1):
        """ Create Single track kinematic car

             obj = SingleTrackModel(Ts)
             Ts - controller sample time"""
        super(SingleTrackModel, self).__init__(Ts)

    def dx(self, t, w, u):
        """Dynamic equation for single-track model

           dw = obj.dx(w, u)

           Input:
             w  - state (x, y, theta, v)
             u - control input (delta, acceleration)

           Output:
             dw - dw/dt"""
        x, y, theta, v = w
        delta = np.max((np.min((u[0], self.steer_limit)), -self.steer_limit))
        a = np.max((np.min((u[1], self.amax)), self.amin))

        dx = np.array([v * np.cos(theta),
                       v * np.sin(theta),
                       v / self.L * np.tan(delta),
                       a])
        return dx


class ControllerBase(ABC):
    def __init__(self):
        self.u_time = 0.0
        self.u_timer = Timer()

    def u_timed(self, t, w):
        self.u_timer.tic()
        u = self.u(t, w)
        self.u_time = self.u_time + self.u_timer.toc()
        return u

    @abstractmethod
    def u(self, t, w):
        pass

    def run(self, _t, _w):
        return True


# def SaveVehicleMovie(t, w, p, fps, speedup=1, filename=None):
#     def movie_init():
#         plt.plot(p[:, 0], p[:, 1], 'b', lw=0.5)
#         return car_pos, t_text, v_text
#
#     def movie_update(frame):
#         car_pos.set_data(px[frame], py[frame])
#         t_text.set_text('t = {:.1f} s'.format(ts[frame]))
#         v_text.set_text('v = {:.1f} m/s'.format(pv[frame]))
#         return car_pos, t_text, v_text
#
#     N = int(t[-1]*fps/speedup)
#     ts = np.linspace(0, t[-1], N)
#     px = np.interp(ts, t, w[:, 0])
#     py = np.interp(ts, t, w[:, 1])
#     pv = np.interp(ts, t, w[:, 3])
#
#     fig, ax = plt.subplots()
#     # fig = plt.figure()
#
#     car_pos, = plt.plot([], [], 'ro', animated=True)
#     t_text = plt.text(0, 20, 't = 0.0 s')
#     v_text = plt.text(0, 18, 'v = 0.0 m/s')
#
#     ani = animation.FuncAnimation(fig, movie_update, frames=np.arange(0, N),
#                                   init_func=movie_init, blit=True)
#     if filename is not None:
#         Writer = animation.writers['ffmpeg']
#         writer = Writer(fps=fps, bitrate=1800)
#         ani.save(filename, writer=writer)
#     else:
#         print('No file is saved, animation on screen ...')
#         plt.show()


def plot_car(w, W, L, *args):
    x, y, theta, _ = w
    p = np.array([x, y])
    heading = np.array([np.cos(theta), np.sin(theta)])
    nc = np.array([-heading[1], heading[0]])

    p_car = np.array([p - W / 2 * nc, p - W / 2 * nc + L * heading,
                      p + W / 2 * nc + L * heading, p + W / 2 * nc, p - W / 2 * nc])
    plt.plot(p_car[:, 0], p_car[:, 1], *args)
