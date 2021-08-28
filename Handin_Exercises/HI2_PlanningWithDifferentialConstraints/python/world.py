"""Class for describing a world with given boundaries and box-shaped
    obstacles."""
import numpy as np
import matplotlib.pyplot as plt


class BoxWorld:
    def __init__(self, lattice):
        """Create a BoxWorld object with the given lattice"""
        self.st_sp = state_space_from_lattice(lattice)
        self.xmin = np.min(lattice[0])
        self.xmax = np.max(lattice[0])
        self.ymin = np.min(lattice[1])
        self.ymax = np.max(lattice[1])

        self._fig = None
        self._boxes = []
        self.x_obst = np.array([]).reshape((0, 2))
        self.y_obst = np.array([]).reshape((0, 2))

    def num_nodes(self):
        """Get the total number of nodes in the state space"""
        return self.st_sp.shape[1]

    def add_box(self, x, y, width, height, fill_box=True):
        """ Add a box to the world

        Input
            x - x coordinate of the lower left corner
            y - y coordinate of the lower left corner
            width - width of the box
            height - height of the box
        """
        self._boxes.append((x, y, width, height, fill_box))
        self.x_obst = np.row_stack((self.x_obst, [x, x + width]))
        self.y_obst = np.row_stack((self.y_obst, [y, y + height]))

    def draw_box(self, b, *args, **kwargs):
        """Help function to function draw, for drawing a box in the figure"""
        x0, y0, W1, W2, fill_box = b

        if fill_box:
            plt.fill([x0, x0 + W1, x0 + W1, x0, x0],
                     [y0, y0, y0 + W2, y0 + W2, y0], *args, **kwargs)
        else:
            plt.plot([x0, x0 + W1, x0 + W1, x0, x0],
                     [y0, y0, y0 + W2, y0 + W2, y0], *args, **kwargs)

    def register_figure(self, fig):
        self._fig = fig

    def draw(self, *args, **kwargs):
        """Draw the obstacles in the world in the figure"""
        if len(args) == 0:
            args = ['r']
        if len(kwargs) == 0:
            kwargs = {'edgecolor': 'k'}

        self.redraw_boxes(*args, **kwargs)

    def redraw_boxes(self, *args, **kwargs):
        for bi in self._boxes:
            self.draw_box(bi, *args, **kwargs)
        if self._fig:
            self._fig.canvas.draw()
            self._fig.canvas.flush_events()

    def in_bound(self, point):
        """Check if a given point is within the world-model boundaries"""
        c = False
        if (point[0] >= self.xmin) and (point[0] <= self.xmax) and (point[1] >= self.ymin) and (point[1] <= self.ymax):
            c = True
        return c

    def obstacle_free(self, p):
        """ Check if any of a set of points are in collision with obstacles in the world

        Input
          p - numpy array with 2 rows and m columns, where each column represents a point to be checked

        Output
          Returns True if all points are in free space, otherwise False.
        """
        for ii in range(p.shape[1]):
            if obstacle_check(p[0, ii], p[1, ii], self.x_obst, self.y_obst):
                return False
        return True


def state_space_from_lattice(lattice):
    """Create a matrix st_sp with all states in the world, given the 
        specified lattice parameters. In the lattice planning, this 3 x N
        matrix is used as a mapping between node number and actual
        coordinates, where the column number is the node number."""
    if len(lattice) == 1:
        st_sp = np.array(lattice[0]).reshape((1, -1))
    else:
        st_sp_1 = state_space_from_lattice(lattice[1:])
        N = st_sp_1.shape[1]
        st_sp = np.array([]).reshape((st_sp_1.shape[0] + 1, 0))
        for xi in lattice[0]:
            st_sp = np.hstack((st_sp,
                               np.row_stack((np.full((1, N), xi),
                                             st_sp_1))))
    return st_sp


def obstacle_check(x, y, x_obst, y_obst):
    """Help function to function obstacle_free, to check collision for a 
        single point x,y"""
    for ii in range(x_obst.shape[0]):
        if (x > x_obst[ii, 0] and x < x_obst[ii, 1]) and \
           (y > y_obst[ii, 0] and y < y_obst[ii, 1]):
            return True
    return False
