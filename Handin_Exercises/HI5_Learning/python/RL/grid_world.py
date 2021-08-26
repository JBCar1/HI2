import numpy as np
import matplotlib.pyplot as plt


def p_grid_world(s, a, params):
    """Return list of triples (next_state, reward, probability) for a given state s and action a.

    Input arguments:
    s -- state (row, col)
    a -- action ({left, right, up, down} = {0, 1, 2, 3})
    """

    def state_feasible(s, params):
        """Function for checking if a state is feasible (i.e., not outside of 
        grid world or an obstacle)"""
        occ_grid = params["occ_grid"]
        n_cols = params['n_cols']
        n_rows = params['n_rows']
        return (0 <= s[0] < n_rows) and (0 <= s[1] < n_cols) and occ_grid[s[0], s[1]] == 0
    
    R = params["R"]
    row, col = s

    P_dist = params['P_dist'] 
    P_move_action = params['P_move_action']

    if a == 0:  # Move left
        s_0 = (row, col - 1)
        s_1 = (row - 1, col)  # Accidental up
        s_2 = (row + 1, col)  # Accidental down
    if a == 1:  # Move right
        s_0 = (row, col + 1)
        s_1 = (row - 1, col)  # Accidental up
        s_2 = (row + 1, col)  # Accidental down
    if a == 2:  # Move up
        s_0 = (row - 1, col)
        s_1 = (row, col - 1)  # Accidental left
        s_2 = (row, col + 1)  # Accidental right
    if a == 3:  # Move down
        s_0 = (row + 1, col)
        s_1 = (row, col - 1)  # Accidental left
        s_2 = (row, col + 1)  # Accidental right

    # If the next state happens to be non-feasible, the agent stays in the
    # current state s

    if not state_feasible(s_0, params):
        s_0 = s

    if not state_feasible(s_1, params):
        s_1 = s

    if not state_feasible(s_2, params):
        s_2 = s

    p = np.array([P_move_action, P_dist, P_dist])

    r = [R[s_0[0], s_0[1]],
         R[s_1[0], s_1[1]],
         R[s_2[0], s_2[1]]]

    return [(s_0, r[0], p[0]),
            (s_1, r[1], p[1]),
            (s_2, r[2], p[2])]


def plot_value_and_policy(V, Pi, params):
    """Plot current iteration of the value function V and policy Pi"""
    
    n_rows = params['n_rows']
    n_cols = params['n_cols']
    occ_grid = params['occ_grid']
    R = params['R']

    goal = params['goal']
    sink = params['sink']

    actions = ['left', 'right', 'up', 'down']

    fig1 = plt.figure(1, clear=True)
    for row in range(n_rows):
        for col in range(n_cols):
            if occ_grid[row, col] == 1:
                plt.text(col, n_rows - 1 - row, '0.0', color='k', ha='center', va='center')
            elif (row, col) in sink:
                plt.text(col, n_rows - 1 - row, f"{R[row, col]:.3f}",
                         color='r', ha='center', va='center')
            elif (row, col) == goal:
                plt.text(col, n_rows - 1 - row, f"{R[row, col]:.3f}",
                         color='g', ha='center', va='center')
            else:
                plt.text(col, n_rows - 1 - row, f"{V[row, col]:.3f}",
                         color='b', ha='center', va='center')
    plt.axis([-1, n_cols, -1, n_rows])
    plt.axis('off')

    fig2 = plt.figure(2, clear=True)
    for row in range(n_rows):
        for col in range(n_cols):
            if not Pi[row, col] == -1:
                plt.text(col, n_rows - 1 - row, actions[Pi[row, col]],
                         color='k', ha='center', va='center')
            elif (row, col) == goal:
                plt.text(col, n_rows - 1 - row, f"{R[row, col]:.3f}",
                         color='g', ha='center', va='center')
            elif (row, col) in sink:
                plt.text(col, n_rows - 1 - row, f"{R[row, col]:.3f}",
                         color='r', ha='center', va='center')
    plt.axis([-1, n_cols, -1, n_rows])
    plt.axis('off')

    fig1.canvas.draw()
    fig1.canvas.flush_events()
    fig2.canvas.draw()
    fig2.canvas.flush_events()