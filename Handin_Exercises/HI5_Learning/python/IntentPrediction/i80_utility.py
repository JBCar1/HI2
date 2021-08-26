import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from dataclasses import dataclass
import os


pred_to_text = {0: "LCL", 1: "LK", 2: "LCR"}


def read_trajectories(filename, convert_units=True):
    """Read I-80 trajectories."""
    def convert_units(data_conv):
        data_conv.Local_X = data_conv.Local_X / 3.28  # m
        data_conv.Local_Y = data_conv.Local_Y / 3.28  # m
        data_conv.v_Length = data_conv.v_Length / 3.28  # m
        data_conv.v_Width = data_conv.v_Width / 3.28  # m
        data_conv.v_Vel = data_conv.v_Vel * 3.6 / 3.28  # m/s
        data_conv.v_Acc = data_conv.v_Acc / 3.28  # m/s
        data_conv.Space_Headway = data_conv.Space_Headway / 3.28  # m
        data_conv.Lane_ID = data_conv.Lane_ID.astype(int) - 1

    d = pd.read_csv(filename).drop(columns=['Global_X', 'Global_Y'])
    if convert_units:
        convert_units(d)
    t_idx = np.argwhere(np.diff(d.Vehicle_ID) != 0).reshape(-1) + 1
    t_idx = np.hstack((0, t_idx, d.shape[0]))
    data = []
    for traj_no in range(len(t_idx) - 1):
        data.append(d[t_idx[traj_no]:t_idx[traj_no + 1]])
    return data


def compute_lane_model(filename="I-80-Emeryville-CA/vehicle-trajectory-data/0400pm-0415pm/trajectories-0400-0415.csv"):
    def lane_center(li):
        return th[0] + th[1] * np.array(li)
    d = pd.read_csv(filename)
    lane_x = [np.median(d[d.Lane_ID == x].Local_X) / 3.28 for x in range(1, 7)]
    th, _, _, _ = np.linalg.lstsq(np.column_stack((np.ones((6, 1)), np.arange(1, 7))), lane_x, rcond=None)
    # lane_centers = [LaneCenter(li) for li in range(1, 7)]
    lane_width = lane_center(2) - lane_center(1)
    x1 = lane_center(1) - 1 / 2 * lane_width
    comp_lane_bounds = [x1 + li * lane_width for li in range(7)]
    return comp_lane_bounds


lane_bounds = [-0.5385917537746898,
               3.231216608594652,
               7.001024970963993,
               10.770833333333336,
               14.540641695702677,
               18.310450058072018,
               22.08025842044136]


def plot_road(plotlanes=None, lb=None, center=False):
    """Plot the highway segment."""
    
    if lb is None:
        lb = [-0.5385917537746898,
              3.231216608594652,
              7.001024970963993,
              10.770833333333336,
              14.540641695702677,
              18.310450058072018,
              22.08025842044136]
    if plotlanes is None:
        plotlanes = range(6)

    y = np.array([0, 500])

    for li in range(min(plotlanes), max(plotlanes) + 1):
        plt.plot(lb[li] + 0 * y, y, 'k-')
        if center:
            plt.plot((lb[li] + lb[li + 1]) / 2 + 0 * y, y, 'k--', lw=0.5)
    plt.plot(lb[max(plotlanes) + 1] + 0 * y, y, 'k-')


def plot_prediction(pos, prediction, lane_bounds):
    """Visualize lane change prediction.
      
    Input:
    pos -- Coordinates of point of prediction
    prediction -- Array with prediction scores for each class
    lane_bounds -- array with lane boundaries      
    """
    xp, yp = pos  # Prediction position
    lane_id = np.sum(xp > lane_bounds) - 1
    plt.fill([lane_bounds[lane_id], lane_bounds[lane_id + 1], lane_bounds[lane_id + 1], lane_bounds[lane_id]],
             [yp - 5, yp - 5, yp + 5, yp + 5], color=np.array([1, 1, 1]) * (1 - prediction[1]))
    if lane_id > 0:
        plt.fill([lane_bounds[lane_id - 1], lane_bounds[lane_id], lane_bounds[lane_id], lane_bounds[lane_id - 1]],
                 [yp - 5, yp - 5, yp + 5, yp + 5], color=np.array([1, 1, 1]) * (1 - prediction[0]))
    if lane_id < 5:
        plt.fill([lane_bounds[lane_id + 1], lane_bounds[lane_id + 2], lane_bounds[lane_id + 2], lane_bounds[lane_id + 1]],
                 [yp - 5, yp - 5, yp + 5, yp + 5], color=np.array([1, 1, 1]) * (1 - prediction[2]))


@dataclass
class TrajectoryData:
    traj_index: int
    time_index: int
    vehicle_ID: int
    lane_ID: int
    x: float
    y: float
    velocity: float
    acc: float


@dataclass
class SurroundingVehicles:
    left_lag: TrajectoryData
    left_lead: TrajectoryData
    right_lag: TrajectoryData
    right_lead: TrajectoryData
    preceding: TrajectoryData
    following: TrajectoryData


def simultaneous_trajectories(ego, data):
    di_idx = ego.traj_index
    time_idx = ego.time_index

    di = data[di_idx]
    ti = di.Global_Time.iloc[time_idx]
    other_traj = [tj for tj in np.argwhere([ti in dj.Global_Time.values for dj in data]).reshape(-1)
                  if tj != di_idx]

    other_time_idx = [np.argwhere(ti == data[tj].Global_Time.values)[0][0] for tj in other_traj]
    other_vehicle_id = [data[tj].Vehicle_ID.iloc[0] for tj in other_traj]
    other_vehicle_lane = [int(data[traj_idx].Lane_ID.iloc[time_idx]) for
                          traj_idx, time_idx in zip(other_traj, other_time_idx)]
    other_vehicle_x = [data[traj_idx].Local_X.iloc[time_idx] for
                       traj_idx, time_idx in zip(other_traj, other_time_idx)]
    other_vehicle_y = [data[traj_idx].Local_Y.iloc[time_idx] for
                       traj_idx, time_idx in zip(other_traj, other_time_idx)]
    other_vehicle_vel = [data[traj_idx].v_Vel.iloc[time_idx] for
                         traj_idx, time_idx in zip(other_traj, other_time_idx)]
    other_vehicle_acc = [data[traj_idx].v_Acc.iloc[time_idx] for
                         traj_idx, time_idx in zip(other_traj, other_time_idx)]

    ret = [TrajectoryData(*z) for z in zip(other_traj, other_time_idx, other_vehicle_id,
                                           other_vehicle_lane, other_vehicle_x, other_vehicle_y,
                                           other_vehicle_vel, other_vehicle_acc)]

    return ret


def relative_lane_position(ego, bounds):
    x1 = bounds[ego.lane_ID]
    x2 = bounds[ego.lane_ID + 1]
    x = ego.x
    return ((x - x1) / (x2 - x1) - 1/2) * 2


def get_surrounding_vehicles(ego, data):
    def get_lead_lag(traj):
        lead_traj = [dt for dt in traj if dt.y > di_ti.Local_Y]
        lag_traj = [dt for dt in traj if dt.y < di_ti.Local_Y]

        if len(lead_traj) > 0:
            lead_idx = np.argmin([td.y for td in lead_traj])
            lead_traj = lead_traj[int(lead_idx)]
        else:
            lead_traj = None

        if len(lag_traj) > 0:
            lag_idx = np.argmax([td.y for td in lag_traj])
            lag_traj = lag_traj[int(lag_idx)]
        else:
            lag_traj = None
        return lead_traj, lag_traj

    di_idx = ego.traj_index
    time_idx = ego.time_index

    sim_traj = simultaneous_trajectories(ego, data)
    di_ti = data[di_idx].iloc[time_idx]

    if di_ti.Preceding == 0:
        veh_preceeding = None
    else:
        veh_preceeding = [dt for dt in sim_traj if
                          (dt.lane_ID == di_ti.Lane_ID and dt.vehicle_ID == di_ti.Preceding)]
        if len(veh_preceeding) == 0:
            veh_preceeding = None
        else:
            veh_preceeding = veh_preceeding[0]

    if di_ti.Following == 0:
        veh_following = None
    else:
        veh_following = [dt for dt in sim_traj if
                         (dt.lane_ID == di_ti.Lane_ID and dt.vehicle_ID == di_ti.Following)]
        if len(veh_following) == 0:
            veh_following = None
        else:
            veh_following = veh_following[0]

    left_lead, left_lag = get_lead_lag([dt for dt in sim_traj if dt.lane_ID == di_ti.Lane_ID - 1])
    right_lead, right_lag = get_lead_lag([dt for dt in sim_traj if dt.lane_ID == di_ti.Lane_ID + 1])

    return SurroundingVehicles(left_lag, left_lead, right_lag, right_lead, veh_preceeding, veh_following)


def impute(feature):
    default_vel = 70 / 3.6
    default_space = 500
    default_acc = 0

    if feature['time_preceding'] is np.nan:
        feature['time_preceding'] = default_space / default_vel
        feature['space_preceding'] = default_space
        feature['velocity_preceding'] = default_vel
        feature['acc_preceding'] = default_acc

    if feature['time_following'] is np.nan:
        feature['time_following'] = default_space / default_vel
        feature['space_following'] = default_space
        feature['velocity_following'] = default_vel
        feature['acc_following'] = default_acc

    if feature['time_left_lead'] is np.nan:
        feature['time_left_lead'] = default_space / default_vel
        feature['space_left_lead'] = default_space
        feature['velocity_left_lead'] = default_vel
        feature['acc_left_lead'] = default_acc

    if feature['time_left_lag'] is np.nan:
        feature['time_left_lag'] = default_space / default_vel
        feature['space_left_lag'] = default_space
        feature['velocity_left_lag'] = default_vel
        feature['acc_left_lag'] = default_acc

    if feature['time_right_lead'] is np.nan:
        feature['time_right_lead'] = default_space / default_vel
        feature['space_right_lead'] = default_space
        feature['velocity_right_lead'] = default_vel
        feature['acc_right_lead'] = default_acc

    if feature['time_right_lag'] is np.nan:
        feature['time_right_lag'] = default_space / default_vel
        feature['space_right_lag'] = default_space
        feature['velocity_right_lag'] = default_vel
        feature['acc_right_lag'] = default_acc

    for k, lane_vel in enumerate(feature['lane_mean_velocity']):
        if np.isnan(lane_vel):
            feature['lane_mean_velocity'][k] = default_vel    


def create_feature_vector(di_idx, ti_idx, pred_horizon, data, lane_bounds):
    di_ti = data[di_idx].iloc[ti_idx]

    # t_pred = np.min((ti_idx + pred_horizon, len(data[di_idx].Local_Y)))

    ego = TrajectoryData(di_idx, ti_idx,
                         data[di_idx].Vehicle_ID.iloc[ti_idx],
                         data[di_idx].Lane_ID.iloc[ti_idx],
                         data[di_idx].Local_X.iloc[ti_idx],
                         data[di_idx].Local_Y.iloc[ti_idx],
                         data[di_idx].v_Vel.iloc[ti_idx],
                         data[di_idx].v_Acc.iloc[ti_idx])

    surr = get_surrounding_vehicles(ego, data)

    feature = {'time_preceding': np.nan,
               'space_preceding': np.nan,
               'time_following': np.nan,
               'space_following': np.nan,
               'time_left_lead': np.nan,
               'space_left_lead': np.nan,
               'time_left_lag': np.nan,
               'space_left_lag': np.nan,
               'time_right_lead': np.nan,
               'space_right_lead': np.nan,
               'time_right_lag': np.nan,
               'space_right_lag': np.nan,
               'velocity_preceding': np.nan,
               'velocity_following': np.nan,
               'velocity_left_lead': np.nan,
               'velocity_left_lag': np.nan,
               'velocity_right_lead': np.nan,
               'velocity_right_lag': np.nan,
               'acc_preceding': np.nan,
               'acc_following': np.nan,
               'acc_left_lead': np.nan,
               'acc_left_lag': np.nan,
               'acc_right_lead': np.nan,
               'acc_right_lag': np.nan,
               'lane_mean_velocity': None,  # 6 lanes
               'lane_density': None,  # 6 lanes
               'ego_lane_ID': np.nan,
               'ego_prev_lane_ID': np.nan,
               'ego_velocity': np.nan,
               'ego_acc': np.nan,
               'ego_lane_position': np.nan
               }

    # Lead-lag, same lane
    if surr.preceding:
        feature['time_preceding'] = di_ti.Time_Headway
        feature['space_preceding'] = di_ti.Space_Headway
        feature['velocity_preceding'] = surr.preceding.velocity
        feature['acc_preceding'] = surr.preceding.acc

    if surr.following:
        feature['space_following'] = ego.y - surr.following.y
        feature['time_following'] = ((ego.y - surr.following.y) /
                                     (surr.following.velocity if surr.following.velocity > 0.1 else 0.1))
        feature['velocity_following'] = surr.following.velocity
        feature['acc_following'] = surr.following.acc

    if surr.right_lead:
        feature['space_right_lead'] = surr.right_lead.y - ego.y
        feature['time_right_lead'] = ((surr.right_lead.y - ego.y) /
                                      surr.right_lead.velocity if surr.right_lead.velocity > 0.1 else 0.1)
        feature['velocity_right_lead'] = surr.right_lead.velocity
        feature['acc_right_lead'] = surr.right_lead.acc

    if surr.right_lag:
        feature['space_right_lag'] = ego.y - surr.right_lag.y
        feature['time_right_lag'] = ((ego.y - surr.right_lag.y) /
                                     surr.right_lag.velocity if surr.right_lag.velocity > 0.1 else 0.1)
        feature['velocity_right_lag'] = surr.right_lag.velocity
        feature['acc_right_lag'] = surr.right_lag.acc

    if surr.left_lead:
        feature['space_left_lead'] = surr.left_lead.y - ego.y
        feature['time_left_lead'] = ((surr.left_lead.y - ego.y) /
                                     surr.left_lead.velocity if surr.left_lead.velocity > 0.1 else 0.1)
        feature['velocity_left_lead'] = surr.left_lead.velocity
        feature['acc_left_lead'] = surr.left_lead.acc

    if surr.left_lag:
        feature['space_left_lag'] = ego.y - surr.left_lag.y
        feature['time_left_lag'] = ((ego.y - surr.left_lag.y) /
                                    surr.left_lag.velocity if surr.left_lag.velocity > 0.1 else 0.1)
        feature['velocity_left_lag'] = surr.left_lag.velocity
        feature['acc_left_lag'] = surr.left_lag.acc

    sim_traj = simultaneous_trajectories(ego, data)
    feature['lane_mean_velocity'] = []
    feature['lane_density'] = []
    vels = []
    for lane_id in range(6):
        vels = [dt.velocity for dt in sim_traj if dt.lane_ID == lane_id]
        if len(vels) > 0:
            feature['lane_mean_velocity'].append(np.mean(vels))
        else:
            feature['lane_mean_velocity'].append(np.nan)
        feature['lane_density'].append(len(vels) / 5)  # Vehicles per 100
    feature['ego_lane_ID'] = ego.lane_ID
    feature['ego_velocity'] = ego.velocity
    feature['ego_acc'] = ego.acc

    lane_traj = data[ego.traj_index].Lane_ID.iloc[:ego.time_index].values
    ch_idx = np.argwhere(np.diff(lane_traj[::-1]) != 0)
    if len(ch_idx) > 0:
        feature['ego_prev_lane_ID'] = lane_traj[::-1][ch_idx[0][0] + 1]
    else:
        feature['ego_prev_lane_ID'] = ego.lane_ID

    feature['ego_lane_position'] = relative_lane_position(ego, lane_bounds)

    impute(feature)

    # Collect feature vector in numpy array
    feature_vector = np.hstack((
        feature['time_preceding'],
        feature['space_preceding'],
        feature['time_following'],
        feature['space_following'],
        feature['time_left_lead'],
        feature['space_left_lead'],
        feature['time_left_lag'],
        feature['space_left_lag'],
        feature['time_right_lead'],
        feature['space_right_lead'],
        feature['time_right_lag'],
        feature['space_right_lag'],
        feature['velocity_preceding'],
        feature['velocity_following'],
        feature['velocity_left_lead'],
        feature['velocity_left_lag'],
        feature['velocity_right_lead'],
        feature['velocity_right_lag'],
        feature['acc_preceding'],
        feature['acc_following'],
        feature['acc_left_lead'],
        feature['acc_left_lag'],
        feature['acc_right_lead'],
        feature['acc_right_lag'],
        feature['lane_mean_velocity'],
        feature['lane_density'],
        feature['ego_lane_ID'],
        feature['ego_prev_lane_ID'],
        feature['ego_velocity'],
        feature['ego_acc'],
        feature['ego_lane_position']))

    # Determine action
    label = np.zeros(len(pred_horizon))
    n = len(data[di_idx].Lane_ID)

    for k, pred_i in enumerate(pred_horizon):
        tpred_end = np.min((ti_idx + pred_i, n))
        lane_ch = np.diff(data[di_idx].Lane_ID.iloc[ti_idx:tpred_end])
        lane_ch_idx = np.argwhere(lane_ch != 0).reshape(-1)
        if len(lane_ch_idx) > 0:
            label[k] = lane_ch[np.min(lane_ch_idx)]
        else:
            label[k] = 0

    return feature_vector, label


def load_i80_features(data_dir='./'):
    """Load features based on the I-80 dataset.
    
    Input:
    data_dir -- Directory where the data directory i80_data is found. Defaults to current directory.
    """

    data_files = ['i80_data/data_3_0400pm_0415pm.npz', 'i80_data/data_3_0500pm_0515pm.npz', 'i80_data/data_3_0515pm_0530pm.npz']

    if not os.path.isfile(data_dir + data_files[0]):
        print("Can't locate feature data, ensure that files")
        print(data_files)
        print("is located in the current directory. If you are running in a LiU Linux lab, do\n")
        print('load_i80_features("/courses/tsfs12/i80_data/")')
        return None, None, None

    d = []
    for fi in data_files:
        di = np.load(data_dir + fi)
        d.append({'ds': di['ds'],
                  'y': di['y'],
                  'info': di['info'],
                  'pred_horizon': di['pred_horizon'],
                  'time_between_predictions': di['pred_horizon']})

    # Combine datasets
    prediction_horizon = 1  # [1, 3, 5, 7] sec
    x = np.vstack([di['ds'] for di in d])
    y = np.hstack([di['y'][:, prediction_horizon] for di in d]) + 1
    info = np.vstack([di['info'] for di in d])

    return x, y, info


def load_i80_trajectories(data_dir='./'):
    """Load trajectories in the I-80 dataset.
    
    Input:
    data_dir -- Directory where the data directory i80_data is found. Defaults to current directory.
    """

    trajectory_files = ['i80_data/0400pm-0415pm/trajectories-0400-0415.csv',
                        'i80_data/0500pm-0515pm/trajectories-0500-0515.csv',
                        'i80_data/0515pm-0530pm/trajectories-0515-0530.csv']


    if not os.path.isfile(os.path.join(data_dir, trajectory_files[0])):
        print("Can't locate trajectory data, ensure that files")
        print(trajectory_files)
        print("is located in the current directory. If you are running in a LiU Linux lab, do\n")
        print('load_i80_trajectories("/courses/tsfs12/i80_data/")')
        return None

    return [read_trajectories(os.path.join(data_dir, trajectory_file_i)) for trajectory_file_i in trajectory_files]
                          
def load_i80_gp_dataset(data_dir='./'):
    """Load trajectories from the I-80 dataset used for the GP model.
    
    Input:
    data_dir -- Directory where the data directory i80_data is found. Defaults to current directory.
    """
    
    def normalized_path_position(x, y):
        s = np.hstack((0, np.cumsum(np.sqrt(np.diff(x)**2 + np.diff(y)**2))))
        return s / s[-1]


    trajectory_file = 'i80_data/0500pm-0515pm/trajectories-0500-0515.csv'

    if not os.path.isfile(os.path.join(data_dir, trajectory_file)):
        print("Can't locate trajectory data, ensure that files")
        print(trajectory_files)
        print("is located in the current directory. If you are running in a LiU Linux lab, do\n")
        print('load_i80_trajectories("/courses/tsfs12/i80_data/")')
        return None

    data = read_trajectories(os.path.join(data_dir, trajectory_file))
    tracks_x_I80 = [di.Local_X.values for di in data]
    tracks_y_I80 = [di.Local_Y.values for di in data]
    lane_id_I80 = [di.Lane_ID.values for di in data]
    N_paths = len(data)
    tracks_s_I80 = [normalized_path_position(xi, yi) for xi, yi in zip(tracks_x_I80, tracks_y_I80)]

    return tracks_s_I80, tracks_x_I80, tracks_y_I80, lane_id_I80, N_paths

def get_trajectory_from_datapoint(datapoint_index, info, trajectories):
    """Get trajectory and prediction datapoint index
    
    Input:
    datapoint_index -- Index in dataset for point of prediction
    info -- information vector from the feature data set
    trajectories -- trajectories from the I-80 dataset
    
    Output:
    trajectory -- the trajectory containing the prediction data_point
    data_points_index -- index to all points on the trajectory included in the feature dataset
    """
    
    trajectory_idx, time_idx, dataset = info[datapoint_index]
    trajectory = trajectories[dataset][trajectory_idx]
    data_points_index = np.argwhere(np.logical_and(info[:, 0] == trajectory_idx, 
                                                   info[:, 2] == dataset)).reshape(-1)
    return trajectory, data_points_index
