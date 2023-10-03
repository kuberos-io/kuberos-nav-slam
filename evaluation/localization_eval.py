#!/usr/bin/env python3

import numpy as np
import argparse
import os
from pathlib import Path

from rosbags.typesys import get_types_from_msg, register_types
from ament_index_python.packages import get_package_share_directory
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr
import inspect

from scipy.spatial.transform import Rotation as R
import pandas as pd
import cv2
import json
import yaml
from tqdm import tqdm

# plotting
import matplotlib.pyplot as plt
import plotly as plotly
import plotly.express as px
import plotly.io as pio

class BagParser(object):

    """Class for parsing bagfile for edge evaluation. """

    def __init__(self, bag, run_index=0):
        """initialization

        :bag: path to bagfile

        """
        self.bagpath = bag.resolve()
        self.filename = bag.stem
        self.register_missing_msg_types(self.bagpath)
        self.metadata = self.parse_meta_data(str(self.bagpath.parent) + '/configmap.yaml')
        if self.metadata is not None:
            self.metadata['experiment_name'] = self.metadata['experiment_name'] + '_' + str(run_index).zfill(4)

    @staticmethod
    def parse_meta_data(yaml_file_path):
        if os.path.exists(yaml_file_path):
            yaml_string = open(str(yaml_file_path), "r")
            python_dict = yaml.load(yaml_string, Loader=yaml.SafeLoader)
            json_string = json.dumps(python_dict)
            meta_data = json.loads(json_string)

            return meta_data[0]['data']
        else:
            return None

    @staticmethod
    def guess_msgtype(path: Path) -> str:
        """Guess message type name from path."""""
        name = path.relative_to(path.parents[2]).with_suffix('')
        if 'msg' not in name.parts:
            name = name.parent / 'msg' / name.name
        return str(name)

    def register_missing_msg_types(self, bagpath):
        """register missing msg types such that the rosbags library is able to
        handle them. This only works if the messages in the workspace correspond
        to the messages in the bagfile, i.e., if messages have changed since the
        recording of the bagfile, the old message also need to be available in the
        system

        :bagpath: path to the bagfile
        :returns: -

        """
        with Reader(bagpath) as reader:
            add_types = {}
            for connection, timestamp, rawdata in reader.messages():
                try:
                    deserialize_cdr(rawdata, connection.msgtype)
                except Exception as e:
                    try:
                        pkg_name = str(e).split('/')[0][1:]
                        msg_name = str(e).split('/')[-1][:-1]
                        msg_path = Path(get_package_share_directory(pkg_name) + '/msg/' + msg_name + '.msg')
                        msg_def = msg_path.read_text(encoding='utf-8')
                        add_types.update(get_types_from_msg(msg_def, self.guess_msgtype(msg_path)))
                        register_types(add_types)
                    except Exception as e1:
                        print(f'something went wrong when trying to add get type from message {e}, namely {e1}')

    def parse_topic_event(self, timestamp, msg, topic):
        """parse publishing of topic and corresponding timestamp

        :timestamp: current timestamp
        :msg: current ROS bag msg
        :returns: dictionary with topic data

        """
        result = {
            'scenario_run': self.filename,
            'timestamp': timestamp,
            'topic_name': topic,
        }

        for k, v in self.metadata.items():
            result[k] = v

        return result

    def parse_robot_pose(self, timestamp, msg, topic, loc):
        """parse robot pose

        :timestamp: current timestamp
        :msg: current ROS bag msg
        :topic: current ROS bag topic
        :returns: dictionary with topic data

        """
        sec = msg.header.stamp.sec
        nanosec = msg.header.stamp.nanosec
        ros_time = sec + nanosec*1e-9
        r = R.from_quat([msg.pose.orientation.x,
                         msg.pose.orientation.y,
                         msg.pose.orientation.z,
                         msg.pose.orientation.w
                         ])
        euler = r.as_euler('xyz')
        yaw = euler[2]
        result = {
            'scenario_run': self.filename,
            'timestamp': timestamp,
            'ros_time': ros_time,
            loc + '_pose_x': msg.pose.position.x,
            loc + '_pose_y': msg.pose.position.y,
            loc + '_pose_yaw': yaw,
        }
        for k, v in self.metadata.items():
            result[k] = v

        return result

    def read_data(self, bagfile):
        """read data from bagfile.

        :bagfile: bagifle to read data from
        :returns: pandas DataFrame containing the parsed data

        """
        data = []
        with Reader(bagfile) as reader:
            for connection, timestamp, rawdata in reader.messages():
                msg = deserialize_cdr(rawdata, connection.msgtype)
                # attributes = inspect.getmembers(msg, lambda a:not(inspect.isroutine(a)))
                # filtered_attributes = [a for a in attributes if not(a[0].startswith('__') and a[0].endswith('__'))]
                if ('base_link_pose' in connection.topic):
                    data.append(self.parse_robot_pose(
                        timestamp,
                        msg,
                        connection.topic,
                        loc='loc'))
                elif ('groundtruth_pose' in connection.topic):
                    data.append(self.parse_robot_pose(
                        timestamp,
                        msg,
                        connection.topic,
                        loc='gt'))
                else:
                    data.append(self.parse_topic_event(timestamp, msg, connection.topic))

        return data


class LocalizationEval(object):

    """Docstring for LocalizationEval. """

    def __init__(self, df, metadata):
        self.df = df
        self.metadata = metadata

    def np_poses(self, df_loc, df_gt, gt_inds=None):
        """get poses as np array

        :df_loc: pandas DataFrame with localization poses
        :df_gt: pandas DataFrame with ground truth poses
        :returns: poses as np array

        """
        loc_xs = df_loc['loc_pose_x'].to_numpy()
        loc_ys = df_loc['loc_pose_y'].to_numpy()
        loc_yaws = df_loc['loc_pose_yaw'].to_numpy()

        gt_xs = df_gt.gt_pose_x.to_numpy()
        gt_ys = df_gt.gt_pose_y.to_numpy()
        gt_yaws = df_gt.gt_pose_yaw.to_numpy()

        if gt_inds is not None:
            gt_xs = gt_xs[gt_inds].reshape(gt_xs[gt_inds].shape[0],)
            gt_ys = gt_ys[gt_inds].reshape(gt_ys[gt_inds].shape[0],)
            gt_yaws = gt_yaws[gt_inds].reshape(gt_yaws[gt_inds].shape[0],)

        loc_poses = np.array([loc_xs, loc_ys, loc_yaws]).transpose()
        gt_poses = np.array([gt_xs, gt_ys, gt_yaws]).transpose()

        return loc_poses, gt_poses

    def calculate_APE(self, loc_poses, gt_poses):
        """this function calculates the absolute pose error (APE) for aligned
        trajectory poses

        :loc_poses: localizaton poses (x,y,yaw)
        :gt_poses: ground truth poses (x,y,yaw)
        :returns: two arrays containing the APE in translation (m) and rotation
        (unitless) over the full trajectory

        """
        # let's calculate the translational error (the simpler part), i.e., the
        # Euclidean distance between the estimated and ground truth 2D position
        ape_trans = np.linalg.norm(gt_poses[:, :2] - loc_poses[:, :2], axis=1)

        # now, let's calculate the rotational error (slightly more complex)
        # first, we calculate the rotation matrices for the estimated and
        # ground truth poses from the yaw angles
        r_mat_loc = R.from_euler(
            'z', loc_poses[:,2], degrees=False).as_matrix()
        r_mat_gt = R.from_euler(
            'z', gt_poses[:,2], degrees=False).as_matrix()

        # this line is where the magic happens: without localization errors,
        # the two rotations for loc and gt are supposed to be the same, so the
        # result of r_mat_loc \dot r_mat_gt^(-1) is the rotation error. We
        # calculate the rotation vector of this rotation error using Rodrigues'
        # algorithm
        r_vec_errs = np.array([cv2.Rodrigues(np.dot(r_mat_loc[i,:,:], np.linalg.inv(r_mat_gt)[i,:,:]))[0] for i in range(loc_poses.shape[0])])

        # finally, we need to calculate the norm of the resulting rotation
        # vectors
        ape_rot = np.linalg.norm(r_vec_errs, axis=1)

        return ape_trans, ape_rot

    def calculate_RPE(self, loc_poses, gt_poses, delta=1.0):
        """this function calculates the relative pose error (APE) for aligned
        trajectory poses. A relative pose is the delta between successive
        trajectory posesi, i.e., p_{i+1} - p_{i}. Here, we calculate delta_loc
        and delta_gt sequences and calculate the error between delta_loc and
        delta_gt

        :loc_poses: localizaton poses (x,y,yaw)
        :gt_poses: ground truth poses (x,y,yaw)
        :returns: two arrays containing the RPE in translation (m) and rotation
        (unitless) over the full trajectory

        """
        delta_loc = loc_poses[1:,:] - loc_poses[:-1,:]
        delta_gt = gt_poses[1:,:] - gt_poses[:-1,:]
        delta_driven_distance = np.linalg.norm(delta_gt[:,:2], axis=1)
        driven_dist = np.array([np.sum(delta_driven_distance[:i]) for i in range(len(delta_driven_distance))])

        total_distance_driven = np.max(driven_dist)

        delta_steps = np.arange(0, total_distance_driven - delta, delta) + delta
        rpe_trans_result = np.zeros(loc_poses.shape[0])
        rpe_rot_result = np.zeros(loc_poses.shape[0])

        if delta_steps.size>0:
            indices = np.argmin([np.abs(driven_dist - delta_step) for delta_step in delta_steps], axis=1)
            indices +=1
            indices = np.insert(indices,0,0)

            loc_poses_delta = loc_poses[indices,:]
            gt_poses_delta = gt_poses[indices,:]

            delta_loc = loc_poses_delta[1:,:] - loc_poses_delta[:-1,:]
            delta_gt = gt_poses_delta[1:,:] - gt_poses_delta[:-1,:]

            rpe_trans, rpe_rot = self.calculate_APE(delta_loc, delta_gt)
            rpe_trans_result[indices[1:]] = rpe_trans
            rpe_rot_result[indices[1:]] = rpe_rot[:,0]
        else:
            print(f'WARNING: not enough data to calculate rpe with delta={delta}, the driven distance is only {total_distance_driven}')

        delta_driven_distance = np.insert(delta_driven_distance,0,0)
        driven_dist = np.insert(driven_dist,0,0)

        return rpe_trans_result, rpe_rot_result, delta_driven_distance, driven_dist


    def align_poses(self):
        """align localization and groundtruh poses
        :returns: indices of the ground truth poses corresponding to the
        localization poses as well as the two pose arrays and the corresponding
        (ROS) time stamps

        """
        df_loc = self.df[~self.df['loc_pose_x'].isna()]
        df_gt = self.df[~self.df['gt_pose_x'].isna()]
        loc_times = df_loc.ros_time.to_numpy()
        gt_times = df_gt.ros_time.to_numpy()
        # we assume that there are more ground truth than localization poses
        # available in the bag file and thus look for the gt poses, whose times
        # best match the localization poses
        gt_inds = np.array([[np.argmin(np.abs(gt_times - loc_time))] for loc_time in loc_times])
        loc_poses, gt_poses = self.np_poses(
            df_loc,
            df_gt,
            gt_inds=gt_inds)

        return gt_inds, loc_poses, gt_poses, loc_times

    def load_eval_data(self, path):
        """load eval data dict from csv file containing pandas DataFrame

        :path: path to the csv file
        :returns: -

        """
        df = pd.read_csv(path)
        self.eval_data = df.to_dict("list")
        del df

    def create_eval_data(self):
        """do everything
        :returns: pandas DataFrame with aligned trajectory poses and
        evaluation metrics

        """
        gt_inds, loc_poses, gt_poses, loc_times = self.align_poses()

        ape_trans, ape_rot = self.calculate_APE(loc_poses, gt_poses)
        rpe_trans, rpe_rot, delta_driven_distance, driven_dist = self.calculate_RPE(loc_poses, gt_poses)

        self.eval_data = dict(
            ros_time=loc_times,
            loc_pose_x=loc_poses[:,0],
            loc_pose_y=loc_poses[:,1],
            loc_pose_yaw=loc_poses[:,2],
            gt_pose_x=gt_poses[:,0],
            gt_pose_y=gt_poses[:,1],
            gt_pose_yaw=gt_poses[:,2],
            ape_trans=ape_trans,
            ape_rot=ape_rot[:,0],
            rpe_trans=rpe_trans,
            rpe_rot=rpe_rot,
            driven_distance=driven_dist,
            driven_distance_delta=delta_driven_distance
        )

        for k,v in self.metadata.items():
            self.eval_data[k] = v

        return self.eval_data

    def summary(self, bagpath):
        """One row summary
        """

        job_name = str(bagpath)
        for item in job_name.split('/'):
            if str(item).startswith('job'):
                job_name = item
                break

        goal = str(self.eval_data['goal_poses']).split(';')[-1]

        ape_trans = np.array(self.eval_data['ape_trans'])
        ape_trans_nonzero = ape_trans[np.nonzero(ape_trans)]
        ape_rot = np.array(self.eval_data['ape_rot'])
        ape_rot_nonzero = ape_rot[np.nonzero(ape_rot)]

        summary = {
            'bagpath': bagpath,
            'start_pose': self.eval_data['start_pose'][0],
            'goal': goal,
            'goal_poses': self.eval_data['goal_poses'][0],
            'localization_method': self.eval_data['localization_method'][0],
            'world': self.eval_data['world'][0],
            'world_var': self.eval_data['world_var'][0],
            # task performance
            'navigation_time': self.eval_data['ros_time'][-1],
            'driven_distance': self.eval_data['driven_distance'][-1],
            'reached_goal_pose_x_gt': self.eval_data['gt_pose_x'][-1],
            'reached_goal_pose_y_gt': self.eval_data['gt_pose_y'][-1],
            'goal_tran_error': np.sqrt(np.square(self.eval_data['gt_pose_x'][-1] - float(goal.split(',')[0])) +
                                       np.square(self.eval_data['gt_pose_y'][-1] - float(goal.split(',')[1]))),
            # statistic evaluation
            'ape_mean': np.mean(ape_trans_nonzero),
            'ape_rmse': np.sqrt(np.dot(ape_trans_nonzero, ape_trans_nonzero)/ len(ape_trans_nonzero)),
            'ape_std': np.std(ape_trans_nonzero),
            'ape_rot_mean': np.mean(ape_rot_nonzero)
        }

        return summary

class PlotSingleRun(object):

    def __init__(self):
        pass

    @staticmethod
    def plot_trajectories_topdown(df, output_path=None, dpi=None):
        plt.style.use(['seaborn-darkgrid'])
        fig = plt.figure(figsize=(20,10))
        localization_method = str(df.localization_method.unique()[0])
        plt.scatter(df['gt_pose_x'], df['gt_pose_y'], s=10)
        plt.plot(df['gt_pose_x'].to_numpy(), df['gt_pose_y'].to_numpy(), label='Ground Truth Pose')
        plt.scatter(df['loc_pose_x'], df['loc_pose_y'], s=10)
        plt.plot(df['loc_pose_x'].to_numpy(), df['loc_pose_y'].to_numpy(), label=localization_method + ' Pose')
        plt.legend(fontsize=20)
        plt.xlabel('X position (m)', fontsize=20)
        plt.ylabel('Y position (m)', fontsize=20)
        plt.xticks(fontsize=18)
        plt.yticks(fontsize=18)
        plt.tight_layout()

        if output_path is not None:
            if dpi is None:
                dpi=300
            plt.savefig(output_path, dpi=dpi)

        return fig

    @staticmethod
    def plot_trajectory_components(df, output_path=None, dpi=None):
        plt.style.use(['seaborn-darkgrid'])

        fig, axs = plt.subplots(3, figsize=(20, 10))
        # X component
        axs[0].plot(df.ros_time.to_numpy(), df.gt_pose_x.to_numpy(), label='Ground Truth')
        axs[0].plot(df.ros_time.to_numpy(), df.loc_pose_x.to_numpy(), label=df.localization_method.unique()[0])
        axs[0].set(ylabel='X position (m)')
        axs[0].legend(loc=1)
        # Y component
        axs[1].plot(df.ros_time.to_numpy(), df.gt_pose_y.to_numpy(), label='Ground Truth')
        axs[1].plot(df.ros_time.to_numpy(), df.loc_pose_y.to_numpy(), label=df.localization_method.unique()[0])
        axs[1].set(ylabel='Y position (m)')
        axs[1].legend(loc=1)
        # yaw component
        axs[2].plot(df.ros_time.to_numpy(), df.gt_pose_yaw.to_numpy(), label='Ground Truth')
        axs[2].plot(df.ros_time.to_numpy(), df.loc_pose_yaw.to_numpy(), label=df.localization_method.unique()[0])
        axs[2].set(ylabel='Rotation (rad)', xlabel='Time (s)')
        axs[2].legend(loc=1)
        plt.tight_layout()

        if output_path is not None:
            if dpi is None:
                dpi=300
            plt.savefig(output_path, dpi=dpi)

        return fig

    @staticmethod
    def plot_ape(df, output_path=None, dpi=None):
        plt.style.use(['seaborn-darkgrid'])

        fig, axs = plt.subplots(2, figsize=(20, 10))
        axs[0].plot(df.ros_time.to_numpy(), df.ape_trans.to_numpy(), label=df.localization_method.unique()[0])
        axs[0].set(ylabel='APE position (m)')
        axs[0].legend(loc=1)

        axs[1].plot(df.ros_time.to_numpy(), df.ape_rot.to_numpy(), label=df.localization_method.unique()[0])
        axs[1].set(ylabel='APE rotation (unitless)', xlabel='Time (s)')
        axs[1].legend(loc=1)
        plt.tight_layout()

        if output_path is not None:
            if dpi is None:
                dpi=300
            plt.savefig(output_path, dpi=dpi)

        return fig


class PlotAllRuns():

    def __init__(self):
        pass

    @staticmethod
    def plot_ape_trans(df, output_path=None):
        fig = px.box(
            df,
            x="localization_method", y="ape_trans", color="world",
            labels={
                'ape_trans':'ATE translation (m)', 'localization_method': 'localization algorithm'},)
        fig.update_layout(legend=dict(
                yanchor="top",
                y=0.99,
                xanchor="left",
                x=0.01

        ))
        fig.update_layout(
                plot_bgcolor='white'

        )
        fig.update_xaxes(
                mirror=True,
                ticks='outside',
                showline=True,
                linecolor='black',
                gridcolor='lightgrey'

        )
        fig.update_yaxes(
                mirror=True,
                ticks='outside',
                showline=True,
                linecolor='black',
                gridcolor='lightgrey'

        )
        
        if output_path is not None:
            fig.write_image(output_path)
        return fig

    @staticmethod
    def plot_ape_rot(df, output_path=None):
        fig = px.box(
            df,
            x="localization_method", y="ape_rot", color="world",
            labels={
                'ape_trans':'APE rotation (unitless)', 'localization_method': 'localization algorithm'},)
        if output_path is not None:
            fig.write_image(output_path)
        return fig

    @staticmethod
    def plot_rpe_trans(df, output_path=None):
        fig = px.box(
            df[df['rpe_rot']>0],
            x="localization_method", y="rpe_trans", color="world",
            labels={'rpe_trans':'RPE translation (m per driven meter)', 'localization_method': 'localization algorithm'},)
        if output_path is not None:
            fig.write_image(output_path)
        return fig

    @staticmethod
    def plot_rpe_rot(df, output_path=None):
        fig = px.box(
            df[df['rpe_rot']>0],
            x="localization_method", y="rpe_rot", color="world",
            labels={'rpe_rot':'RPE rotation (rad per driven meter)', 'localization_method': 'localization algorithm'},)
        if output_path is not None:
            fig.write_image(output_path)
        return fig

    @staticmethod
    def plot_ape_trans_vs_world(df, output_path=None):
        fig = px.box(
            df[df['ape_trans']>0],
            x="world_var", y="ape_trans",
            # color="world",
            labels={'rpe_trans':'RPE translation (m per driven meter)', 'world_var': 'World variant'},
        )
        if output_path is not None:
            fig.write_image(output_path)
        return fig

    @staticmethod
    def plot_ape_rot_vs_world(df, output_path=None):
        fig = px.box(
            df[df['ape_rot']>0],
            x="world_var", y="ape_rot", color="world",
            labels={'rpe_rot':'RPE rotation (rad per driven meter)', 'world_var': 'World variant'},
        )
        if output_path is not None:
            fig.write_image(output_path)
        return fig

def get_rosbag_dirs(parent_dir, rosbag_dirs):
    for path, dirs, _ in os.walk(parent_dir):
        for d in dirs:
            current = os.path.join(path, d)
            if any(fname.endswith('.db3') for fname in os.listdir(current)):
                if any(fname.endswith('metadata.yaml') for fname in os.listdir(current)):
                    rosbag_dirs.append(current)


argparser = argparse.ArgumentParser(description='create dataframes from rosbags')

argparser.add_argument(
    '--parent_dir',
    type=str,
    default='/transfer/',
    help='parent directory containing the bag files')

argparser.add_argument(
    '--skip_parse',
    dest='skip_parse',
    action='store_true',
    help='skip parse each single job',
)

args = argparser.parse_args()

if __name__ == "__main__":
    rosbag_dirs = []
    get_rosbag_dirs(args.parent_dir, rosbag_dirs)

    i = 0
    raw_data = []
    eval_data = []
    summary_data = []

    psr = PlotSingleRun()
    par = PlotAllRuns()

    # parse rosbags in each job
    if not args.skip_parse:
        for bag_path in tqdm(rosbag_dirs):
            bagpath = Path(bag_path)
            bp = BagParser(bagpath, run_index=i)
            if bp.metadata is not None:
                i+=1
                run_data = bp.read_data(bagpath)
                raw_data.extend(run_data)

                df = pd.DataFrame(run_data)
                output_path = str(bagpath.resolve())+ '_parsed_raw_data.csv'
                df.to_csv(output_path, index=False)

                le = LocalizationEval(df, metadata=bp.metadata)
                del df
                del run_data
                eval_data_dict = le.create_eval_data()
                eval_data.append(eval_data_dict)

                # summary
                summary_dict = le.summary(bagpath=bagpath)
                summary_data.append(summary_dict)

                df_sum = pd.DataFrame.from_dict(summary_dict)
                output_path = str(bagpath.resolve())+ '_summary.csv'
                df_sum.to_csv(output_path, index=False)
                del df_sum

                df_eval = pd.DataFrame.from_dict(eval_data_dict)
                output_path = str(bagpath.resolve())+ '_loc_eval_data.csv'
                df_eval.to_csv(output_path, index=False)
                # fig = psr.plot_trajectories_topdown(df_eval, output_path=str(bagpath.resolve())+ '_trajectories_top.png', dpi=300)
                # fig = psr.plot_trajectory_components(df_eval, output_path=str(bagpath.resolve())+ '_trajectories_components.png', dpi=300)
                # fig = psr.plot_ape(df_eval, output_path=str(bagpath.resolve())+ '_ape.png', dpi=300)
                del df_eval
                plt.close('all')
            else:
                print(f'[WARN]: no configmap available for f{bagpath}')

        parent_dir = Path(args.parent_dir)
        df = pd.DataFrame(raw_data)
        output_path = str(parent_dir.resolve()) + '/parsed_raw_data.csv'
        df.to_csv(output_path, index=False)

        df = pd.concat([pd.DataFrame.from_dict(d) for d in eval_data])
        output_path = str(parent_dir.resolve()) + '/eval_data.csv'
        df.to_csv(output_path, index=False)

        # summary
        df = pd.DataFrame(summary_data)
        output_path = str(parent_dir.resolve()) + '/summary.csv'
        df.to_csv(output_path, index=False)
    else:
        # read collected csv
        print("[INFO] Skip the evaluation of every single job -->> Processing jobs statistic")
    parent_dir = Path(args.parent_dir)
    collected_eval_data_path = str(parent_dir.resolve()) + '/eval_data.csv'
    df = pd.read_csv(collected_eval_data_path)

    output_path = str(parent_dir.resolve()) + '/ape_trans.png'
    fig = par.plot_ape_trans(df, output_path=output_path)

    output_path = str(parent_dir.resolve()) + '/ape_rot.png'
    fig = par.plot_ape_rot(df, output_path=output_path)

    output_path = str(parent_dir.resolve()) + '/rpe_trans.png'
    fig = par.plot_rpe_trans(df, output_path=output_path)
    print(f"Save <RPE Translation> in {output_path}")

    output_path = str(parent_dir.resolve()) + '/rpe_rot.png'
    fig = par.plot_rpe_rot(df, output_path=output_path)
    print(f"Save <RPE Rotation> in {output_path}")

    output_path = str(parent_dir.resolve()) + '/ape_trans_world.png'
    fig = par.plot_ape_trans_vs_world(df, output_path=output_path)
    print(f"Save <RPE Translation vs. World Variation> in {output_path}")

    output_path = str(parent_dir.resolve()) + '/ape_rot_world.png'
    fig = par.plot_ape_rot_vs_world(df, output_path=output_path)
    print(f"Save <RPE Rotation vs. World Variation> in {output_path}")

    plt.close('all')
