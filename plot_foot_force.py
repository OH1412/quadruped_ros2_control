#!/usr/bin/env python3
"""
Parse recorded `/guide/foot_force` log and plot per-foot forces.

This script expects a log file produced by `ros2 topic echo /guide/foot_force` (text)
and attempts to extract numeric arrays from each message. The Unitree Guide publishes
12 values (3 axes per foot) in `Float32MultiArray` order; this script groups them
per-foot and plots x,y,z plus magnitude over time for each foot.

Usage:
  python3 plot_foot_force.py -d <logs_dir_name> [-o output_dir]

Example:
  python3 plot_foot_force.py -d 20260306_174350
"""

import re
import os
import argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def parse_float_list_from_text(text):
    # find numeric tokens (ints, floats, scientific)
    nums = re.findall(r"[-+]?[0-9]*\.?[0-9]+(?:[eE][-+]?[0-9]+)?", text)
    return [float(x) for x in nums]


def read_messages_from_log(file_path):
    # Split messages by document separator lines or blank-lines that often separate ros2 echo outputs
    with open(file_path, 'r', encoding='utf-8', errors='ignore') as f:
        content = f.read()

    # Split by '---' documents or by two newlines
    parts = []
    if '---' in content:
        parts = [p for p in content.split('---') if p.strip()]
    else:
        # split by blank line groups
        parts = [p for p in re.split(r"\n\s*\n", content) if p.strip()]

    messages = []
    for p in parts:
        # Prefer numbers after a 'data:' field (ros2 topic echo YAML style)
        if 'data:' in p:
            tail = p.split('data:')[-1]
            nums = parse_float_list_from_text(tail)
        else:
            nums = parse_float_list_from_text(p)
        if not nums:
            continue
        # take contiguous groups of 12 if there are many numbers in one doc
        if len(nums) >= 12:
            # if multiple of 12, break into chunks; else take first 12
            if len(nums) % 12 == 0:
                for i in range(0, len(nums), 12):
                    messages.append(nums[i:i+12])
            else:
                # If parsing included extra leading/trailing numbers (e.g. data_offset),
                # prefer taking the last 12 numbers which are most likely the actual 'data' block.
                if len(nums) > 12:
                    messages.append(nums[-12:])
                else:
                    messages.append(nums[:12])
        else:
            # skip messages with insufficient elements
            continue

    return np.array(messages) if messages else np.empty((0, 12))


def plot_foot_forces(data, out_dir):
    # data: N x 12 array
    os.makedirs(out_dir, exist_ok=True)
    n = data.shape[0]
    time = np.arange(n)

    foot_names = ['FR', 'FL', 'RR', 'RL']
    for f in range(4):
        cols = [3*f + i for i in range(3)]
        xyz = data[:, cols]
        mag = np.linalg.norm(xyz, axis=1)

        fig, axes = plt.subplots(4, 1, figsize=(10, 9))
        axes[0].plot(time, xyz[:, 0], label='Fx')
        axes[0].plot(time, xyz[:, 1], label='Fy')
        axes[0].plot(time, xyz[:, 2], label='Fz')
        axes[0].legend(); axes[0].grid(True); axes[0].set_ylabel('Force (N)')

        axes[1].plot(time, xyz[:, 0], color='tab:blue'); axes[1].set_ylabel('Fx'); axes[1].grid(True)
        axes[2].plot(time, xyz[:, 1], color='tab:orange'); axes[2].set_ylabel('Fy'); axes[2].grid(True)
        axes[3].plot(time, xyz[:, 2], color='tab:green'); axes[3].set_ylabel('Fz'); axes[3].grid(True); axes[3].set_xlabel('Sample')

        fig.suptitle(f'Foot {foot_names[f]} Forces (xyz)')
        plt.tight_layout()
        png_path = os.path.join(out_dir, f'foot_{foot_names[f]}_xyz.png')
        plt.savefig(png_path, dpi=150, bbox_inches='tight')
        plt.close(fig)

        # magnitude
        plt.figure(figsize=(10, 3))
        plt.plot(time, mag, label='|F|')
        plt.grid(True); plt.xlabel('Sample'); plt.ylabel('Force magnitude (N)')
        plt.title(f'Foot {foot_names[f]} Force Magnitude')
        save_path = os.path.join(out_dir, f'foot_{foot_names[f]}_magnitude.png')
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        plt.close()

    # combined plot: Fz for all feet
    plt.figure(figsize=(12, 4))
    colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red']
    for f in range(4):
        plt.plot(time, data[:, 3*f + 2], label=foot_names[f], color=colors[f])
    plt.legend(); plt.grid(True); plt.xlabel('Sample'); plt.ylabel('Fz (N)'); plt.title('Fz per foot')
    plt.savefig(os.path.join(out_dir, 'fz_all_feet.png'), dpi=150, bbox_inches='tight')
    plt.close()


def plot_foot_forces_comparison(data_ref, data_est, out_dir):
    # Overlay plots of reference vs estimated (both N x 12)
    os.makedirs(out_dir, exist_ok=True)
    n = data_ref.shape[0]
    time = np.arange(n)
    foot_names = ['FR', 'FL', 'RR', 'RL']

    for f in range(4):
        cols = [3*f + i for i in range(3)]
        ref_xyz = data_ref[:, cols]
        est_xyz = data_est[:, cols] if data_est is not None and data_est.shape[0] == n else None

        fig, axes = plt.subplots(4, 1, figsize=(10, 9))
        axes[0].plot(time, ref_xyz[:, 0], label='Fx_ref', color='tab:blue')
        if est_xyz is not None:
            axes[0].plot(time, est_xyz[:, 0], label='Fx_est', color='tab:blue', linestyle='--')
        axes[0].plot(time, ref_xyz[:, 1], label='Fy_ref', color='tab:orange')
        if est_xyz is not None:
            axes[0].plot(time, est_xyz[:, 1], label='Fy_est', color='tab:orange', linestyle='--')
        axes[0].plot(time, ref_xyz[:, 2], label='Fz_ref', color='tab:green')
        if est_xyz is not None:
            axes[0].plot(time, est_xyz[:, 2], label='Fz_est', color='tab:green', linestyle='--')
        axes[0].legend(); axes[0].grid(True); axes[0].set_ylabel('Force (N)')

        axes[1].plot(time, ref_xyz[:, 0], color='tab:blue');
        if est_xyz is not None: axes[1].plot(time, est_xyz[:, 0], color='tab:blue', linestyle='--')
        axes[1].set_ylabel('Fx'); axes[1].grid(True)
        axes[2].plot(time, ref_xyz[:, 1], color='tab:orange');
        if est_xyz is not None: axes[2].plot(time, est_xyz[:, 1], color='tab:orange', linestyle='--')
        axes[2].set_ylabel('Fy'); axes[2].grid(True)
        axes[3].plot(time, ref_xyz[:, 2], color='tab:green');
        if est_xyz is not None: axes[3].plot(time, est_xyz[:, 2], color='tab:green', linestyle='--')
        axes[3].set_ylabel('Fz'); axes[3].grid(True); axes[3].set_xlabel('Sample')

        fig.suptitle(f'Foot {foot_names[f]} Forces (ref vs est)')
        plt.tight_layout()
        png_path = os.path.join(out_dir, f'comp_foot_{foot_names[f]}_xyz.png')
        plt.savefig(png_path, dpi=150, bbox_inches='tight')
        plt.close(fig)

        # magnitude
        ref_mag = np.linalg.norm(ref_xyz, axis=1)
        plt.figure(figsize=(10, 3))
        plt.plot(time, ref_mag, label='|F|_ref')
        if est_xyz is not None:
            est_mag = np.linalg.norm(est_xyz, axis=1)
            plt.plot(time, est_mag, label='|F|_est', linestyle='--')
        plt.grid(True); plt.xlabel('Sample'); plt.ylabel('Force magnitude (N)')
        plt.title(f'Foot {foot_names[f]} Force Magnitude (ref vs est)')
        plt.legend()
        save_path = os.path.join(out_dir, f'comp_foot_{foot_names[f]}_magnitude.png')
        plt.savefig(save_path, dpi=150, bbox_inches='tight')
        plt.close()

    # combined Fz
    plt.figure(figsize=(12, 4))
    colors = ['tab:blue', 'tab:orange', 'tab:green', 'tab:red']
    for f in range(4):
        plt.plot(time, data_ref[:, 3*f + 2], label=f'{foot_names[f]}_ref', color=colors[f])
        if data_est is not None and data_est.shape[0] == n:
            plt.plot(time, data_est[:, 3*f + 2], linestyle='--', color=colors[f], label=f'{foot_names[f]}_est')
    plt.legend(); plt.grid(True); plt.xlabel('Sample'); plt.ylabel('Fz (N)'); plt.title('Fz per foot (ref vs est)')
    plt.savefig(os.path.join(out_dir, 'comp_fz_all_feet.png'), dpi=150, bbox_inches='tight')
    plt.close()


def main():
    parser = argparse.ArgumentParser(description='Plot guide foot_force log')
    parser.add_argument('-d', '--directory', required=False, default=None, help='logs directory under ./logs (auto-find latest if omitted)')
    parser.add_argument('-f', '--file', default='guide_foot_force.log', help='log file name (default guide_foot_force.log)')
    parser.add_argument('-f2', '--file2', default='guide_estimated_foot_force.log', help='optional second log file for estimated forces')
    parser.add_argument('-o', '--output', default='foot_force_plots', help='output folder')
    args = parser.parse_args()

    # determine input directory: use provided or auto-find latest under ./logs
    chosen_dir = args.directory
    if chosen_dir is None:
        base_logs = './logs'
        if not os.path.isdir(base_logs):
            print(f"Logs base directory not found: {base_logs}")
            return
        # list subdirectories
        subdirs = [os.path.join(base_logs, d) for d in os.listdir(base_logs) if os.path.isdir(os.path.join(base_logs, d))]
        if not subdirs:
            print(f"No log subdirectories found in {base_logs}")
            return
        # pick most recently modified
        latest = max(subdirs, key=lambda p: os.path.getmtime(p))
        chosen_dir = os.path.basename(latest)
        print(f"Auto-selected latest logs directory: {chosen_dir}")

    input_dir = os.path.join('./logs', chosen_dir)
    if not os.path.isdir(input_dir):
        print(f"Input directory not found: {input_dir}")
        return

    log_path = os.path.join(input_dir, args.file)
    if not os.path.isfile(log_path):
        print(f"Log file not found: {log_path}")
        return

    data = read_messages_from_log(log_path)
    if data.size == 0:
        print('No valid messages parsed (expected 12 floats per message).')
        return

    # try to load optional second file (estimated)
    file2_path = os.path.join(input_dir, args.file2)
    data2 = None
    if os.path.isfile(file2_path):
        data2 = read_messages_from_log(file2_path)
        if data2.size == 0:
            print('Warning: second log parsed but contained no valid messages.')
            data2 = None
        else:
            # if lengths differ, trim to min length for comparison
            if data2.shape[0] != data.shape[0]:
                m = min(data.shape[0], data2.shape[0])
                data = data[:m, :]
                data2 = data2[:m, :]

    out_dir = os.path.join(args.output, chosen_dir)
    plot_foot_forces(data, out_dir)
    if data2 is not None:
        comp_out = os.path.join(args.output, chosen_dir, 'comparison')
        plot_foot_forces_comparison(data, data2, comp_out)
    print('Plots saved to', out_dir)


if __name__ == '__main__':
    main()
