%% Single interrupted stitch: roll-led distal wrist, small proximal motion.
% Open output/scenes/dual_DVRK_ARM_suture.ttt, stop simulation, then run.
% Original jaw geometry/axes/opening are retained; common yaw is modelled.
% The checked plan includes tip-first circular insertion, two continuous
% handoffs, curved extraction and return to the right arm. No knot is tied.
clc; clear; close all;
RunSutureWristCycle(fileparts(mfilename('fullpath')));
