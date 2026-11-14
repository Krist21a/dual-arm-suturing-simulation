%% Single interrupted stitch: small proximal motion, distal wrist control.
% Open output/scenes/dual_DVRK_ARM_suture.ttt, stop simulation, then run.
% The scene embeds SutureController. Jaw axes and open/close logic are original.
% The checked plan includes tip-first circular insertion, two continuous
% handoffs, curved extraction and return to the right arm. No knot is tied.
clc; clear; close all;
RunSutureWristCycle(fileparts(mfilename('fullpath')));
