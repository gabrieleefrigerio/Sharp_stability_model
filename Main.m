% ----------------- Impostazioni iniziali -----------------
clear; clc; close all;

% ----------------- Import data -----------------
Bike = load_PMF_V2();
[Bike, Output, P, T] = getBike(Bike);
Bike.data = computeData(P,Bike, Output);

% ----------------- Model Inputs -----------------
Bike.Input.v_start = 0.1; % start speed in km/h
Bike.Input.v_finish = 200; % finish speed in km/h
Bike.Input.n_points = 200; % speed discretization points

% function to find model input vectors
Bike = ModelInput(Bike);

% Solver Model
Results = SoverModel(Bike);

% Plot Results
PlotResults(Results, Bike.Input)


