% Run the evaluation of the simulation results for the paper
% This creates the robot plots for the paper.

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clear
clc

this_path = fileparts( mfilename('fullpath') );
addpath(this_path);
addpath(fullfile(fileparts(which('i4sdg2023_dimsynth_data_dir.m')), 'dimsynth_postprocess')); % move to first position on search path
% Versuche laden
run(fullfile(fileparts(which('i4sdg2023_dimsynth_data_dir.m')), 'dimsynth_postprocess', ...
  'eval_figures_pareto.m')); close all;
% Die Namen müssen nur einmalig bestimmt werden (sofern sich die Ergebnisse
% nicht ändern
run(fullfile(fileparts(which('i4sdg2023_dimsynth_data_dir.m')), 'dimsynth_postprocess', ...
  'robot_names.m')); close all;
% Auswertung der existierenden Lösung
run(fullfile(fileparts(which('i4sdg2023_dimsynth_data_dir.m')), 'dimsynth_postprocess', ...
  'eval_existing_design.m')); close all;
% Pareto-Bild (gruppiert)
run(fullfile(fileparts(which('i4sdg2023_dimsynth_data_dir.m')), 'dimsynth_postprocess', ...
  'eval_figures_pareto_groups.m')); 
% Redundanzkarte für Paper zeichnen
run(fullfile(fileparts(which('i4sdg2023_dimsynth_data_dir.m')), 'paper', ...
  'figures', 'perfmap_fig.m'));
% Auswahl der Partikel der Pareto-Front für Roboter-Bilder
run(fullfile(fileparts(which('i4sdg2023_dimsynth_data_dir.m')), 'dimsynth_postprocess', ...
  'select_eval_robot_examples.m'));
% Zeichne Roboter-Bilder für Paper
run(fullfile(fileparts(which('i4sdg2023_dimsynth_data_dir.m')), 'paper', ...
  'figures', 'robot_images.m'));