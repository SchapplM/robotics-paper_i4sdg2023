% Convert names of the robots in a readable form other than database name.
% Compute the fitness function once to obtain joint angles for reference
% samples of the trajectory. By this the parallelity of joints is obtained.
% 
% Preliminaries:
% * eval_figures_pareto.m
% 
% Creates file:
% * robot_names_latex.csv

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
usr_writemode = 'edit';
usr_updatemode = 'skip';
%% Definitionen
outputdir = fileparts(which('robot_names.m'));
datadir = fullfile(outputdir,'..','data');
if isempty(which('i4sdg2023_dimsynth_data_dir'))
  error(['You have to create a file i4sdg2023_dimsynth_data_dir pointing to the ', ...
    'directory containing the results of the dimensional synthesis']);
end
resdirtotal = i4sdg2023_dimsynth_data_dir();
serroblibpath=fileparts(which('serroblib_path_init.m'));
%% Öffnen der Ergebnis-Tabelle
% (Wird in results_stack_tables.m erstellt)
tablepath = fullfile(datadir, 'results_all_reps_pareto.csv');
ResTab = readtable(tablepath, 'ReadVariableNames', true);

%% Generiere die Zeichenfolge für die Gelenkkette
namestablepath = fullfile(datadir, 'robot_names_latex.csv');
Robots = unique(ResTab.Name);
ResTab_NameTrans = cell2table(cell(0,7), 'VariableNames', {'PKM_Name', ...
  'Gnum', 'Pnum', 'Chain_Name', 'ChainStructure', 'Chain_Structure_Act', 'Chain_ShortName'});
if strcmp(usr_writemode, 'edit') && exist(namestablepath, 'file')
  ResTab_NameTrans = readtable(namestablepath, 'Delimiter', ';');
end
I = 1:length(Robots);
% Debug: Auswahl eines bestimmten Roboters
% I = find(strcmp(Robots, 'P4RRRRR8V2G1P1A1'))';
for i = I
  RobName = Robots{i};
  fprintf('Bestimme Bezeichnung für Rob %d/%d (%s)\n', i, length(Robots), RobName);
  % Prüfe, ob Roboter schon in Tabelle ist
  if strcmp(usr_updatemode, 'skip') && any(strcmp(ResTab_NameTrans.PKM_Name, RobName))
    fprintf('Roboter %s steht schon in Namenstabelle. Überspringe.\n', RobName);
    continue;
  end
  % Generiere Namen aus abgespeicherten Parallelitäten
  [NLEG, LEG_Names, Actuation, Coupling, ActNr, symrob, EE_dof0, ...
    PName_Kin, PName_Legs, AdditionalInfo_Akt, StructuralDHParam, ...
    JointParallelity] = parroblib_load_robot(RobName);
  Gnum = Coupling(1);
  Pnum = Coupling(2);
  Chain_Name = LEG_Names{1};
  SName_TechJoint = parroblib_format_robot_name(RobName, 2);
  Chain_StructNameAct = parroblib_format_robot_name(RobName, 1);
  Chain_StructName = strrep(Chain_StructNameAct, '\underline', '');
  I_found = strcmp(ResTab_NameTrans.PKM_Name, RobName);
  Row_i = {RobName, Gnum, Pnum, Chain_Name, Chain_StructName, Chain_StructNameAct, SName_TechJoint};
  if any(I_found) % eintragen
    ResTab_NameTrans(I_found,:) = Row_i;
  else % anhängen
    ResTab_NameTrans = [ResTab_NameTrans; Row_i]; %#ok<AGROW>
  end
end

%% Speichere das wieder ab
writetable(ResTab_NameTrans, namestablepath, 'Delimiter', ';');
fprintf('Tabelle %s geschrieben\n', namestablepath);
