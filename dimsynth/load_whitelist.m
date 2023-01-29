% Passe die Einstellungen der Maßsynthese an (Strukturauswahl)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-09
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function Set = load_whitelist(Set, configset)

loadwhitelist = configset.loadwhitelist;
if ~loadwhitelist
  return
end

%% Lade mögliche Verzeichnisse mit Ergebnissen
dimsynthpath = fileparts(which('structgeomsynth_path_init.m'));
datadir_local = fullfile(dimsynthpath, 'results');
datadirs = {datadir_local};
if ~isempty(which('amun_dimsynth_data_dir.m'))
  datadir2 = amun_dimsynth_data_dir();
  datadirs{2} = datadir2;
else
  warning(['Datei amun_dimsynth_data_dir.m existiert nicht. Diese Datei ', ...
    'sollte eigentlich auf das Seafile-Laufwerk zeigen']);
end
%% Definiere Namen der bisherigen Optimierungen
% Es sollten Versuchsläufe sein, in denen alle in Frage kommenden PKM
% einmal ausprobiert wurden
if all(Set.task.DoF == [1 1 1 0 0 0])
  error('not defined yet')
else
  optlist = {'amunpkm_20211104_3T3R_obj_instspc_p50_rndTraj'};
end
%% Gehe Ergebnisordner durch und lade die Liste gültier PKM
whitelist = {};
for i = 1:length(datadirs)
  for j = 1:length(optlist)
    dir_ij = fullfile(datadirs{i}, optlist{j});
    if ~exist(dir_ij, 'file')
      continue
    end
    ResTab = readtable(fullfile(dir_ij, [optlist{j}, '_results_table.csv']));
    % Alternative 1: Nur i.O.-Ergebnisse
    % fval_thresh = 1e3;
    % Alternative 2: Nur Ergebnisse, bei denen die Trajektorie berechnet
    % werden kann (und die dann nicht singulär sind). Werte aus
    % cds_constraints_traj mit Faktor 1e4 multipliziert.
    fval_thresh = 9e3*1e4;
    % Tabelle nur mit ausgewählten Zeilen
    ResTab_filt = ResTab(ResTab.Fval_Opt<fval_thresh,:);
    whitelist = [whitelist; ResTab_filt.Name]; %#ok<AGROW>
  end
end
Set.structures.whitelist = unique(whitelist');
Set.structures.whitelist_job = optlist;
fprintf('Positiv-Liste aus vorhandenen Versuchen geladen: %d Einträge\n', length(Set.structures.whitelist));