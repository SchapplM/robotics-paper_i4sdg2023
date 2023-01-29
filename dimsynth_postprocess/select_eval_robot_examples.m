% Wähle Roboter aus den Ergebnissen aus für die Detail-Untersuchung im Paper
% (Detail-Bilder und Tabelle).
% Beeinflusst robot_images.m
% Rechne die Fitness-Funktion nach und speichere alle relevanten Daten ab.
% Der Roboter wird aus der Pareto-Front genommen, entsprechend der
% Einstellung.
% 
% This script is based on the same file eval_figures_pareto_groups.m from 
% https://github.com/SchapplM/robotics-paper_ark2022_3T1R/
% (ARK paper "Inverse Kinematics for Task Redundancy of Symmetric 3T1R
% Parallel Manipulators using Tait-Bryan-Angle Kinematic Constraints",
% Schappler 2022) 

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

usr_figselection = 'actvelo_actforce'; % siehe andere Skripte
%% Benutzereingaben
recalc_fitnessfcn = true; % Neuberechnung der Fitness-Funktion (optional)
% Compute redundancy map and save it next to the results.
recreate_redundancy_map = false;
regenerate_templates = false; %#ok<*UNRCH> % nur bei erstem Aufruf notwendig.
%% Sonstige Initialisierung
paperrepodir = fileparts(which('i4sdg2023_dimsynth_data_dir.m'));
if isempty(paperrepodir)
  error(['You have to create a file i4sdg2023_dimsynth_data_dir pointing to the ', ...
    'directory containing the results of the dimensional synthesis']);
end
importdir = i4sdg2023_dimsynth_data_dir();
datadir = fullfile(paperrepodir, 'data');
tmp = load(fullfile(datadir, sprintf('results_all_reps_pareto.mat')));
ResTab = tmp.ResTab_ges;
tmp = load(fullfile(datadir, sprintf('robot_groups.mat')));
RobotGroups = tmp.RobotGroups;

%% Suche besten Wert für beide Kriterien
bestphysvals = NaN(1,2);
clear data_all
for i = 1:size(RobotGroups,1)
  GroupName = RobotGroups{i,1};
  if RobotGroups{i,3} == 0, continue; end % keine Ergebnisse vorliegend
  data_i = load(fullfile(datadir, sprintf('group_%s_paretofront_actvelo_actforce.mat', GroupName)));
  if ~exist('data_all', 'var')
    data_all = data_i.pt_i;
  else
    data_all = [data_all; data_i.pt_i]; %#ok<AGROW> 
  end
end
data_phys = [data_all.Crit1, data_all.Crit2];
for k = 1:2
  [bestphysvals(k), Imin] = min(data_phys(:,k));
end

%% Alle Gruppen durchgehen
for i = 1:size(RobotGroups,1)
  GroupName = RobotGroups{i,1};
  if RobotGroups{i,3} == 0, continue; end % keine Ergebnisse vorliegend
  fprintf('Lade Daten für PKM-Gruppe %d/%d (%s)\n', i, size(RobotGroups,1), GroupName);
  data_i = load(fullfile(datadir, sprintf('group_%s_paretofront_%s.mat', GroupName, usr_figselection)));
  % Manuelle Anpassung der zu findenden Roboter
  II = (1:size(data_i.pt_i,1))';
  % Wähle das Partikel, dass nach einer gewichteten Summe aus beiden
  % Kriterien am besten ist (jeweils relativ zum bestmöglichen Wert)
  data_i_rel = [data_i.pt_i.Crit1/bestphysvals(1), data_i.pt_i.Crit2/bestphysvals(2)];
  assert(all(data_i_rel(:)>=1), 'normierte Daten müssen größer 1 sein');
  [~,I_sort] = sort(0.5*data_i_rel(:,1)+0.5*data_i_rel(:,2), 'ascend');
  group_i_success = false;
  for iinearest = I_sort' % Gehe alle Partikel durch und nehme das passendste, was reproduzierbar ist
  inearest = II(iinearest);
  % Lade Daten für diesen Roboter aus den Ergebnissen
  Ipar = data_i.pt_i.ParetoIndNr(inearest);
  OptName = data_i.pt_i.OptName{inearest};
  RobName = data_i.pt_i.RobName{inearest};
  LfdNr = data_i.pt_i.LfdNr(inearest);
  fprintf('Wähle Opt. %s, Rob. %d, %s, Partikel %d\n', OptName, LfdNr, RobName, Ipar);
  setfile = dir(fullfile(importdir, OptName, '*settings.mat'));
  d1 = load(fullfile(importdir, OptName, setfile(1).name));
  Set_i = cds_settings_update(d1.Set);
  resfile = fullfile(importdir, OptName, sprintf('Rob%d_%s_Endergebnis.mat', LfdNr, RobName));
  tmp = load(resfile);
  RobotOptRes_i = tmp.RobotOptRes;
  resfile2 = fullfile(importdir, OptName, sprintf('Rob%d_%s_Details.mat', LfdNr, RobName));
  if exist(resfile2,'file')
    % Lade auch die Detail-Ergebnisse. Geht nicht ohne, wenn die
    % Detail-Ergebnisse genutzt wurden, um die Pareto-Front anzupassen.
    % (z.B. durch nachträgliche Filterung nach zusätzlichen Kriterien
    tmp = load(resfile2);
    PSO_Detail_Data_i = tmp.PSO_Detail_Data;
    RobotOptDetails_i = tmp.RobotOptDetails;
  else
    PSO_Detail_Data_i = [];
    RobotOptDetails_i = [];
  end
  % Nummer der Zielfunktionen
  kk1 = find(strcmp(Set_i.optimization.objective, 'actvelo'));
  kk2 = find(strcmp(Set_i.optimization.objective, 'actforce'));
  % Prüfe, ob Werte zueinander passen. Kann nicht mehr direkt die
  % Pareto-Fronten nehmen, da die Pareto-Fronten neu gebildet werden. Es
  % muss immer das passende Partikel in den Zwischenergebnissen gesucht
  % werden.
  if ~isnan(Ipar) && abs(RobotOptRes_i.physval_pareto(Ipar,kk1) - data_i.pt_i.Crit1(inearest)) < 1e-3
    % Der Gesuchte Wert liegt ganz normal auf der Pareto-Front aus dem
    % Optimierungsergebnis (mit viel Toleranz)
    % Lade weitere Daten aus der Ergebnis-Datei (aus Endergebnis-Variable)
    pval = RobotOptRes_i.p_val_pareto(Ipar,:)';
    physval = RobotOptRes_i.physval_pareto(Ipar,:)';
    fval = RobotOptRes_i.fval_pareto(Ipar,:)';
    pval_desopt = RobotOptRes_i.desopt_pval_pareto(Ipar,:)';
    q0 = RobotOptRes_i.q0_pareto(Ipar,:)';
  elseif ~isempty(PSO_Detail_Data_i)
    % Suche in den Detail-Daten. Nehme beide Kriterien, damit die Daten
    % eindeutig sind. Ansonsten können mehrere Parametersätze teilweise die
    % gleichen Zielkriterien erzeugen
    PosErrMatrix = reshape(PSO_Detail_Data_i.physval(:,kk1,:), ...
      size(PSO_Detail_Data_i.physval,1), size(PSO_Detail_Data_i.physval,3));
    ActForceMatrix = reshape(PSO_Detail_Data_i.physval(:,kk2,:), ...
      size(PSO_Detail_Data_i.physval,1), size(PSO_Detail_Data_i.physval,3));
    k = find( ...
      abs(PosErrMatrix(:)  -data_i.pt_i.PosAcc(inearest))<1e-10 & ...
      abs(ActForceMatrix(:)-data_i.pt_i.ActForce(inearest))<1e-10);
    if length(k) > 1 % Es gibt mehrere Partikel mit genau diesen Werten für die Zielkriterien
      % Prüfe, ob wenigstens die Parameter unterschiedlich sind.
      pval_all = NaN(length(k), size(PSO_Detail_Data_i.pval,2));
      for ii = 1:length(k)
        [ii_ind,ii_gen] = ind2sub(fliplr(size(PSO_Detail_Data_i.comptime)),k(ii));
        pval_all(ii,:) = PSO_Detail_Data_i.pval(ii_ind,:,ii_gen);
      end
      if size(unique(pval_all,'rows'), 1) == 1
        k = k(1); % sind unterschiedlich. Also identische Roboter.
      else % nicht unterschiedlich. Prinzipielles Problem (redundante Parameter)
        error('Suchkriterium in Daten ist nicht eindeutig');
      end
    end
    [k_ind,k_gen] = ind2sub(fliplr(size(PSO_Detail_Data_i.comptime)),k);
    physval = PSO_Detail_Data_i.physval(k_ind,:,k_gen)';
    if abs(physval(kk1)-data_i.pt_i.PosAcc(inearest))>1e-10
      error('Gesuchter Wert konnte nicht gefunden werden. Logik-Fehler');
    end
    fval = PSO_Detail_Data_i.fval(k_ind,:,k_gen)';
    pval = PSO_Detail_Data_i.pval(k_ind,:,k_gen)';
    pval_desopt = PSO_Detail_Data_i.desopt_pval(k_ind,:,k_gen)';
    q0 = PSO_Detail_Data_i.q0_ik(k_ind,:,k_gen)';
  else
    error(['Ergebnis-Partikel liegt nicht in der finalen Pareto-Front ', ...
      'der Optimierung. Detail-Auswertung notwendig. Datei aber nicht da: %s'], ...
      resfile2);
  end

  if ~isempty(PSO_Detail_Data_i)
    % Lese die IK-Anfangswerte aus den Ergebnissen aus (sind indirekt in ge-
    % speicherten Zwischenwerten enthalten)
    [k_gen, k_ind] = cds_load_particle_details(PSO_Detail_Data_i, fval);
    pval_test = PSO_Detail_Data_i.pval(k_ind,:,k_gen)' - pval;
    if any(abs(pval_test))
      error('Indizies für Generation/Individuum stimmen nicht (Parameter nicht gleich)');
    end
    q0 = PSO_Detail_Data_i.q0_ik(k_ind,:,k_gen)';
  end
  %% Nachrechnen der Fitness-Funktion
  % um die Gelenkwinkel aus der IK zu erhalten. Annahme: Die gespeicherten 
  % Detail-Informationen stehen aus Speicherplatzgründen nicht zur Verfügung.
  parroblib_addtopath({RobName}); % Für Ausführung der Fitness-Fcn
  if regenerate_templates
    parroblib_create_template_functions({RobName}, false); % Für Erstellung fehlender Dateien
    R_test = parroblib_create_robot_class(RobName, 1, 1);
    R_test.fill_fcn_handles(true, true); % Zur Kompilierung fehlender Funktionen zum Nachrechnen der Fitness-Funktion
  else
    parroblib_update_template_functions({RobName});
  end
  [R, Structure] = cds_dimsynth_robot(Set_i, d1.Traj, d1.Structures{LfdNr}, true);
  pval = cds_parameters_update(RobotOptRes_i.Structure, ...
    Structure, pval); % Für Aktualisierung des Programms mit neuen Parametern
  cds_update_robot_parameters(R, Set_i, Structure, pval);
  % Fitness-Funktion neu definieren (mit weniger Log-Ausgaben)
  Set = Set_i;
  kk1 = strcmp(Set.optimization.objective,'actforce');
  Set.general.plot_details_in_fitness = 0; % debug: 1e10
  Set.general.save_robot_details_plot_fitness_file_extensions = {};
  Set.general.verbosity = 4; % debug: 3
  Set.optimization.resdir = i4sdg2023_dimsynth_data_dir();
  if recreate_redundancy_map
    resdir_tmp = fullfile(Set.optimization.resdir, Set.optimization.optname, ...
      'tmp', sprintf('%d_%s', Structure.Number, Structure.Name));
    mkdirs(resdir_tmp);
    Set.general.debug_taskred_perfmap = 1;
  end
  Set.general.save_robot_details_plot_fitness_file_extensions = {'png', 'fig'};
  % Schreibe temporäre Daten dahin, wo auch die Ursprungsdaten liegen.
  % Überschreibt nichts.

  cds_log(0, '', 'init', Set);
  % IK-Anfangswerte für dieses Partikel setzen
  if ~isempty(PSO_Detail_Data_i) % geht nur, wenn Detail-Daten vorliegen.
    for iLeg = 1:R.NLEG
      R.Leg(iLeg).qref = q0(R.I1J_LEG(iLeg):R.I2J_LEG(iLeg));
    end
  end
  if recalc_fitnessfcn
    % Funktionsaufruf siehe cds_check_results_reproducability.m
    Structure_tmp = Structure;
    Structure_tmp.calc_dyn_act = Structure.calc_dyn_act | Structure.calc_dyn_reg;
    Structure_tmp.calc_spring_act = Structure.calc_spring_act | Structure.calc_spring_reg;
    Structure_tmp.calc_spring_reg = false;
    Structure_tmp.calc_dyn_reg = false;
  %   Set.optimization.joint_limits_symmetric_prismatic = false; % relaxieren, nicht so wichtig für Bild
  %   Set.optimization.max_range_passive_universal = 150*pi/180;
    % Erzwinge Prüfung dieses Anfangswerts für Trajektorie (falls IK anderes
    % Ergebnis hat). Diese Option sollte nicht notwendig sein. Ist sie leider
    % teilweise.
    Structure_tmp.q0_traj = q0;
    Set.optimization.pos_ik_abort_on_success = true;
    Set.optimization.traj_ik_abort_on_success = true; % Sofort aufhören
    cds_save_particle_details(); cds_fitness();
    [fval_i_test, physval_i_test, Q] = cds_fitness(R, Set, d1.Traj, ...
      Structure_tmp, pval, pval_desopt);
    PSO_Detail_Data_tmp = cds_save_particle_details(Set, R, 0, 0, NaN, NaN, NaN, NaN, 'output');
    condJ = PSO_Detail_Data_tmp.constraint_obj_val(1, 4, 1);  
    if any(fval_i_test > 1e3)
      warning('Die Nebenbedingungen wurden bei erneuter Prüfung verletzt');
      continue
    end
    q0_neu = PSO_Detail_Data_tmp.q0_ik(1,:,1)';
    test_fval = fval - fval_i_test;
    test_physval = physval - physval_i_test;
    if abs(test_fval(kk1)) > 1e-6 % Durch geänderte Implementierung möglich
      warning(['Die Antriebskraft hat einen anderen Wert beim neu nachrechnen ', ...
        '(%1.1f vs %1.1f; Diff %1.1e). Physikalischer Wert: %1.4f vs %1.4f ', ...
        '(Diff %1.1e). IK hat anderes Ergebnis!'], fval_i_test(kk1), fval(kk1), ...
        test_fval(kk1), physval_i_test(kk1), physval(kk1), test_physval(kk1));
  %     if abs(test_physval(kk1)) > 5 % 5N wird noch für das Bild toleriert (Annahme: Auf Cluster war es richtig)
  %       error('Der Fehler ist zu groß. Das lässt sich nicht mehr mit Zufallszahlen-Toleranz erklären');
  %     end
    end
    if any(abs(test_fval(~kk1)) > 1e-5)
      warning(['Andere Zielfunktionen haben einen anderen Wert beim neu nachrechnen: ', ...
        '[%s] vs [%s]. Physikalisch: [%s] vs [%s]'], ...
        disp_array(fval_i_test(~kk1),'%1.6e'), disp_array(fval(~kk1),'%1.6f'), ...
        disp_array(physval_i_test(~kk1),'%1.3e'), disp_array(physval(~kk1),'%1.3e'));
    end
  else
    Q = q0';
  end
  %% Abschließende Berechnungen und Abspeichern
  % Speichere die Trajektorie in der Variable X (für späteres Plotten)
  Traj_0 = cds_transform_traj(R, d1.Traj);
  X = Traj_0.X;
  % Ergebnis-Variable abspeichern
  save(fullfile(datadir, sprintf('detail_result_group_%s_%s.mat', GroupName, usr_figselection)), ...
    'R', 'pval', 'fval', 'physval', 'Structure', 'Q', 'X', ... % Daten zum Ergebnis
    'OptName', 'RobName', 'LfdNr', 'Ipar'); % Herkunft des Ergebnisses
  group_i_success = true;
  break; % Erfolgreich reproduziert
  end % for iinearest
  assert(group_i_success, 'Kein Erfolg bei Reproduktion');
end
fprintf('Je ein Ergebnis aus %d verschiedenen Gruppen ausgewählt und gespeichert\n', size(RobotGroups,1));
