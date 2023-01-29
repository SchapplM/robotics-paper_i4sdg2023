% Erzeuge hochaufgelöste Bilder der einzelnen Roboter aus den
% Optimierungsergebnissen.
% 
% Vorher ausführen:
% * Aggregation der Daten mit eval_figures_pareto_groups.m
% * Zusammenstellen der Details mit select_eval_robot_examples.m
% 
% This script is based on the same file eval_figures_pareto.m from 
% https://github.com/SchapplM/robsynth-paper_mhi2021
% (MHI paper "Combined Structural and Dimensional Synthesis of a Parallel
% Robot for Cryogenic Handling Tasks", Schappler et al. 2022)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
close all

usr_figselection = 'actvelo_actforce'; % siehe andere Skripte

warning('off', 'Coder:MATLAB:rankDeficientMatrix'); % für invkin2
repo_dir = fileparts(which('i4sdg2023_dimsynth_data_dir.m'));
outputdir = fullfile(repo_dir, 'paper', 'figures', 'robots');
datadir = fullfile(repo_dir,'data');
importdir = i4sdg2023_dimsynth_data_dir();
tmp = load(fullfile(datadir, sprintf('robot_groups.mat')));
RobotGroups = tmp.RobotGroups;
namestablepath = fullfile(datadir, 'robot_names_latex.csv');
ResTab_NameTrans = readtable(namestablepath, 'Delimiter', ';');

%% Alle Gruppen durchgehen
countrob = 0; % Nummern aus Legende
for i = 1:size(RobotGroups,1)
  GroupName = RobotGroups{i,1};
  if RobotGroups{i,3} == 0, continue; end % keine Ergebnisse vorliegend
  fprintf('Zeichne Bild für PKM-Gruppe %d/%d (%s)\n', i, size(RobotGroups,1), GroupName);
  erg = load(fullfile(datadir, sprintf('detail_result_group_%s_%s.mat', GroupName, usr_figselection)));
  % countrob = countrob + 1; % nur gültige PKM-Gruppen zählen
  countrob = i; % Geht nur, wenn alle Gruppen gültig sind.
  % Daten der Ergebnisse laden
  fprintf('Anzahl Variablen: %d\n', length(erg.Structure.vartypes));
  disp(strjoin(erg.Structure.varnames));
  fprintf('Parametertypen: [%s]\n', disp_array(erg.Structure.vartypes', '%1.0f'));
  close all
  setfile = dir(fullfile(importdir, erg.OptName, '*settings.mat'));
  assert(~isempty(setfile), sprintf('Für Optimierung %s fehlt die Einstellungsdatei', erg.OptName));
  d1 = load(fullfile(importdir, erg.OptName, setfile(1).name));
  Set_i = cds_settings_update(d1.Set);
  %% Roboter initialisieren
  R = erg.R;
  % Dicke der Segmente reduzieren
  for kk = 1:R.NLEG
    R.Leg(kk).DesPar.seg_par(:,1) = 1e-3;
    R.Leg(kk).DesPar.seg_par(:,2) = 1e-2;
  end
  parroblib_addtopath({R.mdlname});
  %% Anpassung der Visualisierung
  Q = erg.Q;
  X = erg.X;
  Q_plot = Q;
  X_plot = X;

  %% Bild zeichnen und Formatieren
  change_current_figure(1); clf; hold on;
  s_plot = struct('ks_legs', [], 'ks_platform', [], 'mode', 4);
  % Plotte in mittlerer Stellung des Roboters
  x_mean = mean(minmax2(X_plot(:,1:3)'),2);
  % Finde den Punkt der Trajektorie, der der Mitte am nächsten ist.
  if size(Q,1) == 1
    warning('Keine Trajektorie gegeben')
    I_plot = 1; % Nehme den ersten Punkt
  else
    % Nehme den untersten Punkt der Trajektorie
    [xmin,I_plot] = max(X_plot(:,3));
    % Alternativ: Nehme einen Punkt aus der Mitte
%     [~,I_plot] = min(sum((repmat(x_mean',size(X_plot,1),1)-X_plot(:,1:3)).^2,2));
  end
  R.plot(Q_plot(I_plot,:)', X_plot(I_plot,:)', s_plot);
  view(3);
  title('');xlabel('');ylabel('');zlabel('');
  LegColors = [ [0 1 0]; [0 0 0]; [1 0 1]; [0 0 1]; [1 0 0]; [0 1 1] ]; % Grün, Schwarz, Violett, blau, rot, cyan
  % Automatisch generierten Plot nachverarbeiten
  ch = get(gca, 'Children');
  for jj = 1:length(ch)
    % KS entfernen
    if strcmp(ch(jj).Type, 'hgtransform')
      delete(ch(jj)); continue
    end
    % Weise den Beinketten neue Farben zu
    for kk = 1:R.NLEG
      if contains(get(ch(jj), 'DisplayName'), sprintf('Leg_%d_Link', kk))
        set(ch(jj), 'EdgeColor', LegColors(kk,:));
      end
    end 
  end

  % Zusätzlich Bauraum als Rahmenlinien einzeichnen.
  % Umrechnen in Format der plot-Funktion
  q_W = Set_i.task.installspace.params(1:3)';
  u1_W = Set_i.task.installspace.params(4:6)';
  u2_W = Set_i.task.installspace.params(7:9)';
  % letzte Kante per Definition senkrecht auf anderen beiden.
  u3_W = cross(u1_W,u2_W); u3_W = u3_W/norm(u3_W)*Set_i.task.installspace.params(10);
  u3_W(3) = 1.2; % Reduziere Ausdehnung nach oben
  cubpar_c = q_W(:)+(u1_W(:)+u2_W(:)+u3_W(:))/2; % Mittelpunkt des Quaders
  cubpar_l = [norm(u1_W); norm(u2_W); norm(u3_W)]; % Dimension des Quaders
  cubpar_a = 180/pi*tr2rpy([u1_W(:)/norm(u1_W), u2_W(:)/norm(u2_W), u3_W(:)/norm(u3_W)],'zyx')'; % Orientierung des Quaders
  drawCuboid([cubpar_c', cubpar_l', cubpar_a'], ...
    'FaceColor', 'c', 'FaceAlpha', 0.01);

  % Dummy-Plot für Legende mit Roboter-Marker
  hdl=plot3(0,0,NaN, RobotGroups{i,4});
  % abgespeicherten Marker-Typ einzeichnen (zur Wiedererkennung beim
  % Zusammenstellen der Bilder)
  % lh = legend(hdl, legstr_i, 'interpreter', 'latex'); 
  % set(lh, 'location', 'northeastoutside');
  
  % Nehme überall die gleiche Perspektive für Vergleichbarkeit der Bilder?
  view(42,5); % Standard-Perspektive
  % Manuelle Anpassungen für einzelne Bilder möglichst wenig Verdeckungen)
%   if strcmp(GroupName, 'P6RRRRRR8V2G')
%     view(51,12);
%   elseif strcmp(GroupName, 'P6RRRRRR10V3G')
%     view(16,22);
%   elseif strcmp(GroupName, 'P6RRRRRR10V6G')
%     view(24,37);
%   elseif strcmp(GroupName, 'P6RRRRRR5V2G')
%     view(67,20);
%   elseif strcmp(GroupName, 'P6RRRRRR6V2G')
%     view(76,19);
%   else
%     warning('Keine manuell gewählte Perspektive. Nehme Standard.')
%   end

  set(gca,'XTICKLABEL',{});set(gca,'YTICKLABEL', {});set(gca,'ZTICKLABEL',{});
  set(gca,'xtick',[],'ytick',[],'ztick',[]);
  set(get(gca, 'XAxis'), 'visible', 'off');
  set(get(gca, 'YAxis'), 'visible', 'off');
  set(get(gca, 'ZAxis'), 'visible', 'off');
  figure_format_publication(gca);
  set(gca, 'Box', 'off');
  set(1, 'windowstyle', 'normal');
  set_size_plot_subplot(1, ...
    8,8,gca,...
    0,0,0,0,0,0)
  drawnow();
  % Bild speichern
  name = sprintf('RobotFig_PlotNum%d_RobGroup%d_%s', countrob, i, GroupName);
  cd(outputdir);
  export_fig([name, '_r864.png'], '-r864');
  savefig([name, '.fig'])
  % export_fig([name, '.pdf']);
end

%% Zeichne die Marker in jeweils ein eigenes Bild
return % muss nur einmal gemacht werden
for i = 1:size(RobotGroups,1) %#ok<UNRCH>
  change_current_figure(1000);clf;
  if RobotGroups{i,3} == 0, continue; end
  plot(1,1,RobotGroups{i,4});
  figure_format_publication(gca);
  set(gcf, 'color', 'none'); % transparent. Funktioniert nicht in pdf.
  set(gca, 'Box', 'off');
  set(gca, 'XTICK', [], 'YTICK', []);
  set(get(gca, 'XAXIS'), 'visible', 'off');
  set(get(gca, 'YAXIS'), 'visible', 'off');
  set_size_plot_subplot(1000, ...
    1,1,gca,0,0,0,0,0,0)
  export_fig(fullfile(outputdir, sprintf('group%d_marker.pdf', i)));
  cd(outputdir);
  export_fig(sprintf('group%d_marker.png', i), '-r864');
end

fprintf('Marker nach %s exportiert\n', outputdir);
