% Postprocess the performance map figure created in the synthesis toolbox
% 
% Before execution, eval_existing_design.m has to be run.
% 
% This script is based on the same file perfmap_fig.m from 
% https://github.com/SchapplM/robotics-paper_ark2022_3T1R/
% (ARK paper "Inverse Kinematics for Task Redundancy of Symmetric 3T1R
% Parallel Manipulators using Tait-Bryan-Angle Kinematic Constraints",
% Schappler 2022) 

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
% Gebe das Kriterium für die Redundanzkarte vor
cs = i4sdg2023_eval_config();
usr_pm_criterion = cs.perfmap_criterion; %'colldist'; % Möglichkeiten: positionerror, colldist, cond_jac
%% Initialization
repodir = fileparts(which('i4sdg2023_dimsynth_data_dir'));
if isempty(repodir)
  error(['You have to create a file i4sdg2023_dimsynth_data_dir pointing to the ', ...
    'directory containing the results of the dimensional synthesis']);
end
resdirtotal = i4sdg2023_dimsynth_data_dir();

% Muss konsistent mit eval_existing_design.m sein.
OptName = cs.optnames{1}; % 'amunpkm_20230107_sdgpaper_instspc_colldist_ikobj2';
LfdNr = 7;
RobName = 'P6RRRRRR10V6G6P1A1';

datadir = fullfile(fullfile(repodir,'data'));
paperfigdir = fullfile(repodir, 'paper', 'figures');

%% Create one performance map for the existing design
fprintf('Zeichne Redundanzkarte\n');
% Pfad zum temporären Optimierungsordner für die Reproduktion aus Skript
% eval_existing_design.m
tmpdir_i = fullfile(datadir, cs.existingdesign_dataname, ...
  cs.existingdesign_resname, 'tmp', sprintf('%d_%s', LfdNr, RobName));
assert(exist(tmpdir_i, 'file'), 'Tmp-Verzeichnis existiert nicht');
fprintf('Ergebnis-Ordner: %s\n', tmpdir_i);
% Einstellungen generieren
setfile = dir(fullfile(resdirtotal, OptName, '*settings.mat'));
d1 = load(fullfile(resdirtotal, OptName, setfile(1).name));
Set_i = cds_settings_update(d1.Set);
Set_i.general.save_robot_details_plot_fitness_file_extensions = {}; % Keine Bilder von Redundanzkarte separat speichern
[R, Structure_i] = cds_dimsynth_robot(Set_i, d1.Traj, d1.Structures{LfdNr}, true);

trajdatafiles = dir(fullfile(tmpdir_i, '*_Traj*.mat'));
perfmapfiles = dir(fullfile(tmpdir_i, '*Konfig*TaskRedPerfMap_Data.mat'));
trajstats = array2table(NaN(length(trajdatafiles),4), 'VariableNames', ...
  {'ConfigNum', 'perfmapfileidx', 'Fval', 'TrajNum'});

% wn_sel = zeros(R.idx_ik_length.wntraj,1);
PHIz_traj = NaN(length(d1.Traj.t), length(trajdatafiles));
TrajLegendText = cell(1,2);
for k = 1:length(trajdatafiles)
  tmp = load(fullfile(tmpdir_i, trajdatafiles(k).name));
  [tokens,~] = regexp(trajdatafiles(k).name, 'Konfig(\d)+', 'tokens', 'match');
  confignum = str2double(tokens{1}{1});
  perfmapfileidx = find(contains({perfmapfiles.name}, sprintf('Konfig%d', confignum)));
  row_k = array2table([confignum, perfmapfileidx, tmp.fval, tmp.i_ar-1]);
  row_k.Properties.VariableNames = trajstats.Properties.VariableNames;
  trajstats(k,:) = row_k;
  PHIz_traj(:,k) = tmp.PM_phiz_plot(:,end);
  TrajLegendText{k} = sprintf('Traj. %d', k);
end
k_iO = trajstats.Fval <= 1e3;
k_plot = find(k_iO, 1, 'first');
% Wähle weitere i.O.-Trajektorien dieser Konfiguration
k_iOc = k_iO & trajstats.ConfigNum == trajstats.ConfigNum(k_plot);
fprintf('%d/%d Trajektorien führen zu erfolgreichem Ergebnis. %d für diese Konfiguration.\n', ...
  sum(k_iO), length(trajdatafiles), sum(k_iOc));

% Redundanzkarte laden (passend zur Nummer der Konfiguration)
if isempty(perfmapfiles), error('Datei nicht gefunden'); end
dpm = load(fullfile(tmpdir_i, perfmapfiles(trajstats.perfmapfileidx(k_plot)).name));

wn_ik = zeros(R.idx_ik_length.wnpos,1);
wn_phys = zeros(4,1);
critnames_withphys = [fields(R.idx_ikpos_wn)', ... % siehe cds_constraints_traj und cds_debug_taskred_perfmap
  {'coll_phys', 'instspc_phys', 'cond_ik_phys', 'cond_phys'}];
wn_perfmap = [wn_ik; wn_phys];
if strcmp(usr_pm_criterion, 'positionerror')
  wn_perfmap(strcmp(critnames_withphys, 'poserr_ee')) = 1;
elseif strcmp(usr_pm_criterion, 'colldist')
  wn_perfmap(strcmp(critnames_withphys, 'coll_phys')) = 1;
elseif strcmp(usr_pm_criterion, 'cond_jac')
  wn_perfmap(strcmp(critnames_withphys, 'cond_phys')) = 1;
else
  error('Fall nicht definiert');
end
perfmap_settings = struct('wn', wn_perfmap, 'TrajLegendText', {{}}, ...
  'i_ar', 0, 'name_prefix_ardbg', '', 'fval', 0, ...
  'logscale', false, ...
  'critnames', {critnames_withphys}, 'constrvioltext', '');
if strcmp(usr_pm_criterion, 'positionerror')
  H_poserr = dpm.H_all(:,:,R.idx_ikpos_hn.poserr_ee);
  fprintf('Maximaler Positionsfehler in Redundanzkarte: %1.1fmm\n', 1e3*max(H_poserr(:)));
  % Begrenze den Positionsfehler auf 0.5mm, damit die Farbskala einheitlich ist
  hpe_max_pm = 0.5e-3;
  H_poserr(H_poserr>hpe_max_pm) = hpe_max_pm;
  colorscale = 1e6 / 100; % Skalierung in 100 µm
  dpm.H_all(:,:,R.idx_ikpos_hn.poserr_ee) = colorscale*H_poserr;
  perfmap_settings.condsat_limit = colorscale*hpe_max_pm;
  perfmap_settings.colorlimit = 1.5*colorscale*hpe_max_pm;
  perfmap_settings.logscale = true;
elseif strcmp(usr_pm_criterion, 'colldist')
  H_colldist = dpm.H_all(:,:,strcmp(critnames_withphys, 'coll_phys'));
  fprintf('Maximaler Kollisionsabstand in Redundanzkarte: %1.1fmm\n', 1e3*max(H_colldist(:)));
  colorscale = 1e3; % Skalierung in mm
  dpm.H_all(:,:,strcmp(critnames_withphys, 'coll_phys')) = colorscale*H_colldist;
  perfmap_settings.logscale = false; % Werte werden nicht übermäßig groß
elseif strcmp(usr_pm_criterion, 'cond_jac')
  perfmap_settings.logscale = true;
end

% Bild plotten (ohne Trajektorien)
fighdl = cds_debug_taskred_perfmap(Set_i, Structure_i, dpm.H_all, dpm.s_ref, ...
  dpm.s_tref, dpm.phiz_range, NaN(length(dpm.s_tref),0), NaN(length(dpm.s_tref),0), ...
  perfmap_settings);

% Zeichne Trajektorien ein (die für diese Konfiguration gelten)
k_iOc(2:end) = false; % nur die erste zeichnen (sind aktuell identisch)
for k = find(k_iOc)'
  trajhdl = plot(dpm.s_tref, 180/pi*PHIz_traj(:,k), 'k-', 'LineWidth', 2);
  set(trajhdl, 'DisplayName', 'Traj');
end

% Zeichne Grenzen ein
for limval = (-51.81 + [-20, +36]) % TODO: Muss konsistent mit Trajektorie sein (siehe eval_existing_design.m)
  limhdl = plot(dpm.s_tref([1; end]), limval*[1;1], 'k--', 'LineWidth', 1);
end
set(limhdl, 'DisplayName', 'PlatformLimit');

sgtitle(''); title(''); % erst hiernach children bestimmen
set(fighdl, 'Name', 'Performance_Map_Paper')
fch = get(fighdl, 'children');
axhdl = fch(strcmp(get(fch, 'type'), 'axes'));
axch = get(axhdl, 'children');
legdummyhdl = [];
leglbl = {};
% Legende zusammenstellen. TODO: Funktioniert noch nicht wie gewünscht.
for jj = 1:length(axch)
  set(axch(jj), 'MarkerSize', 1)
  if strcmp(get(axch(jj), 'DisplayName'), 'Joint Lim')
    delete(axch(jj)); % Gelenkgrenzen hier nicht verwertbar
    continue
  end
  if strcmp(get(axch(jj), 'DisplayName'), 'Act. Sing.')
    set(axch(jj), 'DisplayName', 'Singularity Type II')
  end
  if strcmp(get(axch(jj), 'DisplayName'), 'IK Sing.')
    set(axch(jj), 'DisplayName', 'Singularity Type I')
  end
  if ~strcmp(get(axch(jj), 'Marker'), 'none')
    % Nur Einträge mit Marker
    leglbl = [leglbl, get(axch(jj), 'DisplayName')];
    legdummyhdl = [legdummyhdl; plot(NaN,NaN)]; %#ok<AGROW> 
    set(legdummyhdl(end), 'Marker', get(axch(jj), 'Marker'));
    set(legdummyhdl(end), 'MarkerSize', 5);
    set(legdummyhdl(end), 'DisplayName', get(axch(jj), 'DisplayName'));
    set(legdummyhdl(end), 'Color', get(axch(jj), 'Color'));
    set(legdummyhdl(end), 'LineStyle', get(axch(jj), 'LineStyle'));
  end
end
% Trajektorie manuell anhängen
legdummyhdl = [legdummyhdl; trajhdl];
leglbl = [leglbl, 'Trajectory'];
legdummyhdl = [legdummyhdl; limhdl];
leglbl = [leglbl, 'Platform Limits'];

% legdummyhdl(strcmp(get(legdummyhdl, 'DisplayName'), 'Traj')) = trajhdl;

% Bestimme Grenzen des Datenbereichs
%   Hcond = dpm.H_all(:,:,R.idx_ikpos_wn.jac_cond);
%   all(isnan(Hcond),2)
% ylim(180/pi*minmax2(PHIz_traj(:,k_plot)')+[-30, +70])
ylim([-120 60])

% Speichere das Bild im Paper-Format
figure_format_publication(fighdl);
set(gca, 'xticklabel', {});
set_size_plot_subplot(fighdl, ...
  11.6,4,axhdl,...
  0.10,0.16,0.14,0.12,... %l r u d
  0,0) % x y
xlabel('Normalized trajectory progress $s$', 'interpreter', 'latex');
ylabel('Redundant coord. $\varphi_z$ in deg', 'interpreter', 'latex');
cbhdl = fch(strcmp(get(fch, 'type'), 'colorbar'));
cbyh = ylabel(cbhdl,'TODO', 'Rotation',90);
if strcmp(usr_pm_criterion, 'positionerror')
  set(cbhdl, 'Ticks', [100, 150, 200:100:500]/100);
  set(cbyh, 'String', 'Position error in 100 µm');
elseif strcmp(usr_pm_criterion, 'colldist')
  set(cbyh, 'String', 'Collision distance in mm');
elseif strcmp(usr_pm_criterion, 'cond_jac')
  set(cbhdl, 'Ticks', [1e1, 1e2, 1e3, 1e4, 1e5], 'TickLabels', ...
    {'10', '10^{2}', '10^{3}', '10^{4}', '10^{5}'})
  set(cbyh, 'String', 'Jacobian condition number');
  set(cbhdl, 'Limits', [100, 1e5]);
else
  error('Fall nicht definiert');
end
%   [leghdl, objsb] = legend(legdummyhdl);
%   leghdl = fch(strcmp(get(fch, 'type'), 'legend'));
%   set(leghdl, 'orientation', 'horizontal', 'position', [0.15,0.92,0.7,0.05]);
%   delete(leghdl)
lfhdl = legendflex(legdummyhdl, leglbl, 'anchor', {'n','n'}, ...
  'ref', fighdl, ... % an Figure ausrichten (mitten oben)
  'buffer', [0 -1], ... % Kein Versatz notwendig, da mittig oben
  'ncol', 0, 'nrow', 1, ... % eine Zeile für Legende
  'fontsize', 8, ...
  'xscale', 0.6, ... % Kleine Symbole
  ... 'padding', [0,1,1], ... % Leerraum reduzieren
  'box', 'on');
% Schriftart der Legende aktualisieren
figure_format_publication(fighdl);
drawnow();

t1 = tic();
obj_names = unique([cs.pareto_obj1, cs.pareto_obj2]);
objstr = '';
for i = 1:length(obj_names)
  objstr = [objstr, obj_names{i}];
end
% For this to work, the Java heap memory has to be high enough.
exportgraphics(fighdl, fullfile(paperfigdir, sprintf( ...
  sprintf('perfmap_%s_obj_%s.pdf', usr_pm_criterion, objstr))),'ContentType','vector');
saveas(fighdl, fullfile(paperfigdir, sprintf( ...
  sprintf('perfmap_%s_obj_%s.fig', usr_pm_criterion, objstr))));
fprintf('Exported performance map as vector graphics. Duration: %1.1fs\n', toc(t1));


%% Erzeuge noch eine kleine Version von dem Bild für 1x2 Darstellung
set_size_plot_subplot(fighdl, ...
  6.5,5,axhdl,...
  0.15,0.21,0.2,0.10,... %l r u d
  0,0) % x y
delete(lfhdl);
lfhdl2 = legendflex(legdummyhdl, leglbl, 'anchor', {'n','n'}, ...
  'ref', fighdl, ... % an Figure ausrichten (mitten oben)
  'buffer', [15 -1], ... % eher nach rechts schieben
  'ncol', 0, 'nrow', 2, ... % eine Zeile für Legende
  'fontsize', 8, ...
  'xscale', 0.5, ... % Kleine Symbole
  ... 'padding', [0,1,1], ... % Leerraum reduzieren
  'box', 'on');
axis(axhdl); % Sonst Bezug auf LegendFlex-Handle

[x_off, x_slope] = get_relative_position_in_axes(gca, 'x');
[y_off, y_slope] = get_relative_position_in_axes(gca, 'y');
% y-Beschriftung nach oben schieben
ylh = get(gca, 'ylabel');
set(ylh, 'Position', [x_off+x_slope*(-1.25), y_off+y_slope*(.25) 0]);

% Beschriftung "(b)" einfügen
thdl = text(0,0,'(b)');
xlh = get(gca, 'xlabel');
set(thdl, 'Position', [x_off+x_slope*(-1.45), y_off+y_slope*(-1.12), 0]);
set(thdl, 'FontWeight', 'bold');
figure_format_publication(); % Schriftart ändern

exportgraphics(fighdl, fullfile(paperfigdir, sprintf( ...
  sprintf('perfmap_%s_obj_%s_small.pdf', usr_pm_criterion, objstr))),'ContentType','vector');
saveas(fighdl, fullfile(paperfigdir, sprintf( ...
  sprintf('perfmap_%s_obj_%s_small.fig', usr_pm_criterion, objstr))));
