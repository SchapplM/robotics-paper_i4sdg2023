% Auswertung des aktuellen Entwurfs zum Vergleich mit der Maßsynthese
% 
% This script is based on the same file perfmap_fig.m from 
% https://github.com/SchapplM/robsynth-paper_mhi2021
% (MHI paper "Combined Structural and Dimensional Synthesis of a Parallel
% Robot for Cryogenic Handling Tasks", Schappler et al. 2022)  

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2022-12
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear
rng(0);
%% Lade Roboterdefinition (aus anderem Versuch, zur Vereinfachung)
repodir = fileparts(which('i4sdg2023_dimsynth_data_dir'));
datadir = fullfile(repodir, 'data');
addpath(fullfile(repodir, 'workspace'));
if isempty(repodir)
  error(['You have to create a file i4sdg2023_dimsynth_data_dir pointing to the ', ...
    'directory containing the results of the dimensional synthesis']);
end
resdirtotal = i4sdg2023_dimsynth_data_dir();
% Maßsynthese-Durchlauf auswählen, dessen Einstellungen benutzt werden,
% um die ausgewählte Parametrierung auszuwerten.
% Sollte übereinstimmen mit Datengrundlage für Pareto-Bilder.
cs = i4sdg2023_eval_config();
% Lade die erste von mehreren möglichen Optimierungen für die Einstellungs-
% datei zum reproduzieren der Ergebnisse (Trajektorie und Einstellungen müssen gleich sein)
OptName = cs.optnames{1}; %'amunpkm_20230107_sdgpaper_instspc_colldist_ikobj2';
% Es muss der Robotertyp gewählt werden, der aufgebaut wurde.
LfdNr = 7;
RobName = 'P6RRRRRR10V6G6P1A1';
% Daten laden
setfile = dir(fullfile(resdirtotal, OptName, '*settings.mat'));
d1 = load(fullfile(resdirtotal, OptName, setfile(1).name));
Set_i = cds_settings_update(d1.Set);
resfile = fullfile(resdirtotal, OptName, sprintf('Rob%d_%s_Endergebnis.mat', ...
  LfdNr, RobName));
tmp = load(resfile);
RobotOptRes_i = tmp.RobotOptRes;
Structure = tmp.RobotOptRes.Structure;

parroblib_update_template_functions({RobName}, 0);
% matlabfcn2mex({[RobName(1:end-6), '_invkin_traj']});
parroblib_addtopath({RobName});
[R, Structure] = cds_dimsynth_robot(Set_i, d1.Traj, Structure, true);

%% Ersetze die Parameter
% Lade existierenden Entwurf
R_ref = load_amun_pkm_realMDH();
vn = Structure.varnames;
pval = NaN(length(vn),1);
pval(strcmp(vn, 'scale')) = 1;
pval(strcmp(vn, 'pkin 1: a2')) = R_ref.Leg(1).MDH.a(2);
pval(strcmp(vn, 'pkin 2: a4')) = R_ref.Leg(1).MDH.a(4);
pval(strcmp(vn, 'pkin 3: alpha2')) = R_ref.Leg(1).MDH.alpha(2);
pval(strcmp(vn, 'pkin 4: alpha4')) = R_ref.Leg(1).MDH.alpha(4);
pval(strcmp(vn, 'pkin 6: d2')) = R_ref.Leg(1).MDH.d(2);
pval(strcmp(vn, 'pkin 7: d4')) = R_ref.Leg(1).MDH.d(4);
pval(strcmp(vn, 'base z')) = R_ref.T_W_0(3,4);
pval(strcmp(vn, 'baserotation z')) = [0,0,1]*r2eulxyz(R_ref.T_W_0(1:3,1:3));
% Aus Besprechung mit Philipp vom 25.09.2020. Reduziere den Gestellradius
% von dort wegen des höheren Sicherheitsbereichs in der kombinierten Synth.
pval(strcmp(vn, 'base radius')) = R_ref.DesPar.base_par(1);
pval(strcmp(vn, 'base_morph_pairdist')) = R_ref.DesPar.base_par(2)/R_ref.DesPar.base_par(1);
pval(strcmp(vn, 'platform radius')) = R_ref.DesPar.platform_par(1);
assert(all(~isnan(pval)), 'p darf nicht NaN sein');
pval_phys_test = cds_update_robot_parameters(R, Set_i, Structure, pval);

assert(all(abs(R.T_W_0(:)-R_ref.T_W_0(:))<1e-6), 'Basis-Transformation ist anders');
assert(strcmp(R.mdlname,R_ref.mdlname), 'Falsches Roboter-Modell');
assert(abs(R.DesPar.base_par(1)-R_ref.DesPar.base_par(1))<1e-6, 'base radius falsch eingetragen');
assert(abs(R.DesPar.base_par(2)-R_ref.DesPar.base_par(2))<1e-6, 'base_morph_pairdist falsch eingetragen');
assert(all(abs(R.r_P_B_all(:)-R_ref.r_P_B_all(:))<1e-6), 'Plattform-Koppelpunkte sind anders')

for i = 1:R.NLEG
  R.Leg(i).qref = R_ref.Leg(i).qref;
end

fprintf('Daten des Roboters:\n');
fprintf('Gestell-Radius: %1.3fmm\n', 1e3*R.DesPar.base_par(1));
fprintf('Plattform-Radius: %1.3fmm\n', 1e3*R.DesPar.platform_par(1));

%% Berechne die Zielfunktion. Dadurch Detail-Kennzahlen zur Kinematik
Set = Set_i;
% Speicherort für Bilder (z.B. Redundanzkarte aus Dynamischer Programmierung)
% Set.optimization.resdir = fullfile(repodir, 'data', 'existing_design_v7');
% Abweichungen von den gespeicherten Einstellungen vornehmen.
% Achtung: Das gefährdet die Vergleichbarkeit der Konstruktionslösung mit
% der aktuellen Maßsynthese
% Debug-Bilder, falls keine Lösung gefunden wird.
% Set.general.plot_details_in_fitness = 1e11;
% Set.general.plot_robot_in_fitness = 1e11;
Set.optimization.pos_ik_abort_on_success = true;
Set.optimization.traj_ik_abort_on_success = true; % Sofort aufhören
Set.general.debug_taskred_perfmap = 1; % Redundanzkarte erzeugen
Set.general.taskred_dynprog = true; % Dynamische Programmierung (DP) benutzen
Set.general.taskred_dynprog_and_gradproj = false; % keine Gradientenprojektion
% Set.general.taskred_dynprog_numstates = [9 9]; % Stützstellen für DP
% Betrachte nur den konstruktiv möglichen Bereich. TODO: Damit keine Lösung
% möglich.
Set.optimization.ee_rotation_limit = (-51.81 + [-20, +36]) * pi/180; % Eingeschränkten Bereich betrachten (da bekannt)
% Set.general.taskred_dynprog_numstates = [11 0]; % Dann glatte 5°-Schritte bei 55° Spannweite
Set.general.taskred_dynprog_numstates = [5 0]; % Dann ca. 10°-Schritte bei 55° Spannweite
Set.optimization.objective_ik = cs.existingdesign_ikopt; % 'coll_par'; % hier anderes Kriterium als 'default' testweise möglich

if strcmp(Set.optimization.objective_ik, 'constant')
  Set.general.taskred_dynprog = false; % Keine DP
end
Set.general.debug_dynprog_files = true; % damit es neu geladen werden kann

Set.optimization.resdir = fullfile(repodir, 'data', cs.existingdesign_dataname); % 'existing_design_collopt'
Set.optimization.optname = cs.existingdesign_resname; % Damit es immer im gleichen Ordner landet
% Debug: 
Set.optimization.max_velocity_ee_rotation = 2*pi; % ist nicht ganz fair, da in Optimierung nicht so schnell erlaubt. Macht hauptsächlich die Redundanzkarte interessanter. Sonst geht nämlich nur konstante Orientierung.
% Set.optimization.constraint_obj(4) = inf;
% Set.optimization.pos_ik_tryhard_num = 100;
% Set.optimization.objective = {'actforce', 'actvelo', 'positionerror'};
% Set.optimization.objective = {'actforce', 'actvelo', 'installspace'};
defstruct = cds_definitions();
% Optimierungskriterien. Minimal: unique([cs.pareto_obj1, cs.pareto_obj2, ... 
% TODO: Das funktioniert hier nicht, da evtl. Bauraum-Schwellwerte
% verändert werden.
Set.optimization.objective = defstruct.obj_names_all; % Berechne alle Kriterien (dann Ergebnis für jede Auswertung nutzbar, sofern nicht 'default' in IK optimiert wird
Set.optimization.obj_limit = zeros(length(Set.optimization.objective), 1);
Set.optimization.obj_limit_physval = Set.optimization.obj_limit;
% Es muss auch die Schnittkraft berechnet werden
Structure.calc_cut = true;
% Ordner für Speicherung von tmp-Bildern erzeugen.
[~,~,~,resdir] = cds_get_new_figure_filenumber(Set, Structure, '');
mkdirs(resdir);
% Ohne die Vorgabe von q0_traj funktionierte es mal nicht. Jetzt doch wieder. Unklar, warum.)
% Structure.q0_traj = cat(1,R_ref.Leg(:).qref);
Set.general.save_robot_details_plot_fitness_file_extensions = {'fig', 'png'}; % Speicherung der Redundanzkarte als Bild
cds_log(0, '', 'init', Set);
cds_save_particle_details(); cds_fitness();
[fval, physval, Q, QD, QDD, TAU, ~, Jinv_ges] = cds_fitness(R, Set,d1.Traj, Structure, pval);
PSO_Detail_Data_tmp = cds_save_particle_details(Set, R, 0, 0, NaN, NaN, NaN, NaN, 'output');
Traj_0 = cds_transform_traj(R, d1.Traj);
X = Traj_0.X;
objective_names = Set_i.optimization.objective;
save(fullfile(datadir, sprintf('%s.mat', cs.existingdesign_resname)), ... % 'detail_result_engineering_solution.mat'
  'R', 'pval', 'fval', 'physval', 'Structure', 'Q', 'Jinv_ges', 'X', ... % Daten zum Ergebnis
  'objective_names', 'Set', 'Traj_0');


%% Statische Traglast berechnen
% siehe https://gitlab.projekt.uni-hannover.de/imes-projekt-dfg_mrkpkm/studentische-arbeiten/studarb_giourgas_antonios_sa/-/blob/main/Versuchsstand_AMUN_Hexa/AMUN_Hexa_Versuchsstand_Init.m
% Massen aus CAD-Modell einfügen (nicht die Standardwerte aus der Synthese)
% Debug: Daten von oben direkt wieder laden (dazu nur Initialisierung von
% oben notwendig, keine Trajektorien-Neuberechnung).
load(fullfile(datadir, sprintf('%s.mat', cs.existingdesign_resname)));
mges1 = [...
    2.0738;     % (KS)1
    0;          % (KS)2
    0.43682;    % (KS)3
    0;          % (KS)4
    0;          % (KS)5
    0;          % (KS)6
    0.766];     % (KS)P
rSges1 = [...
    -0.0185 -0.2870 0.1419;     % (KS)1
    0 0 0;                      % (KS)2
    -0.3698 -0.3807 0.0588;     % (KS)3
    0 0 0;                      % (KS)4
    0 0 0;                      % (KS)5
    0 0 0;                      % (KS)6
    0 0 0];                     % (KS)P
Icges1 =[...
    0.0711 0.0711 0.0711 0 0 0;     % (KS)1
    0 0 0 0 0 0;                    % (KS)2   
    0.0414 0.0414 0.0414 0 0 0;     % (KS)3
    0 0 0 0 0 0;                    % (KS)4
    0 0 0 0 0 0;                    % (KS)5
    0 0 0 0 0 0;                    % (KS)6
    0.0011 0.0011 0.0021 0 0 0];    % (KS)P
R.update_dynpar1(mges1, rSges1, Icges1);

Set.optimization.nolinkmass = false;
Set.optimization.noplatformmass = false;

% Wähle verschiedene Zusatzmassen und bestimme das maximale Drehmoment
m_payload_list = 0:20; % in kg
taumax_list = NaN(length(m_payload_list),1);
for i_pl = 1:length(m_payload_list)
  mges2 = mges1; % Massen, die vorher festgelegt worden sind
  mges2(end) = mges1(end)+m_payload_list(i_pl);
  R.update_dynpar1(mges2, rSges1, Icges1);
  data_dyn_i_pl = cds_obj_dependencies(R, Traj_0, Set, Structure, Q, Q*0, Q*0, Jinv_ges);
  [fval_actforce,~, ~, physval_actforce_i_pl] = cds_obj_actforce(data_dyn_i_pl.TAU);
  taumax_list(i_pl) = physval_actforce_i_pl;
end
% Berechne die theoretische maximale Traglast
max_payload_at50Nm = interp1(taumax_list, m_payload_list, 50, 'spline');
fprintf('Theoretische maximale Traglast (in Referenztrajektorie) bei 50Nm max. Moment: %1.2fkg\n', max_payload_at50Nm);
if abs(max_payload_at50Nm-13)>1, warning('Ergebnis für Traglast im Paper stimmt nicht mehr (wegen geänderter Trajektorie)'); end
% Vergleich. Idealerweise ist das maximale Drehmoment eine Gerade, aber
% durch die Maximum-Funktion in der `cds_obj_actforce` evtl. auch nicht.
figure(1);clf;hold on;
plothdl = NaN(2,1);
plothdl(1)=plot(m_payload_list, taumax_list);
% Trage Wert aus der Synthese ein
synthres_mass = Set.task.payload.m;
synthres_taumax = physval(strcmp(Set.optimization.objective, 'actforce'));
plothdl(2)=plot(synthres_mass, synthres_taumax, 'o');
plothdl(3)=plot(m_payload_list([1;end]), [50;50], 'r-');
plothdl(4)=plot(max_payload_at50Nm*[1;1], [0;50], 'b--');
xlabel('additional payload');
ylabel('maximum joint torque');
grid on;
legend(plothdl,{'with CAD parameters and payload', 'from synthesis (different parameters)', ...
  'max. gear torque', 'max. payload'});
