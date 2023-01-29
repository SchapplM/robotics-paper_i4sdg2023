% Kombinierte Struktur- und Maßsynthese für Unkraut-PKM (AMUN-Projekt)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

clc
clear

hexa_test = false; % Debug-Schalter zur detaillierten Inspektion einer einzigen PKM
% Name der Optimierung:
configset = struct('optname', 'amunpkm_20230108_sdgpaper_plffix2');
if hexa_test
  configset = struct('optname', 'amunpkm_20230108_sdgpaper_dbg3_Hexa_perfmap');
end

%% Einstellungen für Programmfluss
% Fülle Variable für spezielle AMUN-Maßsynthese-Einstellungen zusätzlich zu
% der Standard-Struktur für die kombinierte Synthese

% Schalter zum Nachverarbeiten der vom Cluster heruntergeladenen Ergebnisse
% (Zusammenfassen einer aufgeteilten Berechnung)
configset.merge_results_only = false;
% Berechnung auf Cluster durchführen
configset.cluster = true;
% Parellele Berechnung nutzen
configset.parallel = false; % nur für lokale Berechnung wirksam
% Für abgebrochene Berechnungen oder nachträgliche Auswertung:
configset.repair = false; % Nach abgebrochenem Durchlauf: Zusammenfassung neu generieren (aus abgeschlossenen Optimierungen)
configset.finish_aborted = false; % nach abgebrochenem Durchlauf auf Cluster: Unfertige Optimierungen mit vorläufigem Stand abschließen
% Benutze nur PKM, die vorher schon funktioniert haben (für diese Aufgabe)
configset.loadwhitelist = false;
if hexa_test
  configset.loadwhitelist = false;
end

%% Einstellungen aus Aufgabengeometrie
% Aufgaben-FG
DoF = [1 1 1 1 1 0]; % Mit Schwenkwinkeln, 3T2R-PKM
% DoF = [1 1 1 0 0 0]; % 3T0R-PKM (bzw. aufgabenredundante 3T1R-PKM)

% Einstellungs-Struktur initialisieren
Set = cds_settings_defaults(struct('DoF', DoF));
% Spezifische Einstellungen laden
Set = amun_synth_env(Set);

%% Trajektorie laden
Set.task.Ts = 10e-3; % sample time (muss eher kleiner als 5e-2 gewählt werden)
Set.task.Tv = 1e-1; % Verschliffzeit Beschleunigung (max Ruck)
Set.task.amax = 20;
% Set.task.refTraj = 'weeds';
% Set.task.refTraj = 'pyramid';
Set.task.refTraj = 'random';
% Debug:
% Set.task.profile = 0; % Nur Eckpunkte
% Temporär auf 3T0R stellen, damit keine Schwenkbewegungen durchgeführt
% werden müssen. Dadurch sind 3T0R- und 3T3R-PKM vergleichbar, da sie die
% gleiche Trajektorie durchführen.
% Set.task.DoF = logical([1 1 1 0 0 0]);
Traj = amun_synth_traj(Set.task);
Set.task.DoF = logical(DoF); % Zurücksetzen auf Vorgabe für Strukturauswahl
Set.task.pointing_task = true;
% Debug: Visualisierung der Aufgabe
% cds_show_task(Traj, Set)

%% Sonstige Optimierungseinstellungen
configset.num_repetitions = 1; % Anzahl der Wiederholungen der Optimierung
Set.optimization.NumIndividuals = 200;  % default 200. Kleinere Werte erlauben auch sehr kurze Test-Durchführung.
Set.optimization.MaxIter = 100; % default 100; Anzahl Generationen (wird voraussichtlich sowieso vor Ende abgebrochen)

Set.general.verbosity = 4;
Set.general.matfile_verbosity = 3;

% Am Ende von jedem Roboter ein Video erstellen und das Pareto-Bild
Set.general.animation_styles = {'3D'};
Set.general.save_animation_file_extensions = {'mp4'};
Set.general.eval_figures = {'pareto_all_phys', 'pareto'};
Set.general.maxduration_animation = 10;  % Länge der Animation begrenzen

%% Debug-Optionen
% Debug: Wähle Ausmaß der Debug-Plots während der Maßsynthese
% Set.general.plot_details_in_fitness = 4e9; % Selbstkollision
% Set.general.plot_details_in_fitness = 1e4; % Probleme in Traj.
Set.general.plot_robot_in_fitness = 0;
% Debug: Auswertungsbild zum Debuggen der Aufgabenredundanz (dauert sehr
% lange; dient zur Validierung der IK-Methoden @Schappler)
Set.general.debug_taskred_perfmap = false;
if hexa_test
  Set.general.debug_taskred_perfmap = false;
end
% Debug: Bilder für Trajektorien. Auswertung der Aufgabenredundanz. (@Schappler)
% Set.general.debug_taskred_fig = true;
% Debug-Bilder nicht nur generieren, sondern auch speichern
% Set.general.save_robot_details_plot_fitness_file_extensions = {''};
Set.general.save_robot_details_plot_fitness_file_extensions = {'png', 'fig'};

% Debug: Abbruchkriterien so definieren, dass nur eine einzige gültige
% Lösung direkt genommen wird
% Set.optimization.obj_limit = [1e3;1e3;1e3]; % Unter 1e3 ist gültig.

% Vorlagen neu erzeugen, falls veraltete Dateien vorliegen
% Set.general.create_template_functions = true;

%% Strukturauswahl (manuell)
% Auswahl der Strukturen (Beispiele). Wenn leer gelassen, werden alle
% untersucht (gemäß sonstigen Filterkriterien)
if hexa_test
  Set.structures.whitelist = {'P6RRRRRR10V6G6P4A1'}; % Hexa
end
% Set.structures.whitelist = {'P6RRRRRR10V6G6P1A1'}; % Hexa (6RUS)
% Set.structures.whitelist = {'P6RRPRRR14V3G1P4A1'}; % Hexapod
% Set.structures.whitelist = {'P6RRRRRR10V3G6P4A1'}; % Hexa mit Drehgelenken
% Set.structures.whitelist = {'P6PRRRRR6V2G8P1A1'}; % 6PUS (zum Debuggen mit anderer Struktur)
% Set.structures.whitelist = {'P6RRRRRR8V2G8P3A1'}; % RRUU

% Liste von PKM, die gut funktioniert haben
% Set.structures.whitelist = {'P6RRRRRR10V3G6P4A1', 'P6RRRRRR10V5G6P2A1', ...
%   'P6RRRRRR10V7G6P1A1', 'P6RRRRRR8V2G6P3A1'}; %'P6RRRRRR10V7G6P4A1','P6RRRRRR8V4G6P5A1'};
% Nur eine Struktur untersuchen und dafür mehrfach wiederholen
% Set.structures.whitelist = {'P3RRRRR10G2P2A1'};

% Nur eine Struktur
% Set.structures.whitelist = {'P6RRRRRR10V6G6P1A1'};
% Set.structures.repeatlist = {{'P6RRRRRR10V6G6P1A1', 8}};

% Laden der bisher erfolgreichen Versuche. Dadurch weniger Rechenaufwand.
% Vorherige Optimierungen zur Erkundung möglicher PKM sollten möglichst
% vollständig gewesen sein.
Set = load_whitelist(Set, configset);
% wenn keine Strukturen in der Whitelist, dann flag negativ setzen. Nur
% dann funktionieren eingestellte joint-filter
if isempty(Set.structures.whitelist)
    configset.loadwhitelist = false;
end
% Setze Ordner mit bisherigen Ergebnissen für Startwerte der Optimierung.
% Dauert teilweise etwas länger beim Laden, da auf Projektablage
% zugegriffen wird.
if ~configset.cluster
  Set.optimization.result_dirs_for_init_pop = {amun_dimsynth_data_dir()};
end
% Nutze globalen Index für schnelleren Start. Muss vorher aktualisiert sein
Set.optimization.InitPopFromGlobalIndex = true;
%% Strukturauswahl (automatisch)
% Für den Fall, dass keine Positiv-Liste genommen wird:
Set.structures.joint_filter = 'R*****'; % Nur Drehgelenke am Gestell
Set.structures.max_index_active = 1; % erstes Gelenk angetrieben
Set.structures.use_serial = false; % keine seriellen Roboter
if all(DoF == [1 1 1 1 1 0])
  Set.structures.num_tech_joints = 3:4; % Nehme keine Ketten aus 5 oder 6 Drehgelenken (zu kompliziert)
elseif all(DoF == [1 1 1 0 0 0])
%   Set.structures.num_tech_joints = 3:5; % Nehme alle möglichen Fälle (sonst zu wenig verschiedene)
  Set.structures.num_tech_joints = 3:4;
end
% Die Filter für Gestell- und Plattformausrichtung sind aktiv, auch wenn
% die Positiv-Liste gesetzt ist.
if ~configset.loadwhitelist
  Set.structures.parrob_basejointfilter = [2 6]; % nur tangential angeordnete Drehgelenke als Basis
  Set.structures.parrob_platformjointfilter = 1:6; % keine speziellen Gelenkanordnungen für Plattform
end

%% Weitere Rahmenbedingungen der Maßsynthese
% Konditionszahl darf nicht total schlecht sein. Dann werden
% Antriebskräfte und innere Kräfte zu hoch (Erfahrungswerte)
Set.optimization.constraint_obj(4) = 500; % max. Wert für Konditionszahl
% Die Antriebskraft sollte nicht zu groß werden.
Set.optimization.constraint_obj(3) = 100; % max. Wert in Nm
% Der Positionsfehler darf nicht zu groß sein
Set.optimization.constraint_obj(7) = 0.5e-3; % in m, am Endeffektor

% Debug: Nur statische Kräfte und keine Masse der Beinketten annehmen.
Set.optimization.static_force_only = true;
Set.optimization.nolinkmass = false;
Set.optimization.noplatformmass = false;

name_orig = configset.optname;
for ps = 4:10
% Zielkriterien (2 oder 3 für Pareto-Optimierung)
% {'mass','stiffness','energy', 'actforce', 'condition','actforce', 'actvelo', 'installspace','footprint'}
% Antriebskraft und -geschwindigkeit maßgeblich für die Antriebsauslegung
Set.optimization.objective = {'actforce', 'actvelo', 'TODO'};
% Drittes Kriterium sollte so gewählt werden, dass vielseitige Ergebnisse
% rauskommen und Wahl der Lösung in sinnvolle Richtung geht
ps_list = {'mass','stiffness','energy', 'condition', 'installspace', ... % 1..5
  'footprint', 'colldist', 'positionerror', 'power', 'chainlength'}; % 6..10
Set.optimization.objective{3} = ps_list{ps};
configset.optname = sprintf('%s_obj3_%s', name_orig, Set.optimization.objective{3});

if Set.task.profile == 0 % nur Eckpunkte. Keine Geschwindigkeit bestimmbar
  Set.optimization.objective = {'actforce', 'condition'};
end
% IK-Einstellungen in Maßsynthese. Benutze nur die dynamische
% Programmierung mit konstanter Orientierung.
Set.optimization.objective_ik = 'coll_par';
Set.general.taskred_dynprog = true;
Set.general.taskred_dynprog_and_gradproj = false;
Set.general.taskred_dynprog_only = true;
% Annahme: Eine Trajektorie reicht. Geht schneller und größter Teil der
% Leistungsmerkmale wird in Bewegung entschieden. Außerdem ist die erste
% Konfiguration aus cds_constraints die aus Optimierung.
Set.optimization.traj_ik_abort_on_success = true;
% Set.optimization.max_velocity_passive_universal = 1; % Um Abbruch deswegen zu erzwingen
Set.optimization.ee_rotation_limit_rel_traj = [-45, 45]*pi/180;
% Nur wenige Zustände nötig, da Bereich bereits auf 90° eingeschränkt.
% Annahme: 30° reichen als Diskretisierung
% Annahme 2: Es werden ja die Zustandsoptimierungen per Einzelpunkt-IK
% durchgeführt. Das sollte daher deutlch für die DP reichen.
Set.general.taskred_dynprog_numstates = [3 0];

% Einstellungen je nach Programmfluss anpassen
Set = adapt_settings(Set, configset);
%% Starten
start_dimsynth(Set, Traj, configset);
end