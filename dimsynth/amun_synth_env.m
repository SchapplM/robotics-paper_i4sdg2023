% Geometrie der Unkraut-Aufgabe definieren: Bauraum, Kollisionskörper
% 
% Eingabe:
% Set
%   Einstellungsvariable aus cds_settings_defaults
% 
% Ausgabe:
% Set
%   wie Eingang, mit mehr Informationen (durchgeschleift)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function Set = amun_synth_env(Set)

%% Bauraum
% Der Roboter soll innerhalb eines Quaders arbeiten, der Abstand zu
% benachbarten Robotern gewährleisten soll
b_Quader = 1.5*0.600; % Seitenlänge des Quaders, Annahme: jeweils zwei Jätemodule nebeneinander (Testweise mit rel. Aufmaß)
h_Quader = 2.005; % Höhe des Quaders (wie Gestell des Roboters; Beinketten dürfen nicht nach oben gehen.)
p_Quader = [[-b_Quader/2, -b_Quader/2, 0], ... % Aufpunkt (unten links);
  [b_Quader,0,0], [0,b_Quader,0], ... % Zwei Kantenvektoren
  h_Quader]; % Länge des letzten Kantenvektors
Set.task.installspace = struct( ...
  'type', uint8(1), ... % Nummern der Geometrie-Typen, siehe Implementierung (SerRob, check_collisionset_simplegeom)
  'params', p_Quader, ...  % Quader
  'links', {{0:6}}); % Alle Gelenke müssen in dem Quader sein

%% Hindernisse
% Der Boden zwischen den Reihen ist unten durch den Quader definiert.
% Der Damm auf dem die Nutzpflanzen stehen, ist durch zwei Kapseln
% dargestellt (passt als Objekt und ist verfügbar).
% Primärer Anwendungsfall ist flacher Zwiebelanbau. Erweiterung auf
% Dammkulturen soll durch verschieblich montierbare Jätemodule erfolgen
% Set.task.obstacles = struct('type', uint8(zeros(1,1)), 'params', NaN(1,7));
% Set.task.obstacles.type(1) = uint8(3); % Kapsel
% Set.task.obstacles.params(1,:) = [-1,0,-0.2,1,0,-0.2, 0.30]; % 2m lang, 30cm hoch; etwas tiefer als Bearbeitungspunkte (da Kollisionskörper des Roboters etwas größer sind)
% Eine Kugel oben in der Mitte erzwingt, dass die Beinketten nicht durch
% die Mitte gehen können, was unplausibel sein könnte.
% Set.task.obstacles.type(2) = 4; % Kugel
% Set.task.obstacles.params(2,:) = [0,0,1.2,0.2,NaN(1,3)];
%% Eigenschaften des Roboters, von Aufgabengeometrie beeinflusst
% Das Gestell sollte nicht größer als die Bauraumgrenze (Quader) sein.
Set.optimization.base_size_limits = [0.20, b_Quader/2];
% Die Plattform sollte nicht größer als das Gestell sein, eher viel kleiner
% Benutze die tatsächliche Konstruktion als Maßstab. Dort 75mm gewählt.
Set.optimization.platform_size_limits = [0.050, 0.080]; % NEMA17-Motor: 42mm Flanschmaß
% Die Basis kann in der Höhe verschoben werden (relativ hoch über
% Referenzpunkten).
Set.optimization.basepos_limits = [[0, 0]; [0, 0]; [0.400 1.000]];
% Roboter soll hängend von oben nach unten arbeiten.
Set.structures.mounting_parallel = 'ceiling';
% Kollisionsprüfung ist notwendig.
Set.optimization.constraint_collisions = true;
% Die Aufgabe ist rotationssymmetrisch (es kommen 3T3R-PKM in Betracht)
Set.structures.max_task_redundancy = 1;
% Debug (bei 3T1R-PKM): Nur aufgabenredundante Strukturen wählen
% Führt dazu, dass keine 3T0R-PKM mehr gewählt werden.
Set.structures.min_task_redundancy = 1;
% Technisch unrealistisch, aber für Ergebnis-Veranschaulichung besser.
Set.optimization.max_range_passive_universal = 180*pi/180;
Set.optimization.max_range_passive_spherical = 120*pi/180;
% Plattform-Drehung nur moderat zulassen. Die Oszillationen treten nur bei
% Gradienten-IK auf. Nur noch Benutzung der DP-IK.
% beieinander liegen (hyperbolische Abstoßung; z.B. bei Hexa-Roboter).
Set.optimization.max_velocity_ee_rotation = 180*pi/180;
% Beschleunigung der redundanten Koordinate eher groß wählen. Abbau der max.
% Geschwindigkeit ist damit sehr schnell möglich
Set.optimization.max_acceleration_ee_rotation = 2*pi/0.200;
% Annahme: Symmetrische Maschine ohne Schrägstellung des Werkzeugs ist
% günstiger. Sonst ist der Arbeitsraum nicht symmetrisch
Set.optimization.ee_rotation = false;
Set.optimization.ee_translation = false;
% Form der Roboterbasis: Keine Kollisionskörper vorgeben (weniger
% Ausschluss durch Selbstkollision, konstruktion schwieriger).
Set.optimization.collshape_base = {'joint'};
% Optimiere auch das Aussehen der Plattform (Abstand von paarweise
% angeordneten Koppelgelenken) (ist standardmäßig aktiv)
Set.optimization.platform_morphology = true;
% Optimiere auch Paarabstand von Gelenken (ist standardmäßig aktiv)
Set.optimization.base_morphology = true;
%% Zusätzliche Masse am Endeffektor
% Repräsentiert das Werkzeug. Erste Annahme: 3kg
Set.task.payload = struct('m', 2, 'rS', zeros(3,1), 'Ic', zeros(6,1));
Set.task.payload.Ic(1:3) =  2/5 * Set.task.payload.m * (60e-3)^2; % Abschätzung MTM: Kugel Radius 60mm
