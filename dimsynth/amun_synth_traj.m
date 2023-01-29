% Definition der Beispiel-Trajektorie für die Maßsynthese der Unkraut-PKM
% 
% Eingabe:
% trajset
%   Einstellungen für Trajektorie
%   Siehe cds_settings_defaults.m
% 
% Ausgabe:
% Traj
%   Struktur mit Trajektorie des Endeffektors (Zeit, Position, Geschw.)

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-07
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function Traj = amun_synth_traj(trajset)

%% Eckpunkte für Beispiel-Trajektorie bestimmen
% Minimalmaße Arbeitsraum
b_ws = 0.190;       % minimale Breite workspace (Anforderungen)
t_ws = 0.190;       % minimale Tiefe workspace (Anforderungen)
h_ws = 0.340;       % minimale Höhe workspace (Anforderungen), kann ggf. geringer gewählt werden (max. 10cm Nutzpflanzenhöhe)
% Punkte im Welt-KS.
X0 = [ [0.00;0.00;0.4]; [0;0;0]*pi/180 ]; % Ruhestellung, [TODO: Abstimmen auf vertikale Basisposition. Ruhestellung darüber ist sinnlos.]
X1 = X0' + [0, 0, -h_ws,0,0,0]; % Arbeitshöhe

% Verschiedene Punkte. Werkzeug zeigt zunächst senkrecht.
% Die z-Achse muss im Welt-KS nach oben zeigen, auch wenn in der späteren
% Anwendung die z-Achse besser nach unten zeigen sollte.
k=1;   XL(k,:) = X1 + [ b_ws/2,0,0, r2eulxyz(rotx(0))'];
k=k+1; XL(k,:) = X1 + [-b_ws/2,0,0, r2eulxyz(rotx(0))'];
k=k+1; XL(k,:) = X1 + [ 0, b_ws/2,0, r2eulxyz(rotx(0))'];
k=k+1; XL(k,:) = X1 + [0, -b_ws/2,0, r2eulxyz(rotx(0))'];

%% Bestimme eine Reihenfolge der anzufahrenden Punkte
% Abwechselnd den Punkt unten und wieder zurück in Ausgangs-Lage
XL1 = NaN(2*size(XL,1)+1,6);
XL1(1:2:end,:) = repmat(X0', size(XL,1)+1, 1);
XL1(2:2:end,:) = XL;

if strcmp(trajset.refTraj, 'random')
    % zufällige Posen im Arbeitsraum
    rotE = deg2rad(15);     % maximaler EE-Schwenkwinkel in Trajektorie
    X = genRndTargetPoints(20, b_ws, t_ws, h_ws, rotE, X1(3));
    XL1 = [XL1; X];
end

if strcmp(trajset.refTraj, 'weeds')
    % zufällige Verteilung Unkrautpflanzen nach Anforderungsliste
    rotE = deg2rad(15);     % maximaler EE-Schwenkwinkel in Trajektorie
    freq_plants = 4;       % Unkrautpflanzen pro Meter
    distTask = 3;          % simulierte Strecke mit Unkräutern
    XL1 = genWeedDist(0.015, X1(3), freq_plants, distTask, rotE);
end

%% Trajektorie generieren
if trajset.profile == 1
  [X_ges,XD_ges,XDD_ges,T_ges,IL1] = traj_trapez2_multipoint(XL1, ...
    trajset.vmax, trajset.vmax/trajset.amax, trajset.Tv, trajset.Ts, 0);
elseif trajset.profile == 0 % Nur Eckpunkte
  X_ges = XL1;
  XD_ges = XL1*0;
  XDD_ges = XL1*0;
  T_ges = (1:size(XL1,1))'; % Muss Spaltenvektor sein
  IL1 = (1:size(XL1,1))';
else
  error('Profil nicht definiert');
end

%% Geschwindigkeit des Zugfahrzeuges addieren
if strcmp(trajset.refTraj, 'weeds')
    % Annahme zielgeschwindigkeit: 1km/h
    v_tractor = 1/3.6*0.9*1;      % in m/s
    X_ges(:,2) = X_ges(:,2) - T_ges*v_tractor;
    XD_ges(:,2) = XD_ges(:,2) - v_tractor;
    XL1 = X_ges;
end

%% Ausgabe
[XL1_unique, I_unique] = unique(XL1, 'rows','stable');
Traj = struct('X', X_ges, 'XD', XD_ges, 'XDD', XDD_ges, 't', T_ges, ...
  'XE', XL1_unique, 'IE', IL1(I_unique));
end

function X = genWeedDist(y0, z0, freq_plants, distTask, rotE)
    % generate randomly distributed weed plants in the coordinate frame of
    % the robot
    %
    % Inputs:
    % y0
    %   distance of first weed in direction of tractor motion
    % z0
    %   vertical position of weed (z-coordinate of the ground)
    % n_plants
    %   number of weeds per meter
    % distTask
    %   distance of field to be handled
    % rotE
    %   max. end effecor rotation
    %
    % Outputs:
    % X
    %   coordinates of target points

    cSeed = rng;        % store current seed
    rng(0);

    w_weeds = 0.06;     % width of area, where weeds are around row of crop
    n_weeds = round(distTask * freq_plants);  % number of weeds
    distTask = n_weeds / freq_plants;

    x11 = (rand(n_weeds,1) - 0.5) * w_weeds;
%     tmp = sort(rand(n_weeds,1));
%     x21 = (tmp - tmp(1)) * distTask + y0;
    x21 = linspace(0,1,n_weeds)' * distTask + y0;
    x31 = ones(n_weeds,1) * z0;
    
    % rotational dofs
    x4 = zeros(n_weeds*2,1);
    x51 = rand(n_weeds,1) * rotE .* sign(x11);
    x6 = zeros(n_weeds*2,1);
    
    % add support poses between targets in a safe vertical position
%     x12 = zeros(n_weeds,1);
    x12 = mean([x11, circshift(x11,-1)],2);
    x1 = reshape([x11,x12]',[],1);
    x22 = mean([x21, circshift(x21,-1)],2);
    x2 = reshape([x21,x22]',[],1);
    x32 = ones(n_weeds,1) * z0 + rand(n_weeds,1)*0.3;
    x3 = reshape([x31,x32]',[],1);

    x52 = zeros(n_weeds,1);
    x5 = reshape([x51,x52]',[],1);

    X = [x1,x2,x3,x4,x5,x6];

    X = X(1:end-1,:);

    rng(cSeed);         % restore seed
end

function X = genRndTargetPoints(nPts, w_ws, d_ws, h_ws, rotE, minZ)
    % generate randomly distributed target points within the specified
    % workspace
    % 
    % Inputs:
    % nPts
    %   number of target points
    % w_ws
    %   max. workspace width
    % d_ws
    %   max. workspace depth
    % h_ws
    %   max. workspace height
    % rotE
    %   max. end effecor rotation
    % minZ
    %   minimum Z coordinate (reached while weeding)
    %
    % Outputs:
    % X
    %   coordinates of target points
    
    cSeed = rng;        % store current seed
    rng(5);

%     x1 = (rand(nPts,1) - 0.5) * w_ws;
    x1 = (rand(nPts,1)) * w_ws * 0.5;      % just use half width for trajectory because of symmetric kinematic

    x2 = (rand(nPts,1) - 0.5) * d_ws;
    x3 = ones(size(x1)) * minZ;
    x4 = zeros(size(x1));
    x5 = zeros(size(x1));
    x6 = zeros(size(x1));

    % allow large end effector rotations and vertical translation just in
    % central area of workspace
    bCent = (abs(x1) < 0.5*0.5*w_ws) & (abs(x2) < 0.5*0.5*d_ws);
    x3(find(bCent)) = (rand(sum(bCent),1)) * h_ws + minZ;
    
    bCent2 = bCent & (x3 < minZ + 0.1);
%     x4(find(bCent2)) = (rand(sum(bCent2),1) - 0.5)*2 * rotE;
%     x5(find(bCent2)) = (rand(sum(bCent2),1) - 0.5)*2 * rotE;
    x4(find(bCent)) = (rand(sum(bCent),1) - 0.5)*2 * rotE .* (1 - (x3(find(bCent)) - minZ)/h_ws);
    x5(find(bCent)) = (rand(sum(bCent),1) - 0.5)*2 * rotE .* (1 - (x3(find(bCent)) - minZ)/h_ws);

    X = [x1,x2,x3,x4,x5,x6];

    rng(cSeed);
end