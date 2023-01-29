% Zentrale Konfiguration für die Auswahl von Daten für die Auswertung
% 
% Ausgabe:
% cs [struct]
%   Konfigurations-Struktur

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2023-01
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function cs = i4sdg2023_eval_config()

cs = struct( ...
'optnames', {{}}, ... % Ordner muss existieren
... % Einstellungen für Speicherung und Berechnung der Lösung mit bestehenden Entwurf
'existingdesign_dataname', 'existing_design_collopt', ... % muss nicht für jede Optimerung neu gewählt werden
'existingdesign_resname', 'amunpkm_20230108_existing_design', ... % Unter dem Namen wird Redundanzkarte+DP für das existierende Design gespeichert
'existingdesign_ikopt', 'coll_par', ...
... % Zielfunktionen für Pareto-Diagramm (müssen oben vorkommen)
'pareto_obj1', {{'actvelo', 'actforce'}}, ...
'pareto_obj2', {{'actvelo', 'actforce'}}, ... % zwei mal das gleiche für kombiniertes Bild
... Redundanzkarte mit Kriterien. Möglichkeiten: positionerror, colldist, cond_jac
... % Benutze Positionsfehler, da es einen Überblick über die Leistung zeigt.
'perfmap_criterion', 'positionerror'); % Möglichkeiten: positionerror, positionerror, cond_jac

% Alternatives Diagramm:
% cs.pareto_obj1 = {'colldist', 'footprint'};
% cs.pareto_obj2 = {'positionerror', 'footprint'};

% Wähle einen Versuch mit Dynamik-Berechnung und ohne Betrachtung des
% Eigengewichts des Roboters. Ansatz: Kraftübertragung entscheidend
% cs.optnames={'amunpkm_20221220_sdgpaper_run4_4joint_dynamic_dp_nomass'};
% return
% Alternativ:

% Benutze die Maßsynthese-Auswertungen seit dem 08.01.2023, bei denen die
% Plattform-Größe festgehalten wurde.
% for i = 1:10
%   cs.optnames{i} = sprintf('amunpkm_20230109_sdgpaper_manyobj_rep%d', i);
% end

% Zusätzlich: Alle Optimierungen eines bestimmten Typs benutzen
resdirtotal = i4sdg2023_dimsynth_data_dir();
i = length(cs.optnames);
for d = dir(fullfile(resdirtotal, 'amunpkm_20230108_sdgpaper_plffix*'))'
  i = i+1;
  if d.isdir
    cs.optnames{i} = d.name;
  end
end