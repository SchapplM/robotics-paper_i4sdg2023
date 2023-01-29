% Starte die Maßsynthese aus dem Repo zur kombinierten Synthese.
% Zusätzliche Ablaufsteuerung mit Einstellungen aus Eingabe configset
% 
% Eingabe:
% Set, Traj
%   Standard-Eingaben der Maßsynthese; siehe cds_settings_defaults, cds_start
% configset
%   Zusätzliche Einstellungen

% Moritz Schappler, moritz.schappler@imes.uni-hannover.de, 2021-09
% (C) Institut für Mechatronische Systeme, Leibniz Universität Hannover

function start_dimsynth(Set, Traj, configset)

merge_results_only = configset.merge_results_only;
num_repetitions = configset.num_repetitions;
optname = configset.optname;

for k = 1:num_repetitions
  if num_repetitions == 1 % Ohne Wiederholung
    Set.optimization.optname = sprintf('%s', optname);
  else % Mehrfache Durchführung
    Set.optimization.optname = sprintf('%s_rep%d', optname, k);
  end
  if ~merge_results_only
    cds_start(Set, Traj);
    if k ~= num_repetitions % nicht beim letzten
      pause(30); % Damit nicht alle exakt zeitgleich starten; paralleler Start des parpools nicht möglich
    end
  else
    cds_merge_results( Set.optimization.optname, 'copy', true, true );
  end
end