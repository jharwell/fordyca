#!/opt/data/local/bin/fish

set -g scenarios "single-source"
set -g controllers "stateless"  "stateful" "depth1"
set -g timesteps 500000

for s in $scenarios
        for c in $controllers
                sed  's/depth1_foraging/'$c'_foraging/g' exp/$s.argos > /tmp/exp.argos
                sed "s/experiment length=\"0\"/experiment length=\"$timesteps\"/g" -i /tmp/exp.argos
                sed "s/__current_date__/$c/g" -i /tmp/exp.argos
                argos3 -c /tmp/exp.argos
        end
end
