#!/opt/data/local/bin/fish

set -g scenarios "single-source"
set -g controllers "stateless" "stateful" "depth1"
set -g timesteps 100000

for s in $scenarios
        for c in $controllers
                echo "Running scenario $s-foraging with controller $c for $timesteps steps"
                sed  's/depth1_foraging/'$c'_foraging/g' exp/$s.argos > /tmp/exp.argos
                sed "s/experiment length=\"0\"/experiment length=\"$timesteps\"/g" -i /tmp/exp.argos
                sed "s/__current_date__/$c/g" -i /tmp/exp.argos
                if not string match "depth1" $c
                        sed 's/create_static="true"/create_static="false"/g' -i /tmp/exp.argos
                end
                argos3 -c /tmp/exp.argos 2>&1 > /dev/null
        end
end
