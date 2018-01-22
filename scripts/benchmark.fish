#!/opt/data/local/bin/fish

set -g scenarios "single-source"
set -g controllers "stateless"
set -g robots 4 8 12 16 20
set -g timesteps 20000
set -g speed_throttles 0 10 20 40 80
set -g cache_penalties 25 50 100 200 400 800
set -g partitionings true false

# argv[1] - Scenario
# argv[2] - Controller
# argv[3] - Number of robots
# argv[4] - Speed throttle
# argv[5] - cache_penalty
# argv[6] - always_partition value
function generate_input_file
        sed 's/experiment length="__0__"/experiment length="'$timesteps'"/g' exp/benchmark-$argv[1].argos > /tmp/exp-$argv[1].argos
        sed "s/__current_date__/$argv[1]-$argv[2]-n_robots=$argv[3],throttle=$argv[4],penalty=$argv[5],always_partition=$argv[6]/g" -i /tmp/exp-$argv[1].argos
        sed  's/__controller__/'$argv[2]'_foraging/g' -i /tmp/exp-$argv[1].argos
        sed  's/n_robots=\"__0__\"/n_robots=\"'$argv[3]'\"/g' -i /tmp/exp-$argv[1].argos
        sed  's/quantity=\"__0__\"/quantity=\"'$argv[3]'\"/g' -i /tmp/exp-$argv[1].argos
        sed 's/speed_throttle_block_carry="__0__"/speed_throttle_block_carry=\"'$argv[4]'"/g' -i /tmp/exp-$argv[1].argos
        sed 's/usage_penalty="__0__"/usage_penalty="'$argv[5]'"/g' -i /tmp/exp-$argv[1].argos
        sed 's/always_partition="__0__"/always_partition="'$argv[6]'"/g' -i /tmp/exp-$argv[1].argos
        if not string match "depth1" $argv[2]
                sed 's/create_static="true"/create_static="false"/g' -i /tmp/exp-$argv[1].argos
        end

end

for s in $scenarios
        for c in $controllers
                for r in $robots
                        if not string match "depth1" $c
                                generate_input_file $s $c $r 0 0 false
                                echo "Running scenario=$s, controller=$c, n_robots=$r"
                                argos3 -c /tmp/exp-$s.argos 2>&1 > /dev/null
                                continue
                        end

                        for t in $speed_throttles
                                for penalty in $cache_penalties
                                        for partitioning in $partitionings
                                                generate_input_file $s $c $r $t $penalty $partitioning
                                                echo "Running scenario=$s, controller=$c, n_robots=$r, throttle=$t, cache_penalty=$penalty, always_partition=$partitioning"
                                                argos3 -c /tmp/exp-$s.argos 2>&1 > /dev/null
                                        end
                                end
                        end
                end
        end
end
