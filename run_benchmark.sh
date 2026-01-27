#!/bin/bash

# Create output directory if it doesn't exist
mkdir -p examples/output

CONFIG_FILE="examples/planner_cfgs/config_db_rrt_example.yaml"

# Run the planner 100 times
for i in $(seq 1 100); do
    echo "Running iteration $i/100..."

    # Update the seed in the config file to the current run number
    sed -i "s/seed: .*/seed: $i/" "$CONFIG_FILE"

    ./build/mr_syclop \
        -i examples/simple_4robots.yaml \
        -o "examples/output/mr_syclop_db_rrt_solution_${i}.yaml" \
        --joint "examples/output/mr_syclop_db_rrt_joint_${i}.yaml" \
        --optimization "examples/output/mr_syclop_db_rrt_optimization_${i}.yaml" \
        -c "$CONFIG_FILE" \
        > "examples/output/run_${i}_output.txt" 2>&1

    # Save the out_dbrrt.yaml file with the run number
    if [ -f "out_dbrrt.yaml" ]; then
        cp "out_dbrrt.yaml" "examples/output/out_dbrrt_${i}.yaml"
    fi

    echo "Iteration $i complete. Output saved to examples/output/run_${i}_output.txt"
done

echo "All 100 runs completed!"
