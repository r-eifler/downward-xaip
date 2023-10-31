DOMAIN=~/phd_thesis/benchmarks/xaip_benchmarks/resource_and_time_windows/satellite/domain.pddl
PROBLEM=~/phd_thesis/benchmarks/xaip_benchmarks/resource_and_time_windows/satellite/problem_03_01.pddl
# SETTINGS=~/phd_thesis/benchmarks/xaip_benchmarks/resource_and_time_windows/satellite/LTL_vs_AS/properties_LTL_03_01_10.json
SETTINGS=~/phd_thesis/benchmarks/xaip_benchmarks/resource_and_time_windows/satellite/LTL/properties_03_01_10.json

./fast-downward.py $DOMAIN $PROBLEM --translate-options --explanation-settings $SETTINGS --search-options --search 'astar(eval=blind(), pruning=rgsst)' > out.log