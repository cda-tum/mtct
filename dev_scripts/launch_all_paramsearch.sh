SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

sh $SCRIPT_DIR/launch_genetic_paramsearch.sh
sh $SCRIPT_DIR/launch_greedy_paramsearch.sh
sh $SCRIPT_DIR/launch_local_paramsearch.sh
