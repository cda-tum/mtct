SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

sh $SCRIPT_DIR/launch_all_paramsearch.sh
sh $SCRIPT_DIR/launch_all_method_compare.sh
