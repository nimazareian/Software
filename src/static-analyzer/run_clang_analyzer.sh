BAZEL_TARGET=$1
TITLE="Clang analyzer report for Bazel Target $1"
REPO_ROOT=$PWD
EXECUTION_ROOT=`bazel info execution_root`

bazel clean
bazel build --experimental_action_listener static-analyzer:generate_compile_commands_listener $BAZEL_TARGET
/opt/tbotspython/bin/python3.8 ./static-analyzer/generate_compile_commands_json.py
echo "Performing static analysis..."

cd $EXECUTION_ROOT
scan-build --cdb $REPO_ROOT/compile_commands.json -o $REPO_ROOT/clang-analysis --html-title "$TITLE" --use-analyzer usr/bin/clang
cd -
