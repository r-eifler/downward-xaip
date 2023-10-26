#include "command_line.h"
#include "option_parser.h"
#include "search_engine.h"

#include "options/registries.h"
#include "tasks/root_task.h"
#include "task_utils/task_properties.h"
#include "../utils/logging.h"
#include "utils/system.h"
#include "utils/timer.h"
// #ifdef DOWNWARD_PLUGIN_POLICY_TESTING
#include "xaip/policy/policy_client.h"
// #endif /* DOWNWARD_PLUGIN_POLICY_TESTING */

#include <iostream>

using namespace std;
using namespace policy;
using utils::ExitCode;

int main(int argc, const char **argv) {
    utils::register_event_handlers();
        string input_file_override; // if set, read input from this file instead of from stdin

// #ifdef DOWNWARD_PLUGIN_POLICY_TESTING
    if (argc >= 3 && static_cast<string>(argv[1]) == "--remote-policy") {
        try {
            PolicyClient::establish_connection(static_cast<string>(argv[2]));
        } catch (const RemotePolicyError &err) {
            err.print();
            utils::exit_with(ExitCode::REMOTE_POLICY_ERROR);
        }
        for (int i = 3; i < argc; ++i) {
            argv[i - 2] = argv[i];
        }
        argc -= 2;
    } else if (argc >= 3 && static_cast<string>(argv[1]) == "--input-file") {
        input_file_override = static_cast<string>(argv[2]);
        for (int i = 3; i < argc; ++i) {
            argv[i - 2] = argv[i];
        }
        argc -= 2;
    }
// #endif /* DOWNWARD_PLUGIN_POLICY_TESTING */

    if (argc < 2) {
        utils::g_log << usage(argv[0]) << endl;
        utils::exit_with(ExitCode::SEARCH_INPUT_ERROR);
    }

    bool unit_cost = false;
    if (static_cast<string>(argv[1]) != "--help") {
// #ifdef DOWNWARD_PLUGIN_POLICY_TESTING
        if (!PolicyClient::connection_established()) {
             utils::g_log << "reading input from file" << endl;
            if (input_file_override.empty()) {
                tasks::read_root_task(cin, true);
            } else {
                std::ifstream file_stream(input_file_override);
                if (!file_stream.is_open()) {
                    std::cerr << "Cannot open " << input_file_override << std::endl;
                }
                tasks::read_root_task(file_stream, true);
            }
        } else {
            utils::g_log << "reading input from policy" << endl;
            try {
                std::string task = PolicyClient::input_fdr();
                std::istringstream strin(task);
                tasks::read_root_task(strin, false);
            } catch (const RemotePolicyError &err) {
                err.print();
                utils::exit_with(ExitCode::REMOTE_POLICY_ERROR);
            }
        }
// #else /* DOWNWARD_PLUGIN_POLICY_TESTING */
        // tasks::read_root_task(cin, true);
// #endif /* DOWNWARD_PLUGIN_POLICY_TESTING */
        utils::g_log << "done reading input!" << endl;
        TaskProxy task_proxy(*tasks::g_root_task);
        unit_cost = task_properties::is_unit_cost(task_proxy);
    }

    shared_ptr<SearchEngine> engine;

    // The command line is parsed twice: once in dry-run mode, to
    // check for simple input errors, and then in normal mode.
    try {
        options::Registry registry(*options::RawRegistry::instance());
        parse_cmd_line(argc, argv, registry, true, unit_cost);
        engine = parse_cmd_line(argc, argv, registry, false, unit_cost);
    } catch (const ArgError &error) {
        error.print();
        usage(argv[0]);
        utils::exit_with(ExitCode::SEARCH_INPUT_ERROR);
    } catch (const OptionParserError &error) {
        error.print();
        usage(argv[0]);
        utils::exit_with(ExitCode::SEARCH_INPUT_ERROR);
    } catch (const ParseError &error) {
        error.print();
        utils::exit_with(ExitCode::SEARCH_INPUT_ERROR);
    }

    utils::Timer search_timer;
    engine->search();
    search_timer.stop();
    utils::g_timer.stop();

    engine->save_plan_if_necessary();
    engine->print_statistics();
    utils::g_log << "Search time: " << search_timer << endl;
    utils::g_log << "Total time: " << utils::g_timer << endl;

    ExitCode exitcode = engine->found_solution()
        ? ExitCode::SUCCESS
        : ExitCode::SEARCH_UNSOLVED_INCOMPLETE;
    if(engine->get_status() == FINISHED){
        exitcode = ExitCode::SEARCH_FINISHED;
    }
    utils::report_exit_code_reentrant(exitcode);
    return static_cast<int>(exitcode);
}
