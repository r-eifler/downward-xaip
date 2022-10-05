#include "goal_subset_search.h"
#include "../search_engines/search_common.h"
#include "../search_engine.h"

#include "../option_parser.h"
#include "../option_parser_util.h"
#include "../plugin.h"

using namespace std;
using namespace goal_subset_search;

namespace plugin_sgss {
static shared_ptr<SearchEngine> _parse(OptionParser &parser) {

    parser.add_option<ParseTree>("engine_config",
                                "search engine for each search");
    parser.add_option<bool>("all_soft_goals",
                            "treat all goals as soft goals",
                            "false");

    SearchEngine::add_options_to_parser(parser);
    Options opts = parser.parse();


    if (parser.help_mode()) {
        return nullptr;
    } else if (parser.dry_run()) {
        //check if the supplied search engines can be parsed
        OptionParser test_parser(opts.get<ParseTree>("engine_config"), parser.get_registry(),
                                     parser.get_predefinitions(), true);
        test_parser.start_parsing<shared_ptr<SearchEngine>>();
        
        return nullptr;
    } else {
        opts.set("weakening", false);
        return make_shared<GoalSubsetSearch>(opts, parser.get_registry(), parser.get_predefinitions());
    }
}

static Plugin<SearchEngine> _plugin("sgss", _parse);
}
