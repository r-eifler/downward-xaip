#include "asnets.h"

#include "../../option_parser.h"
#include "../../plugin.h"

#include <iostream>

static const char *ASNETS_MODULE = "asnets.fd";

namespace policy_testing {
static PyObject *glob_next_state_func = nullptr;

std::string
fact_name_to_asnets(const std::string &name) {
    if (name.find("Atom") == 0) {
        std::string out;
        for (size_t i = 5; i < name.size(); ++i) {
            if (name[i] == '(') {
                out += ' ';
            } else if (name[i] == ',') {
            } else if (name[i] == ')') {
            } else {
                out += name[i];
            }
        }

        size_t end = out.find_last_not_of(" \t");
        if (end != std::string::npos)
            end += 1;
        return out.substr(0, end);
    } else {
        return "none-of-those";
    }
}

PyObject *
var_mapping() {
    int var_size = tasks::g_root_task->get_num_variables();
    PyObject *map = PyList_New(var_size);
    for (int var_id = 0; var_id < var_size; ++var_id) {
        int val_size = tasks::g_root_task->get_variable_domain_size(var_id);
        PyObject *lvar = PyList_New(val_size);
        for (int val_id = 0; val_id < val_size; ++val_id) {
            FactPair fact(var_id, val_id);
            std::string name = tasks::g_root_task->get_fact_name(fact);
            name = fact_name_to_asnets(name);
            PyList_SetItem(lvar, val_id, PyUnicode_FromString(name.c_str()));
        }
        PyList_SetItem(map, var_id, lvar);
    }

    return map;
}

PyObject *
act_mapping() {
    int op_size = tasks::g_root_task->get_num_operators();
    PyObject *map = PyList_New(op_size);
    for (int op_id = 0; op_id < op_size; ++op_id) {
        std::string name = tasks::g_root_task->get_operator_name(op_id, false);
        PyList_SetItem(map, op_id, PyUnicode_FromString(name.c_str()));
    }

    return map;
}

ASNetInterface::ASNetInterface(
    const std::string &domain_pddl,
    const std::string &problem_pddl,
    const std::string &snapshot) {
    if (glob_next_state_func != nullptr) {
        next_state_func = glob_next_state_func;
        return;
    }

    Py_Initialize();

    // Hack to bypass asnets internals requiring command line arguments
    wchar_t *prog = Py_DecodeLocale("/dummy", nullptr);
    PySys_SetArgvEx(1, &prog, 0);
    PyMem_RawFree(prog);

    // Load python module
    PyObject *module = PyImport_ImportModule(ASNETS_MODULE);
    if (module == nullptr) {
        PyErr_Print();
        std::cerr << "Fatal Error: Could not load module " << ASNETS_MODULE
                  << std::endl;
        exit(-1);
    }

    // Obtain init() function from the module
    PyObject *init_func = PyObject_GetAttrString(module, "init");

    // Call init() and check that we obtained the action policy object
    PyObject *args = PyTuple_New(5);
    PyTuple_SetItem(args, 0, PyUnicode_FromString(snapshot.c_str()));
    PyTuple_SetItem(args, 1, PyUnicode_FromString(domain_pddl.c_str()));
    PyTuple_SetItem(args, 2, PyUnicode_FromString(problem_pddl.c_str()));
    PyTuple_SetItem(args, 3, var_mapping());
    PyTuple_SetItem(args, 4, act_mapping());
    PyObject *policy = PyObject_CallObject(init_func, args);
    if (policy == nullptr) {
        PyErr_Print();
        std::cerr << "Fatal Error: Could not obtain policy" << std::endl;
        exit(-1);
    }
    Py_DECREF(init_func);
    Py_DECREF(policy);

    // Obtain next_state() function from the module
    next_state_func = PyObject_GetAttrString(module, "next_state");
    if (next_state_func == nullptr) {
        PyErr_Print();
        std::cerr << "Fatal Error: Could not find next_state()" << std::endl;
        exit(-1);
    }
    glob_next_state_func = next_state_func;

    Py_DECREF(module);
}

ASNetInterface::~ASNetInterface() {
    Py_DECREF(next_state_func);
    Py_Finalize();
}

OperatorID
ASNetInterface::apply_policy(
    const State &state,
    const std::vector<OperatorID> &applicable_ops) {
    PyObject *py_state = PyList_New(state.size());
    for (size_t i = 0; i < state.size(); ++i) {
        PyList_SetItem(py_state, i, PyLong_FromLong(state[i].get_value()));
    }

    PyObject *py_ops = PyList_New(applicable_ops.size());
    for (size_t i = 0; i < applicable_ops.size(); ++i) {
        PyList_SetItem(
            py_ops, i, PyLong_FromLong(applicable_ops[i].get_index()));
    }

    PyObject *args = PyTuple_New(2);
    PyTuple_SetItem(args, 0, py_state);
    PyTuple_SetItem(args, 1, py_ops);
    PyObject *ret = PyObject_CallObject(next_state_func, args);
    if (ret == nullptr) {
        PyErr_Print();
        std::cerr << "Fatal Error: Something bad happened!" << std::endl;
        exit(-1);
    }
    int id = PyLong_AsLong(ret);
    OperatorID op_id(id);
    Py_DECREF(ret);

    /* DEBUG
    for (size_t i = 0; i < applicable_ops.size(); ++i){
        if (applicable_ops[i] == op_id){
            std::cerr << "FOUND! " << op_id << std::endl;
        }
    }
    */
    return op_id;
}

static PluginTypePlugin<ASNetInterface> _type_plugin("ASNetInterface", "");

static std::shared_ptr<ASNetInterface>
_parse_policy(OptionParser &parser) {
    parser.document_synopsis("ASNets policy", "");
    parser.add_option<std::string>("domain_pddl", "Domain PDDL", "");
    parser.add_option<std::string>("problem_pddl", "Problem PDDL", "");
    parser.add_option<std::string>("snapshot", "Snapshot .pkl file", "");
    Options opts = parser.parse();
    if (parser.dry_run()) {
        return nullptr;
    }

    static std::shared_ptr<ASNetInterface> policy = nullptr;
    if (policy == nullptr) {
        policy = std::make_shared<ASNetInterface>(
            opts.get<std::string>("domain_pddl"),
            opts.get<std::string>("problem_pddl"),
            opts.get<std::string>("snapshot"));
    }
    return policy;
}

static Plugin<ASNetInterface> _plugin_policy("asnets_policy", _parse_policy);

ASNetsPolicyPruning::ASNetsPolicyPruning(options::Options &opts)
    : PruningMethod()
      , policy(
          opts.get<std::string>("domain_pddl"),
          opts.get<std::string>("problem_pddl"),
          opts.get<std::string>("snapshot")) {
    (void)opts;
}

void
ASNetsPolicyPruning::prune_operators(
    const State &state,
    std::vector<OperatorID> &op_ids) {
    OperatorID policy_op_id = policy.apply_policy(state, op_ids);
    std::vector<OperatorID> aops;
    aops.push_back(policy_op_id);
    op_ids.swap(aops);
}

void
ASNetsPolicyPruning::print_statistics() const {
    // TODO
}

static std::shared_ptr<PruningMethod>
_parse_pruning(OptionParser &parser) {
    parser.document_synopsis("ASNets pruning", "");
    parser.add_option<std::string>("domain_pddl", "Domain PDDL", "");
    parser.add_option<std::string>("problem_pddl", "Problem PDDL", "");
    parser.add_option<std::string>("snapshot", "Snapshot .pkl file", "");
    Options opts = parser.parse();
    if (parser.dry_run()) {
        return nullptr;
    }

    return std::make_shared<ASNetsPolicyPruning>(opts);
}

static Plugin<PruningMethod> _plugin_pruning("asnets", _parse_pruning);
} /* namespace policy_testing */
