#include <phase_manager/phase.h>
#include <phase_manager/timeline.h>

Phase::Phase(Timeline& timeline, int n_nodes, std::string name):
    _n_nodes(n_nodes),
    _name(name),
    _timeline(timeline)
//    _phase_manager(phase_manager)

{
    _init_nodes(_n_nodes);

//    setMap("items", &_items_base);
//    _elem_map["items_ref"] = _items_ref;
//    _elem_map["constraints"] = _constraints;
//    _elem_map["costs"] = _costs;
//    _elem_map["variables"] = _variables;
//    _elem_map["parameters"] = _parameters;

}

std::string Phase::getName()
{
    return _name;
}

int Phase::getNNodes()
{
    return _n_nodes;
}


bool Phase::addItem(ItemBase::Ptr item, std::vector<int> nodes)
{
    auto active_nodes = _check_active_nodes(nodes);

    _add_element(item);

    auto it = std::make_shared<ItemManager>(item, _n_nodes, active_nodes);
    _add_element_manager(it);

    _items_base.push_back(item);

    return true;

}

bool Phase::addItemReference(ItemWithValuesBase::Ptr item_with_ref, Eigen::MatrixXd values, std::vector<int> nodes)
{
    auto active_nodes = _check_active_nodes(nodes);

    _add_element(item_with_ref);

    auto it = std::make_shared<ItemReferenceManager>(item_with_ref, _n_nodes, active_nodes, values);
    _add_element_manager(it);

    _items_ref.push_back(item_with_ref);

    return true;

}

bool Phase::addItemWeight(ItemWithWeightBase::Ptr item_with_weight, Eigen::MatrixXd values, std::vector<int> nodes)
{
    auto active_nodes = _check_active_nodes(nodes);

    _add_element(item_with_weight);

    auto it = std::make_shared<ItemWeightManager>(item_with_weight, _n_nodes, active_nodes, values);

    _add_element_manager(it);


    _items_weight.push_back(item_with_weight);

    return true;

}

bool Phase::addConstraint(ItemWithBoundsBase::Ptr constraint, std::vector<int> nodes)
{

    auto active_nodes = _check_active_nodes(nodes);

    _add_element(constraint);

    auto it = std::make_shared<ConstraintManager>(constraint, _n_nodes, active_nodes);
    _add_element_manager(it);

    _constraints.push_back(constraint);

    return true;
}


bool Phase::addCost(ItemBase::Ptr cost, std::vector<int> nodes)
{
    auto active_nodes = _check_active_nodes(nodes);

    _add_element(cost);

    auto it = std::make_shared<CostManager>(cost, _n_nodes, active_nodes);
    _add_element_manager(it);

    _costs.push_back(cost);

    return true;
}

bool Phase::addVariableBounds(ItemWithBoundsBase::Ptr variable, Eigen::MatrixXd lower_bounds, Eigen::MatrixXd upper_bounds, std::vector<int> nodes)
{

    auto active_nodes = _check_active_nodes(nodes);
    // bounds are updated only on the active nodes

    _add_element(variable);

    auto it = std::make_shared<VariableManager>(variable, _n_nodes, active_nodes, lower_bounds, upper_bounds);
    _add_element_manager(it);

    _variables.push_back(variable);

    return true;
}

bool Phase::addParameterValues(ItemWithValuesBase::Ptr parameter, Eigen::MatrixXd values, std::vector<int> nodes)
{

    auto active_nodes = _check_active_nodes(nodes);
    // bounds are updated only on the active nodes

    _add_element(parameter);

    auto it = std::make_shared<ParameterManager>(parameter, _n_nodes, active_nodes, values);
    _add_element_manager(it);

    _parameters.push_back(parameter);
    return true;
}

std::unordered_set<int> Phase::getNodesAsSet()
{
    return _nodes_as_set;
}

bool Phase::_add_element_manager(NodesManager::Ptr element)
{
    _elements.push_back(element);
    return true;
}

//std::unordered_map<ItemBase::Ptr, std::vector<int>> Phase::getItemLocalNodes()
//{
//    return _items_local_nodes;
//}


std::unordered_map<int, std::vector<int>> Phase::_stretch(std::vector<int> nodes, double stretch_factor)
{
    // output a map with old_node = vector of stretch_nodes
    // stretches the duration of the phase, assigning to each old node a list of new node
    // (some of these list will be empty if the stretch_factor < 1)

    std::unordered_map<int, std::vector<int>> stretch_map;

    int initial_node = 0;
    for (int node : nodes)
    {

        int new_node = static_cast<int>( (node + 1) * stretch_factor);
        std::vector<int> stretch_nodes;
        for (int it = initial_node; it < new_node; it++)
        {
            stretch_nodes.push_back(it);
            initial_node = it + 1;
        }
        stretch_map[node] = stretch_nodes;
    }

    return stretch_map;
}

std::vector<int> Phase::_extract_stretch_nodes(std::unordered_map<int, std::vector<int>> stretch_map, std::vector<int> nodes)
{
    // get vector of stretched nodes
    std::vector<int> new_nodes;

    for (auto node : nodes)
    {
        for (auto new_node : stretch_map[node])
        {
            new_nodes.push_back(new_node);
        }
    }

    return new_nodes;
}

std::vector<int> Phase::_check_active_nodes(std::vector<int> nodes)
{
    // check if added nodes are correct w.r.t. the nodes of the phase
    // if nodes is empty, assume all the nodes are active

    for (int num : nodes) {
        if (_nodes_as_set.find(num) == _nodes_as_set.end()) {
            throw std::invalid_argument("Node inserted ("
                                        + std::to_string(num)
                                        + ") is outside of phase nodes.");
        }
    }

    std::vector<int> active_nodes = (nodes.empty()) ? _vec_nodes : nodes;

    return active_nodes;
}

std::vector<ItemBase::Ptr> Phase::getItems()
{
    return _items_base;
}

std::vector<ItemWithValuesBase::Ptr> Phase::getItemsReference()
{
    return _items_ref;
}

std::vector<ItemWithBoundsBase::Ptr> Phase::getConstraints()
{
    return _constraints;
}

std::vector<ItemBase::Ptr> Phase::getCosts()
{
    return _costs;
}

std::vector<ItemWithBoundsBase::Ptr> Phase::getVariables()
{
    return _variables;
}

std::vector<ItemWithValuesBase::Ptr> Phase::getParameters()
{
    return _parameters;
}

std::vector<NodesManager::Ptr> Phase::getElements()
{
    return _elements;
}

bool Phase::_add_element(ItemBase::Ptr elem)
{
    return _timeline.addElement(elem);
}

//std::unordered_map<ItemBase::Ptr, InfoContainer::Ptr> Phase::getItemsInfo()
//{
//    return _info_items_base;
//}

//std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer::Ptr> Phase::getItemsReferenceInfo()
//{
//    return _info_items_ref;
//}

//std::unordered_map<ItemWithBoundsBase::Ptr, InfoContainer::Ptr> Phase::getConstraintsInfo()
//{
//    return _info_constraints;
//}

//std::unordered_map<ItemBase::Ptr, InfoContainer::Ptr> Phase::getCostsInfo()
//{
//    return _info_costs;
//}

//std::unordered_map<ItemWithBoundsBase::Ptr, BoundsContainer::Ptr> Phase::getVariablesInfo()
//{
//    return _info_variables;
//}

//std::unordered_map<ItemWithValuesBase::Ptr, ValuesContainer::Ptr> Phase::getParametersInfo()
//{
//    return _info_parameters;
//}

bool Phase::_init_nodes(int n_nodes)
{
    _nodes_as_set.clear();
    // create unordered set for phase nodes
    for (int i = 0; i < n_nodes; i++) {
        _nodes_as_set.insert(i);
    }

    _vec_nodes.clear();
    for (int i = 0; i < n_nodes; i++) {
        _vec_nodes.push_back(i);
    }

    return true;
    //    std::cout << "set nodes: " << std::endl;
    //    for (auto node : _set_nodes) {
    //        std::cout << node << " ";
    //    }
    //    std::cout << std::endl;

    //    std::cout << "vec nodes: " << std::endl;
    //    for (auto node : _vec_nodes) {
    //        std::cout << node << " ";
    //    }
    //    std::cout << std::endl;

}


//std::unordered_map<ItemBase::Ptr, std::vector<int>> Phase::getItems()
//{
//    return _items_base;
//}

//std::unordered_map<ItemWithValuesBase::Ptr, Phase::ValuesContainer> Phase::getItemsReference()
//{
//    return _items_ref;
//}

//std::unordered_map<ItemWithBoundsBase::Ptr, std::vector<int>> Phase::getConstraints()
//{
//    return _constraints;
//}

//std::unordered_map<ItemBase::Ptr, std::vector<int>> Phase::getCosts()
//{
//    return _costs;
//}

//std::unordered_map<ItemWithBoundsBase::Ptr, Phase::BoundsContainer> Phase::getVariables()
//{
//    return _variables;
//}

//std::unordered_map<ItemWithValuesBase::Ptr, Phase::ValuesContainer> Phase::getParameters()
//{
//    return _parameters;
//}


PhaseToken::PhaseToken(Phase::Ptr phase):
    _abstract_phase(phase),
    _initial_node(0)
{

    // copy construct a private info container
    for (auto element : _abstract_phase->getElements())
    {
        if (auto item_ref_manager = std::dynamic_pointer_cast<ItemReferenceManager>(element))
        {
            ItemReferenceManager::Ptr item_ref_copy = std::make_unique<ItemReferenceManager>(*item_ref_manager);
            _cloned_ref_elements[item_ref_copy->getItem()->getName()] = std::move(item_ref_copy);
        }

        if (auto item_weight_manager = std::dynamic_pointer_cast<ItemWeightManager>(element))
        {
            ItemWeightManager::Ptr item_weight_copy = std::make_unique<ItemWeightManager>(*item_weight_manager);
            _cloned_weight_elements[item_weight_copy->getItem()->getName()] = std::move(item_weight_copy);
        }

        if (auto parameter_manager = std::dynamic_pointer_cast<ParameterManager>(element))
        {
            auto item_ref_copy = std::make_unique<ParameterManager>(*parameter_manager);
            _cloned_par_elements[item_ref_copy->getItem()->getName()] = std::move(item_ref_copy);
        }

    }
}


std::string PhaseToken::getName()
{
    return _abstract_phase->getName();
}

const std::vector<int>& PhaseToken::getActiveNodes()
{
    /*
     * get the active nodes of the phase (WARNING: relative nodes, the are NOT the absolute nodes in the horizon)
     */
    return _active_nodes;
}

const int PhaseToken::getPosition()
{
    return _initial_node;
}

const int PhaseToken::getNNodes()
{
    return _abstract_phase->getNNodes();
}

bool PhaseToken::setItemReference(std::string item_name, Eigen::MatrixXd values)
{
    // set item reference for independent phase token
    for (auto [name, item] : _cloned_ref_elements)
    {
        if (name == item_name)
        {
            std::static_pointer_cast<ItemReferenceManager>(item)->setValues(values);
            return true;
        }
    }

    return false;
}

bool PhaseToken::setItemWeight(std::string item_name, Eigen::MatrixXd weight)
{
    // set item reference for independent phase token
    for (auto [name, item] : _cloned_weight_elements)
    {
        if (name == item_name)
        {
            std::static_pointer_cast<ItemWeightManager>(item)->setWeight(weight);
            return true;
        }
    }

    return false;
}


bool PhaseToken::setItemNodes(std::string item_name, std::vector<int> nodes)
{
    // set nodes for independent phase token
    for (auto item : _abstract_phase->getItems())
    {
        if (item->getName() == item_name)
        {
            item->setNodesInternal(nodes, true);
            return true;
        }
    }

    return false;
}


std::vector<int>& PhaseToken::_get_active_nodes()
{
    return _active_nodes;
}

Phase::Ptr PhaseToken::get_phase()
{
    return _abstract_phase;
}

std::pair<std::vector<int>, std::vector<int>> PhaseToken::_compute_horizon_nodes(std::vector<int> nodes, int initial_node)
{
    /***
     * input:
     * item nodes inside the phase (relative nodes)
     * initial node of the phase (position of phase in horizon)

     * output:
     * active nodes of item relative to the phase
     * absolute nodes in horizon

     * _active_nodes: active nodes of the phase (relative nodes)
     * nodes: nodes of the item (relative nodes)
     * initial_node: where the phase is positioned in the horizon, in terms of node
     * active_item_nodes: item nodes in phase that are active (intersection between active node of the phase and nodes of the item in the phase)
     * horizon_nodes: active nodes of item in horizon

    ***/

    std::vector<int> active_item_nodes; // Vector to store positions of elements in 'nodes' that are in '_active_nodes'

    // check which node of the item (constraint, cost, var...) is active inside the phase, given the active nodes of the phase
    std::set_intersection(nodes.begin(), nodes.end(),
                          _active_nodes.begin(), _active_nodes.end(),
                          std::back_inserter(active_item_nodes));



    std::vector<int> horizon_nodes(active_item_nodes.size());
    // active phase nodes             : [2 3 4 5]
    // active nodes of item in phase  : [3 4]
    // phase position in horizon      : 7
    // item node position in horizon  : 7 + 3 (node 0 of 'active nodes') = 10
    // item node position in horizon  : 7 + 4 (node 1 of 'active nodes') = 11
    for (int node_i = 0; node_i < horizon_nodes.size(); node_i++)
    {
        horizon_nodes[node_i] = initial_node + active_item_nodes[node_i];
    }

//    std::cout << "    initial_node: " << initial_node << std::endl;
//    std::cout << "      active item nodes: ";
//    for (auto node_i : active_item_nodes)
//    {
//        std::cout << node_i << " ";
//    }
//    std::cout << std::endl;
//    std::cout << "------" << std::endl;

    return std::make_pair(active_item_nodes, horizon_nodes);

}

bool PhaseToken::_set_position(int initial_node)
{
    _initial_node = initial_node;
    return true;
}

//bool PhaseToken::_set_position(int initial_node)
//{
//    _initial_node = initial_node;
//}

bool PhaseToken::update()
{
    /*
     * update items contained in phase in horizon based on the position of the phase
     */

//    std::cout << "   -> updating phase: '" << getName() << "' at node: " << _initial_node << std::endl;
    if (!_active_nodes.empty())
    {

        for (auto element : _abstract_phase->getElements())
        {
            auto it = element;
            if (std::dynamic_pointer_cast<ItemReferenceManager>(it))
            {
                if (_cloned_ref_elements.find(element->getName()) != _cloned_ref_elements.end())
                {
                        it = _cloned_ref_elements[element->getName()];
                }
            }
            if (std::dynamic_pointer_cast<ItemWeightManager>(it))
            {
                if (_cloned_weight_elements.find(element->getName()) != _cloned_weight_elements.end())
                {
                        it = _cloned_weight_elements[element->getName()];
                }
            }
            if (std::dynamic_pointer_cast<ParameterManager>(it))
            {
                if (_cloned_par_elements.find(element->getName()) != _cloned_par_elements.end())
                {
                        it = _cloned_par_elements[element->getName()];
                }
            }

//            std::cout << "        --> element updated: " << element->getName() << std::endl;
            auto pair_nodes = _compute_horizon_nodes(element->getSelectedNodes(), _initial_node);

            it->update(pair_nodes.first, pair_nodes.second);
        }

//        std::cout << "===========================" << std::endl;
    }
    return true;

}
