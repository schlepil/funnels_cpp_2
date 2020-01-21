//
// Created by philipp on 12/16/19.
//

#include "funnels/utils.hh"

#include <algorithm>
#include <tuple>
#include <iostream>

namespace funnels {
  
  const std::vector<std::string> all_expressions__(
      {"=", "+", "-", "*", "/", "%",
          "==", "!=", "<=", "<", ">", ">="
          }
      );
  
  const std::vector<std::string> all_ieq__(
      {"<=", "<", ">", ">="}
  );
  
  bool check_op__(const std::string & op_){
    return std::find(all_expressions__.cbegin(), all_expressions__.cend(),
        op_)!=all_expressions__.end();
  }
  
  bool check_ieq_op__(const std::string & op_){
    return std::find(all_ieq__.cbegin(), all_ieq__.cend(),
                     op_)!=all_ieq__.end();
  }
  
  
  
  ////////////////////////
  
  template<>
  size_t pair_hash::operator()(const std::pair<size_t,size_t> &pair)const{
    size_t seed = pair.first;
    boost::hash_combine(seed, pair.second);
    return seed;
  }
  
  ////////////////////////
  
  named_var_t::named_var_t(size_t id, const std::string &name) :
      _id(id), _name(name) {}
  
  size_t named_var_t::id() const {
    return _id;
  }
  
  const std::string &named_var_t::name() const {
    return _name;
  }
  
  std::string named_var_t::declare() const {
    std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
    throw std::runtime_error("Virtual");
  }
  
  bool named_var_t::operator==(const named_var_t& other) const {
    return _id == other._id;
  }
  
  bool named_var_t::operator!=(const named_var_t& other) const {
    return !(this->operator==(other));
  }
  
  ////////////////////////
  
  expression_t::expression_t(const funnels::named_var_t &lhs,
      std::string act, long value,
      std::vector<std::pair<std::string, const named_var_t *>> rhs):
      _lhs(lhs), _act(std::move(act)), _value(value), _rhs(std::move(rhs)){
    assert(check_op__(_act));
    for (const auto & a_rhs : _rhs){
      assert(check_op__(a_rhs.first));
    }
  }
  
  void expression_t::add_rhs(std::string act, const funnels::named_var_t &rhs) {
    assert(check_op__(act));
    _rhs.emplace_back(std::move(act), &rhs);
  }
  
  std::string expression_t::eval_str() const {
    std::string temp_str = _lhs.name()+_act+std::to_string(_value);
    for (const auto & v: _rhs){
      temp_str += v.first + v.second->name();
    }
    return temp_str;
  }
  
  ////////////////////////
  
  const std::string initial_t::_str = "initial::";
  const std::string committed_t::_str = "committed::";
  const std::string urgent_t::_str = "urgent::";
  
  ////////////////////////
  
  expr_list_attr_t::expr_list_attr_t(std::vector<expression_t> expr):
  _expr(std::move(expr)){}
  
  void expr_list_attr_t::add_expr(const expression_t &expr) const {
    _expr.push_back(expr);
  }
  
  void expr_list_attr_t::add_expr(const expression_t &&expr) const {
    _expr.push_back(expr);
  }
  
  void expr_list_attr_t::clear(){
    _expr.clear();
  }
  
  std::string expr_list_attr_t::eval_str(const std::string & id_string,
      const std::string & join_string) const {
    std::string temp_str="";
    if (_expr.size()) {
      temp_str += id_string;
      for (const auto &v : _expr) {
        temp_str += v.eval_str() + join_string;
      }
      for(size_t i=0; i<join_string.size(); ++i){
        temp_str.pop_back();
      }
      temp_str += ":";
    }
    return temp_str;
  }
  
  const std::string invariant_t::_id_str = "invariant:";
  const std::string provided_t::_id_str = "provided:";
  const std::string do_t::_id_str = "do:";
  
  const std::string invariant_t::_join_str = "&&";
  const std::string provided_t::_join_str = "&&";
  const std::string do_t::_join_str = ";";
  
  ////////////////////////
  
  label_t::label_t(size_t id, std::string name):
      named_var_t(id, name){};
  std::string label_t::eval_str() const {
    return _name;
  };
  
  labels_t::labels_t(){};
  
  labels_t::labels_t(const std::vector<label_t> &label_vec){
    for (const label_t & l : label_vec){
      _label_id_vec.push_back(l.id());
    }
  };
  
  void labels_t::add_label(const label_t &label) {
    _label_id_vec.push_back(label.id());
  }
      
  std::string labels_t::eval_str() const{
    std::string temp_str = "";
    if (!_label_id_vec.empty()) {
      temp_str += "labels:";
      for (auto v:_label_id_vec) {
        temp_str += utils_ext::label_map[v].eval_str() + ",";
      }
      temp_str.pop_back();
    }
    temp_str += ":";
    return temp_str;
  }
  
  ////////////////////////
  
  event_t::event_t(size_t id, const std::string &name) :
      named_var_t(id, name) {}
  
  std::string event_t::declare() const {
    return "event:" + name();
  }
      
  ////////////////////////
  
  process_t::process_t(size_t id, const std::string &name) :
      named_var_t(id, name) {}
  
  std::string process_t::declare() const {
   return "process:"+name();
  }
  
  ////////////////////////
  
  location_t::location_t(size_t id, const std::string &name,
      const process_t &process, const location_type_t l_t) :
      named_var_t(id, name), _process(process),
      _is_comm_urg(l_t){}
      
   void location_t::add_label(const funnels::label_t &lbl) const {
    _lbl.add_label(lbl);
  }
  
  void location_t::set_loc_typ(location_type_t new_type) const{
    assert(0<=new_type && new_type<=1);
    _is_comm_urg=new_type;
  }
  
  const std::string &location_t::proc_name() const{
    return _process.name();
  }
  
  std::string location_t::declare() const{
    std::string temp_string = "location:"+proc_name()+":"+name()+"{";
    std::string temp_string2;
    size_t l_init = temp_string.size();
    if (_is_comm_urg == COMMITTED){
      //committed
      temp_string += committed_t::_str;
    }
    if (_is_comm_urg == URGENT){
      //urgent
      temp_string += urgent_t::_str;
    }
    if(_is_initial){
      temp_string += initial_t::_str;
    }
    temp_string2 = _lbl.eval_str();
    if (temp_string2.size()>1){
      temp_string+=temp_string2;
    }
    temp_string2 = invariant_t::eval_str();
    if (temp_string2.size()>1){
      temp_string+=temp_string2;
    }
    if(temp_string.size()>l_init){
      temp_string.pop_back();
    }
    temp_string += "}";
    return temp_string;
  }
  
  void location_t::set_initial(bool is_initial)const{
    _is_initial = is_initial;
  }
  
  const process_t &location_t::process()const{
    return _process;
  }
  
  ////////////////////////
  
  
  sync_t::sync_t(size_t id, const std::string &name):
      named_var_t(id, name){}
  
  void
  
  sync_t::add_sync(const process_t &proc, const event_t &event, bool is_weak) {
    _sync_list.emplace_back(proc, event, is_weak ? "?":"");
  }
  
  std::string sync_t::declare() const {
    if (_sync_list.size()<2){
      throw std::runtime_error("At least two events needed");
    }
    std::string temp_string = "sync";
    
    for (auto v : _sync_list){
      temp_string += (":" + std::get<0>(v).name() + "@" +
          std::get<1>(v).name() + std::get<2>(v));
    }
    return temp_string;
  }
  
  ////////////////////////
  
  clock_ta_t::clock_ta_t(size_t id, const std::string &name) :
      named_var_t(id, name) {}
  
  ////////////////////////
  
  intvar_t::intvar_t(size_t id, const std::string &name,
                     size_t min_v, size_t max_v, size_t init) :
      named_var_t(id, name),
      _min_v(min_v), _max_v(max_v), _init(init) {
    assert(_min_v<=_init && _init<=_max_v && "out of range");
  }
  
  ////////////////////////
  
}

namespace utils_ext{
  ////////////////////////
  
  funnels::named_var_map_t<funnels::clock_ta_t> clock_map;
  funnels::named_var_map_t<funnels::location_t> loc_map;
  funnels::named_var_map_t<funnels::intvar_t> intvar_map;
  funnels::named_var_map_t<funnels::process_t> process_map;
  funnels::named_var_map_t<funnels::event_t> event_map;
  funnels::named_var_map_t<funnels::label_t> label_map;
  funnels::named_var_map_t<funnels::sync_t> sync_map;
  
  void clear_all_maps(){
    clock_map.clear();
    loc_map.clear();
    intvar_map.clear();
    process_map.clear();
    event_map.clear();
    label_map.clear();
    sync_map.clear();
  }
  
  ////////////////////////
}
