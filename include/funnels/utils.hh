//

// Created by philipp on 12/16/19.
//

#ifndef FUNNELS_CPP_UTILS_HH
#define FUNNELS_CPP_UTILS_HH

#include <iostream>
#include <string>
#include <cassert>
#include <unordered_map>
#include <deque>
#include <vector>
#include <memory>

#include <boost/container_hash/hash.hpp>

namespace funnels{
  
  ////////////////////////
  
  struct pair_hash{
    template <class T1, class T2>
    size_t operator()(const std::pair<T1,T2> &pair)const{
      throw std::runtime_error("Not implemented");
      return 0;
    }
  };
  
  template<>
  size_t pair_hash::operator()(const std::pair<size_t,size_t> &pair)const;
  
  ////////////////////////
  
  class named_var_t{
  public:
    named_var_t(size_t id, const std::string &name);
    
    size_t id() const;
    
    const std::string& name() const;
    
    [[nodiscard]] virtual std::string declare() const;
    
    bool operator==(const named_var_t& other) const;
    bool operator!=(const named_var_t& other) const;
  
  protected:
    const size_t _id;
    const std::string _name;
  };
  
  ////////////////////////
  
  class expression_t{
  public:
    expression_t(const named_var_t &lhs, std::string act,
        long _value=0,
        std::vector<std::pair<std::string, const named_var_t *>> rhs=
        std::vector<std::pair<std::string, const named_var_t *>>());
    
    expression_t(const expression_t &expr)=default;
    expression_t(expression_t &&expr)=default;
    
    void add_rhs(std::string act, const named_var_t &rhs);
    
    std::string eval_str() const;
  
  protected:
    const named_var_t &_lhs;
    const std::string _act;
    const long _value;
    std::vector<std::pair<std::string, const named_var_t *>> _rhs;
  };
  
  ////////////////////////
  
  class attribute_t{
  public:
    void virtual add_expr(const expression_t &expr){
      std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
      throw std::runtime_error("Virtual");
    };
    [[nodiscard]] virtual std::string eval_str() const{
      std::cerr << __FILE__ << ": " << __LINE__ << std::endl;
      throw std::runtime_error("Virtual");
    }
  };
  
  ////////////////////////
  
  class initial_t final: public attribute_t{
  public:
    std::string eval_str() const{
      return _str;
    }
    static const std::string _str;
  };
  
  ////////////////////////
  
  class committed_t final: public attribute_t{
  public:
    //virtual
    std::string eval_str() const{
      return _str;
    }
    // static
    static const std::string _str;
  };
  
  ////////////////////////
  
  class urgent_t final: public attribute_t{
  public:
    //virtual
    std::string eval_str() const{
      return _str;
    }
    // static
    static const std::string _str;
  };
  
  ////////////////////////
  
  class label_t: public named_var_t{
  public:
    label_t(size_t id, std::string name);
    
    std::string eval_str() const;
  };
  
  class labels_t final{
  public:
    labels_t();
    labels_t(const std::vector<label_t> &label_vec);
    
    void add_label(const label_t &label_vec);
    std::string eval_str() const;

  protected:
    std::vector<size_t> _label_id_vec;
  };
  
  ////////////////////////
  
  class expr_list_attr_t: public attribute_t{
  public:
    expr_list_attr_t(){};
    expr_list_attr_t(std::vector<expression_t> expr);
    
    void add_expr(const expression_t &expr) const;
    void add_expr(const expression_t &&expr) const;
    
    template<class ...ARGS>
    void add_expr(const ARGS & ...args) const {
      _expr.emplace_back(args...);
    }
    template<class ...ARGS>
    void add_expr(const ARGS && ...args) const {
      _expr.emplace_back(args...);
    }
    
    void clear();
    
    [[nodiscard]] std::string eval_str(const std::string &id_string,
                                       const std::string &join_string) const;
  
  protected:
    mutable std::vector<expression_t> _expr;
  };
  
  ////////////////////////
  
  class invariant_t: public expr_list_attr_t{
  public:
    invariant_t(){};
    invariant_t(std::vector<expression_t> expr):
    expr_list_attr_t(expr){};
    
    std::string eval_str() const{
      return expr_list_attr_t::eval_str(_id_str, _join_str);
    };
  protected:
    static const std::string _id_str;
    static const std::string _join_str;
  };
  
  ////////////////////////
  
  class provided_t final: public expr_list_attr_t{
  public:
    provided_t(){};
    provided_t(std::vector<expression_t> expr):
        expr_list_attr_t(expr){};
    
    std::string eval_str() const{
      return expr_list_attr_t::eval_str(_id_str, _join_str);
    };
  protected:
    static const std::string _id_str;
    static const std::string _join_str;
  };
  
  ////////////////////////
  
  class do_t final: public expr_list_attr_t{
  public:
    do_t(){};
    do_t(std::vector<expression_t> expr):
        expr_list_attr_t(expr){};
    
    std::string eval_str() const{
      return expr_list_attr_t::eval_str(_id_str, _join_str);
    };
  protected:
    static const std::string _id_str;
    static const std::string _join_str;
  };
  
  ////////////////////////
  
  class event_t final:public named_var_t{
  public:
    event_t(size_t id, const std::string &name);
    
    [[nodiscard]] std::string declare() const;
  };

  ////////////////////////
  
  class process_t final:public named_var_t{
  public:
    process_t(size_t id, const std::string &name);
    
    [[nodiscard]] std::string declare() const;
  };
  
  ////////////////////////
  
  enum location_type_t{
    BASE,
    COMMITTED,
    URGENT
  };
  
  class location_t final:public named_var_t, public invariant_t{
  public:
    location_t(size_t id, const std::string &name,
        const process_t & process, const location_type_t l_t=BASE);
    
    [[nodiscard]] const std::string &proc_name()const;
    [[nodiscard]] std::string declare() const;
  
    void add_label(const label_t &lbl) const;
    void set_loc_typ(location_type_t new_type) const;
    void set_initial(bool is_initial)const;
    
    const process_t &process()const;

  protected:
    const process_t & _process;
    mutable location_type_t _is_comm_urg=location_type_t::BASE;
    mutable bool _is_initial=false;
    mutable labels_t _lbl;
  };
  
  ////////////////////////
  
  
  class sync_t final:public named_var_t{
  public:
    sync_t(size_t id, const std::string &name);
    
    void add_sync(const process_t & proc, const event_t & event,
        bool is_weak=false);
        
    [[nodiscard]] std::string declare() const;
  
  private:
    std::deque<std::tuple<const process_t&, const event_t&, const std::string>>
    _sync_list;
  };
  
  ////////////////////////
  
  class clock_ta_t final:public named_var_t{
  public:
    clock_ta_t(size_t id, const std::string &name);
  
    [[nodiscard]] std::string declare() const{
      return "clock:1:"+name();
    }
  };
  
  ////////////////////////
  
  class intvar_t final:public named_var_t{
  public:
    intvar_t(size_t id, const std::string &name,
        size_t min_v, size_t max_v, size_t init);
  
    [[nodiscard]] std::string declare() const{
      return "int:1:"+std::to_string(_min_v)+":"+std::to_string(_max_v)+
        ":"+std::to_string(_init)+":"+name();
    }

  protected:
    const size_t _min_v, _max_v, _init;
  };
  
  ////////////////////////
  
  template <class T>
  class named_var_map_t{
  public:
    T &operator[](const std::string &name){
      return *_name2elem.at(name);
    }
    T &operator[](size_t id){
      return *_id2elem.at(id);
    }
    
    bool find(const std::string &name){
      return _name2elem.find(name)!=_name2elem.end();
    }
  
    bool find(size_t id){
      return _id2elem.find(id)!=_id2elem.end();
    }
    
    template <class ... ARGS>
    T & create_and_get(const std::string &name, const ARGS & ...args){
      assert(_name2elem.find(name) == _name2elem.end() && "Not allowed to exist");
      size_t this_id = _next_id;
      
      _objs.emplace_back(this_id, name, args ...);
      _name2elem[name] = &_objs.back();
      _id2elem[this_id] = &_objs.back();
      
      _next_id++;
      
      return _objs.back();
    }
    
    void clear(){
      _id2elem.clear();
      _name2elem.clear();
      _objs.clear();
      _next_id=0;
    }

  protected:
    std::unordered_map<size_t, T*> _id2elem;
    std::unordered_map<std::string, T*> _name2elem;
    std::deque<T> _objs;
    size_t _next_id = 0;
  };
  
} //funnels

namespace utils_ext{
  ////////////////////////
  
  extern funnels::named_var_map_t<funnels::clock_ta_t> clock_map;
  extern funnels::named_var_map_t<funnels::location_t> loc_map;
  extern funnels::named_var_map_t<funnels::intvar_t> intvar_map;
  extern funnels::named_var_map_t<funnels::process_t> process_map;
  extern funnels::named_var_map_t<funnels::event_t> event_map;
  extern funnels::named_var_map_t<funnels::label_t> label_map;
  extern funnels::named_var_map_t<funnels::sync_t> sync_map;
  
  void clear_all_maps();
  
  ////////////////////////
}

#endif //FUNNELS_CPP_UTILS_HH
