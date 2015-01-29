#include <iostream>

template <class T>
class Strategy{
  private:
    T strategy;
  public:
    T getStrategy(){ return strategy; }
    void change_strategy(T strategy){ this->strategy = strategy; }
};

template <class T, class S>
class IRosServiceInvoker
{
  private:
    Strategy<S> strategy;
  public:
    void set_strategy(S strategy){ this->strategy.change_strategy(strategy); }
    S get_strategy(void) { return strategy.getStrategy(); }
    virtual void setup(T s) = 0;
    virtual std::string call_service() = 0;
};

