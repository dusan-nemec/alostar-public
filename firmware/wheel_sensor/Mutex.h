#ifndef MUTEX_H_
#define MUTEX_H_

struct Mutex
{
  portMUX_TYPE mx;
  Mutex() : mx(portMUX_INITIALIZER_UNLOCKED) {}
  inline void lock() { portENTER_CRITICAL_SAFE(&mx); }
  inline void unlock() { portEXIT_CRITICAL_SAFE(&mx); }
};

class Lock
{
  private:
    portMUX_TYPE* mx;
  public:
    Lock(Mutex& _mx) : mx(&_mx.mx) { portENTER_CRITICAL_SAFE(mx); }
    ~Lock() { portEXIT_CRITICAL_SAFE(mx); }
};


#endif // MUTEX_H_