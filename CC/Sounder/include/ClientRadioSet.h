/**
 * @file ClientRadioSet.h
 * @brief Declaration file for the ClientRadioSet class.
 */
#ifndef CLIENT_RADIO_SET_H_
#define CLIENT_RADIO_SET_H_

#include <atomic>
#include <cstddef>

#include "Radio.h"
#include "config.h"

class ClientRadioSet {
 public:
  ClientRadioSet(Config* cfg);
  ~ClientRadioSet(void);
  int triggers(int i);
  int radioRx(size_t radio_id, void* const* buffs, int numSamps,
              long long& frameTime);
  int radioTx(size_t radio_id, const void* const* buffs, int numSamps,
              int flags, long long& frameTime);
  void radioStop(void);
  bool getRadioNotFound() { return radioNotFound; }

 private:
  struct ClientRadioContext {
    ClientRadioSet* crs;
    std::atomic_ulong* thread_count;
    size_t tid;
  };
  void init(ClientRadioContext* context);
  static void* init_launch(void* in_context);

  Config* _cfg;
  std::vector<Radio*> radios;
  bool radioNotFound;
};

#endif /* CLIENT_RADIO_SET_H_ */
