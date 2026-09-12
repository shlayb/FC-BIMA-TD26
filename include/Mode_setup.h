#pragma once
#include <Arduino.h>

extern int mode_phase,Mode;

enum class ModeId : uint8_t { COPT, QHOV, GUIDED, MANU, FBWA, AUTO, TRNS, RTL, LOITER, AUTO2, COUNT };

extern ModeId current;
extern ModeId active_mode;

static inline const char* modeCode4(ModeId id){
  switch(id){
    case ModeId::MANU: return "MANU";
    case ModeId::FBWA: return "FBWA";
    case ModeId::AUTO: return "AUTO";
    case ModeId::AUTO2: return "AUTO2";
    case ModeId::COPT: return "COPT";
    case ModeId::QHOV: return "QHOV";
    case ModeId::GUIDED: return "GUID";
    case ModeId::TRNS: return "TRNS";
    case ModeId::RTL: return "RTL ";
    case ModeId::LOITER: return "LOIT";
    // case ModeId::FIXEDWING: return "FIXEDWING";
    default: return "UNKN";
  }
}

class ModeBase {
public:
  virtual ~ModeBase() = default;
  virtual ModeId id() const = 0;
  virtual const char* code4() const { return modeCode4(id()); }

  bool enter()   { return _enter(); }
  void update()  { _update(); }   // tanpa dt
  void exit()    { _exit(); }

protected:
  virtual bool _enter()     { return true; }
  virtual void _update()    {}   
  virtual void _exit()      {}
};

class ModeManager {
public:
  ModeManager() { for (auto &p : _tbl) p = nullptr; }

  void registerMode(ModeBase* m){
    if(!m) return;
    _tbl[(uint8_t)m->id()] = m;
  }

  bool setMode(ModeId id);

  void update(){ if(_cur) _cur->update(); }   // tanpa dt
  ModeBase* currentMode() const { return _cur; }
  const char* code4() const { return _code4; }

   // ==== Tambahan: suspend semua mode ====
  void suspend(){
    if(_cur){ _cur->exit(); }
    _cur = nullptr;
    _code4 = "UNKN";
  }
  bool is_suspended() const { return _cur == nullptr; }

private:
  ModeBase* _tbl[(uint8_t)ModeId::COUNT]{};
  ModeBase* _cur = nullptr;
  const char* _code4 = "UNKN";
};

extern ModeManager modes;

inline bool ModeManager::setMode(ModeId id){
  ModeBase* next = _tbl[(uint8_t)id];
  if(!next) return false;
  if(_cur) _cur->exit();
  _cur = next;
  if(!_cur->enter()){ 
    _cur = nullptr;
    return false;
  }
  _code4 = _cur->code4();
  active_mode = id;
  current = id;
  return true;
}