#ifndef MODE_H
#define MODE_H

// Mode: abstract base class for all car operating modes.
//
// Lifecycle:
//   onEnter() - called once when the mode becomes active
//   update()  - called every loop iteration while active
//   onExit()  - called once when leaving the mode
//
// All future modes (WallFollow, ViveMotion, GeneralMotion, Centering,
// PressButton, ...) should derive from this so the main loop can drive
// them through a uniform interface.
class Mode {
public:
  virtual ~Mode() {}
  virtual void onEnter() {}
  virtual void update()  = 0;
  virtual void onExit()  {}
  virtual const char* name() const { return "Mode"; }
};

#endif // MODE_H
