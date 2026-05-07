#ifndef MODE_H
#define MODE_H

// Mode: abstract base class for all car operating modes.
//
// Lifecycle:
//   onEnter() - called once when the mode becomes active
//   update()  - called every loop iteration while active
//   onExit()  - called once when leaving the mode
//   isDone()  - true means "I've completed my work, supervisor please
//               transition me out." Modes that run forever (ManualDrive,
//               WallFollow, open-ended Centering) leave the default
//               (false). Sequenced/auto-completing modes (PressTower,
//               LowTower, Centering with a front-stop threshold) flip
//               this to true when finished — the supervisor then routes
//               them back to MANUAL_DRIVE without each mode having to
//               know about the supervisor.
//
// All future modes derive from this so the main loop can drive them
// through a uniform interface.
class Mode {
public:
  virtual ~Mode() {}
  virtual void onEnter() {}
  virtual void update()  = 0;
  virtual void onExit()  {}
  virtual bool isDone() const { return false; }
  virtual const char* name() const { return "Mode"; }
};

#endif // MODE_H
