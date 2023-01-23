#include <chrono/physics/ChSystem.h>
#include <chrono_irrlicht/ChVisualSystemIrrlicht.h>

void draw_system_init(chrono::ChSystem& system, std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> application);

void draw_system(chrono::ChSystem& system, std::shared_ptr<chrono::irrlicht::ChVisualSystemIrrlicht> application);