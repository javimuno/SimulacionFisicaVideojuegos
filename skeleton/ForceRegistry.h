#pragma once
#include <vector>

class Particle;
class ForceGenerator;

class ForceRegistry {
public:
    void add(Particle* p, ForceGenerator* fg);
    void remove(Particle* p, ForceGenerator* fg);
    void clear();
    void updateForces(float dt); // limpia y aplica todos los FG
    struct Entry { Particle* p; ForceGenerator* fg; };
    std::vector<Entry> regs;
    void removeAll(Particle* p);


private:
    
};
