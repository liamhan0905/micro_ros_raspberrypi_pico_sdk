#ifndef BEHAVIOR_H
#define BEHAVIOR_H

#include <string>

class Behavior {
public:
    // Constructors
    Behavior();
    Behavior(std::string name, int id, float x, float y, float orientation);

    // Destructor
    ~Behavior();

    // Methods
    void setName(std::string name);

    void moveForward();
    void moveForward();
    void turnLeft();
    void turnRight();

private:
};

#endif // BEHAVIOR_H

