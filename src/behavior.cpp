#include "behavior.hpp"
#include "motorDriver.hpp"
#include <cmath>

// Constructors
Behavior::Behavior() {
}

Behavior::Behavior(std::string name, int id, float x, float y, float orientation) {
    m_name = name;
    m_id = id;
    m_x = x;
    m_y = y;
    m_orientation = orientation;
}

// Destructor
Behavior::~Behavior() {}


// Movement methods
void behavior::moveForward(float distance) {
    float radian_orientation = m_orientation * M_PI / 180.0;
    m_x += distance * cos(radian_orientation);
    m_y += distance * sin(radian_orientation);
}

void behavior::turnLeft(float angle) {
    m_orientation -= angle;
    if (m_orientation < 0) {
        m_orientation += 360;
    }
}

void behavior::turnRight(float angle) {
    m_orientation += angle;
    if (m_orientation >= 360) {
        m_orientation -= 360;
    }
}

