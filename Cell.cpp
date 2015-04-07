#include "Cell.h"

namespace mack {

Cell::Cell() : m_x(-1), m_y(-1), m_sequenceNumber(0), m_parent(0), m_sourceDirection(0), m_distance(0), m_examined(false) {
    for (int i = 0; i < 4; i += 1) {
        m_walls[i] = false;
    }
}

int Cell::getX() const {
    return m_x;
}

int Cell::getY() const {
    return m_y;
}

void Cell::setPosition(int x, int y) {
    m_x = x;
    m_y = y;
}

bool Cell::isWall(int direction) const {
    return m_walls[direction];
}

void Cell::setWall(int direction, bool isWall) {
    m_walls[direction] = isWall;
}

int Cell::getSequenceNumber() const {
    return m_sequenceNumber;
}

void Cell::setSequenceNumber(int sequenceNumber) {
    m_sequenceNumber = sequenceNumber;
}

Cell* Cell::getParent() const {
    return m_parent;
}

void Cell::setParent(Cell* parent) {
    m_parent = parent;
}

int Cell::getSourceDirection() const {
    return m_sourceDirection;
}

void Cell::setSourceDirection(int sourceDirection) {
    m_sourceDirection = sourceDirection;
}

int Cell::getStraightAwayLength() const {
    return m_straightAwayLength;
}

void Cell::setStraightAwayLength(int straightAwayLength) {
    m_straightAwayLength = straightAwayLength;
}

float Cell::getDistance() const {
    return m_distance;
}

void Cell::setDistance(float distance) {
    m_distance = distance;
}

bool Cell::getExamined() const {
    return m_examined;
}

void Cell::setExamined(bool examined) {
    m_examined = examined;
}

} // namespace mack
