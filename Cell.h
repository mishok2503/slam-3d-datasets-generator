#pragma once

struct TCell {
    bool isOccupied;
};

inline std::ostream& operator <<(std::ostream& os, const TCell& cell) {
    return os << cell.isOccupied;
}
