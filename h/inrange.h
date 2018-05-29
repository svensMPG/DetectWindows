#ifndef INRANGE_H
#define INRANGE_H


template<typename T>
class inRange
{
public:
    inRange(T lo, T hi) : low(lo), high(hi) {}
    bool contains(T value) const { return low <= value && value < high; }  //return true if within range else return false.
private:
    T low;
    T high;
};
template<typename T>
inRange<T> inrange(T lo, T hi) { return inRange<T>(lo, hi); }

#endif // INRANGE_H
