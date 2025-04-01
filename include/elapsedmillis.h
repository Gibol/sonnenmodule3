#pragma once

class elapsedMillis
{
private:
    uint32_t ms;
public:
    // Default constructor: sets the start time to current uptime.
    elapsedMillis(void) { ms = k_uptime_get_32(); }

    // Constructor that simulates a timer started "val" milliseconds ago.
    elapsedMillis(unsigned long val) { ms = k_uptime_get_32() - val; }

    // Copy constructor.
    elapsedMillis(const elapsedMillis &orig) { ms = orig.ms; }

    // Implicit conversion to unsigned long returns the elapsed time.
    operator unsigned long () const { return k_uptime_get_32() - ms; }

    // Copy assignment operator.
    elapsedMillis & operator = (const elapsedMillis &rhs) { ms = rhs.ms; return *this; }

    // Assignment from an unsigned long value: resets the timer such that the elapsed time equals the given value.
    elapsedMillis & operator = (unsigned long val) { ms = k_uptime_get_32() - val; return *this; }

    // Adjust the timer: add a delay.
    elapsedMillis & operator -= (unsigned long val) { ms += val; return *this; }

    // Adjust the timer: subtract a delay.
    elapsedMillis & operator += (unsigned long val) { ms -= val; return *this; }

    // Arithmetic operators that return a new elapsedMillis object.
    elapsedMillis operator - (int val) const           { elapsedMillis r(*this); r.ms += val; return r; }
    elapsedMillis operator - (unsigned int val) const    { elapsedMillis r(*this); r.ms += val; return r; }
    elapsedMillis operator - (long val) const            { elapsedMillis r(*this); r.ms += val; return r; }
    elapsedMillis operator - (unsigned long val) const   { elapsedMillis r(*this); r.ms += val; return r; }
    elapsedMillis operator + (int val) const             { elapsedMillis r(*this); r.ms -= val; return r; }
    elapsedMillis operator + (unsigned int val) const    { elapsedMillis r(*this); r.ms -= val; return r; }
    elapsedMillis operator + (long val) const            { elapsedMillis r(*this); r.ms -= val; return r; }
    elapsedMillis operator + (unsigned long val) const   { elapsedMillis r(*this); r.ms -= val; return r; }
};