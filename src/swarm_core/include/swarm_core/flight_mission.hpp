#pragma once

namespace swarm {
class FlightMission {
public:
    virtual ~FlightMission() = default;

    virtual void control_loop() = 0;
    virtual const char* name() const = 0;
};

}
