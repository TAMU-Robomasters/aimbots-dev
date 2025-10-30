#pragma once
#include <drivers.hpp>
#include <tap/control/subsystem.hpp>

#ifdef JETSON_COMPATIBLE

namespace src::Jetson {

class JetsonSubsystem : public tap::control::Subsystem {
public:
    JetsonSubsystem(src::Drivers*);
    ~JetsonSubsystem() = default;

    void initialize() override;
    void refresh() override;

    const char* getName() const override { return "Jetson Subsystem"; }
private:
    
} 
    
} // namespace src::Jetson

#endif //#ifdef JETSON_COMPATIBLE

