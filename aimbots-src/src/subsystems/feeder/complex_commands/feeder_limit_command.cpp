#include "feeder_limit_command.hpp"

#include "tap/communication/gpio/leds.hpp"
#include "tap/control/command.hpp"

#include "subsystems/feeder/control/feeder.hpp"
#include "utils/tools/common_types.hpp"
#include "utils/ref_system/ref_helper_turreted.hpp"

#include "drivers.hpp"

#ifdef FEEDER_COMPATIBLE

namespace src::Feeder {

bool limitPressed = false;
bool wantToShoot = false;
bool watchFire = false;
bool underHeat = false;
int displayState = -1;
double displayHeat = 420.0;
double heatRegenDis = 69.0;
uint32_t timeDis = 20;

FeederLimitCommand::FeederLimitCommand(
    src::Drivers* drivers,
    FeederSubsystem* feeder,
    src::Utils::RefereeHelperTurreted* refHelper,
    int UNJAM_TIMER_MS)
    : drivers(drivers),
      feeder(feeder),
      refHelper(refHelper),
      UNJAM_TIMER_MS(UNJAM_TIMER_MS) {
    addSubsystemRequirement(dynamic_cast<tap::control::Subsystem*>(feeder));
}

void FeederLimitCommand::initialize() {
    feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor);
    startupThreshold.restart(500);  // delay to wait before attempting unjam
    unjamTimer.restart(0);
    prevTime = tap::arch::clock::getTimeMilliseconds();
}

void FeederLimitCommand::execute() {
    updateBarrelHeat();
    limitPressed = feeder->getPressed();  
    wantToShoot = (drivers->remote.getSwitch(Remote::Switch::RIGHT_SWITCH) == Remote::SwitchState::UP || drivers->remote.getMouseL()==true || drivers->cvCommunicator.shouldFire());
    bool underHeat = barrelHeat >= 100;
    displayState = currState;
    watchFire = false;
    displayHeat = barrelHeat;
   // if(!startupThreshold.isExpired()){
        switch(currState){
            case loading:
                if(limitPressed){
                    currState = loaded;
                    feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor);
                    canShoot = true;
                }else{
                    feeder->ForFeederMotorGroup(KICKER, &FeederSubsystem::deactivateFeederMotor);
                    feeder->ForFeederMotorGroup(LOADER, &FeederSubsystem::activateFeederMotor);
                }
                break;
            case loaded:
                canShoot = underHeat;
                if(wantToShoot && underHeat){
                    currState = firing;
                    registerShot();
                    feeder->ForFeederMotorGroup(KICKER, &FeederSubsystem::activateFeederMotor);
                    canShoot = false;
                    //funny hero shoot noise
                    //drivers->canSoundSystem.play(src::communicators::can_sound_system::CanSoundSystem::SOUND_SHOOT, 20);
                }else{
                    feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor);
                }
                break;
            case firing:
                if(!limitPressed){
                    currState = loading;
                    feeder->ForFeederMotorGroup(KICKER, &FeederSubsystem::deactivateFeederMotor);
                    feeder->ForFeederMotorGroup(LOADER, &FeederSubsystem::activateFeederMotor);
                }else{
                    watchFire = true;
                    feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::activateFeederMotor);
                }
                break;
     //   }
    }
}

void FeederLimitCommand::updateBarrelHeat(){
    uint32_t currTime =  tap::arch::clock::getTimeMilliseconds();
    timeDis = currTime;
    uint32_t timeDiff = currTime - prevTime;
    prevTime = currTime;
    double heatLoss = (24.0/1000.0)*timeDiff;
    heatRegenDis = heatLoss;
    barrelHeat += heatLoss;
    if(barrelHeat > 200){barrelHeat = 200;}
};

void FeederLimitCommand::registerShot(){
    barrelHeat -= 100; 
};

void FeederLimitCommand::end(bool) { feeder->ForFeederMotorGroup(ALL, &FeederSubsystem::deactivateFeederMotor); }

bool FeederLimitCommand::isReady() { return true; }

bool FeederLimitCommand::isFinished() const { return false; }

}  // namespace src::Feeder
#endif
