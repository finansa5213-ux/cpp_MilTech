// ============================================================
// MissionProcessor.cpp — реалізація методів процесора.
// ============================================================

#include "MissionProcessor.h"
#include <iostream>

MissionProcessor::MissionProcessor(IConfigLoader* loader,
                                   ITargetProvider* provider,
                                   IBallisticSolver* solver)
    : loader_(loader), provider_(provider), solver_(solver),
      currentIdx_(0), initialized_(false)
{}

bool MissionProcessor::init(const char* /*configSource*/) {
    if (loader_ == nullptr || provider_ == nullptr || solver_ == nullptr) {
        std::cerr << "MissionProcessor::init — нульовий компонент" << std::endl;
        return false;
    }

    if (!loader_->load()) {
        std::cerr << "MissionProcessor::init — лоадер не завантажився" << std::endl;
        return false;
    }
    cfg_      = loader_->getConfig();
    ammo_     = loader_->getAmmoParams();
    dronePos_ = cfg_.startPos;

    if (provider_->getTargetCount() <= 0) {
        std::cerr << "MissionProcessor::init — провайдер не має цілей "
                     "(чи був викликаний loadFromFile перед init?)" << std::endl;
        return false;
    }

    currentIdx_  = 0;
    initialized_ = true;
    return true;
}

bool MissionProcessor::hasNext() const {
    if (!initialized_) return false;
    return currentIdx_ < provider_->getTargetCount();
}

DropPoint MissionProcessor::step() {
    DropPoint dp;
    if (!hasNext()) return dp;

    Target tgt = provider_->getTarget(currentIdx_);
    dp = solver_->solve(dronePos_, tgt,
                        cfg_.altitude, ammo_, cfg_.attackSpeed);
    dp.targetIndex = currentIdx_;
    currentIdx_++;
    return dp;
}

void MissionProcessor::reset() {
    currentIdx_ = 0;
}

void MissionProcessor::changeSolver(IBallisticSolver* s) {
    if (s != nullptr) solver_ = s;
}
