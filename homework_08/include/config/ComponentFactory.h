#pragma once
// ============================================================
// ComponentFactory — фабричні функції створення компонентів.
// ============================================================
#include "interfaces/ITargetProvider.h"
#include "interfaces/IBallisticSolver.h"
#include "interfaces/IConfigLoader.h"
#include <string>

enum class SolverType   { ANALYTICAL };
enum class ProviderType { JSON };
enum class LoaderType   { FILE };

IBallisticSolver* createSolver(SolverType type);
ITargetProvider*  createProvider(ProviderType type, const std::string& param);
IConfigLoader*    createLoader(LoaderType type,
                                const std::string& configPath,
                                const std::string& ammoPath);
