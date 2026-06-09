// ============================================================
// ComponentFactory.cpp
// Усі new замінені на std::make_unique. Функції повертають
// std::unique_ptr — власність переходить до викликаючого коду.
// ============================================================

#include "config/ComponentFactory.h"
#include "providers/JsonTargetProvider.h"
#include "solvers/AnalyticalSolver.h"
#include "solvers/TableSolver.h"
#include "config/FileConfigLoader.h"
#include <iostream>

std::unique_ptr<IBallisticSolver> createSolver(SolverType type,
                                               const std::string& tablePath)
{
    switch (type) {
        case SolverType::ANALYTICAL:
            return std::make_unique<AnalyticalSolver>();
        case SolverType::TABLE:
            return std::make_unique<TableSolver>(tablePath);
    }
    std::cerr << "createSolver: невідомий тип\n";
    return nullptr;
}

// Розбирає "path|arrayTimeStep"
static void parseProviderParam(const std::string& param,
                                std::string& pathOut,
                                float& atsOut)
{
    atsOut = 1.0f;
    auto sep = param.find('|');
    if (sep == std::string::npos) {
        pathOut = param;
        return;
    }
    pathOut = param.substr(0, sep);
    try   { atsOut = std::stof(param.substr(sep + 1)); }
    catch (...) { atsOut = 1.0f; }
    if (atsOut <= 0.f) atsOut = 1.0f;
}

std::unique_ptr<ITargetProvider> createProvider(ProviderType type,
                                                const std::string& param)
{
    switch (type) {
        case ProviderType::JSON: {
            std::string path;
            float ats = 1.0f;
            parseProviderParam(param, path, ats);
            return std::make_unique<JsonTargetProvider>(path, ats);
        }
    }
    std::cerr << "createProvider: невідомий тип\n";
    return nullptr;
}

std::unique_ptr<IConfigLoader> createLoader(LoaderType type,
                                            const std::string& configPath,
                                            const std::string& ammoPath)
{
    switch (type) {
        case LoaderType::FILE:
            return std::make_unique<FileConfigLoader>(configPath, ammoPath);
    }
    std::cerr << "createLoader: невідомий тип\n";
    return nullptr;
}
