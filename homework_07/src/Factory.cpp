// ============================================================
// Factory.cpp — реалізація фабричних функцій.
// ============================================================

#include "Factory.h"
#include "JsonTargetProvider.h"
#include "AnalyticalSolver.h"
#include "FileConfigLoader.h"
#include <iostream>
#include <cstdlib>
#include <cstring>

IBallisticSolver* createSolver(SolverType type) {
    switch (type) {
        case SolverType::ANALYTICAL:
            return new AnalyticalSolver();
    }
    std::cerr << "createSolver: невідомий тип" << std::endl;
    return nullptr;
}

// Допоміжна: розбір param-рядка формату "path|arrayTimeStep".
// Якщо роздільник '|' відсутній — використовуємо arrayTimeStep = 1.0.
// Так зберігаємо сигнатуру з ТЗ (createProvider приймає лише const char*),
// але можемо передати додатковий параметр, потрібний JsonTargetProvider
// для обчислення швидкості цілей.
static void parseProviderParam(const char* param,
                               char* pathOut, size_t pathCap,
                               float& arrayTimeStepOut)
{
    arrayTimeStepOut = 1.0f;
    if (param == nullptr) { pathOut[0] = '\0'; return; }

    const char* sep = strchr(param, '|');
    if (sep == nullptr) {
        // Просто шлях
        strncpy(pathOut, param, pathCap - 1);
        pathOut[pathCap - 1] = '\0';
        return;
    }
    // Шлях — до роздільника
    size_t pathLen = (size_t)(sep - param);
    if (pathLen >= pathCap) pathLen = pathCap - 1;
    memcpy(pathOut, param, pathLen);
    pathOut[pathLen] = '\0';

    // arrayTimeStep — після
    arrayTimeStepOut = (float)atof(sep + 1);
    if (arrayTimeStepOut <= 0.f) arrayTimeStepOut = 1.0f;
}

ITargetProvider* createProvider(ProviderType type, const char* param) {
    switch (type) {
        case ProviderType::JSON: {
            char  path[256];
            float ats = 1.0f;
            parseProviderParam(param, path, sizeof(path), ats);
            return new JsonTargetProvider(path, ats);
        }
    }
    std::cerr << "createProvider: невідомий тип" << std::endl;
    return nullptr;
}

IConfigLoader* createLoader(LoaderType type,
                            const char* configPath,
                            const char* ammoPath) {
    switch (type) {
        case LoaderType::FILE:
            return new FileConfigLoader(configPath, ammoPath);
    }
    std::cerr << "createLoader: невідомий тип" << std::endl;
    return nullptr;
}
