#pragma once
// ============================================================
// ComponentFactory — фабричні функції створення компонентів.
// Розумні вказівники: фабрики повертають std::unique_ptr
// замість голих вказівників.
// ============================================================
#include "interfaces/ITargetProvider.h"
#include "interfaces/IBallisticSolver.h"
#include "interfaces/IConfigLoader.h"
#include <memory>
#include <string>

enum class SolverType   { ANALYTICAL, TABLE };
enum class ProviderType { JSON };
enum class LoaderType   { FILE };

// Для TABLE — шлях до файлу таблиці (за замовчуванням ballistic_table.txt)
std::unique_ptr<IBallisticSolver> createSolver(
        SolverType type,
        const std::string& tablePath = "ballistic_table.txt");

std::unique_ptr<ITargetProvider>  createProvider(
        ProviderType type, const std::string& param);

std::unique_ptr<IConfigLoader>    createLoader(
        LoaderType type,
        const std::string& configPath,
        const std::string& ammoPath);
