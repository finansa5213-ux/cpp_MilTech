#pragma once
// ============================================================
// Factory.h — фабричні функції створення компонентів.
//
// Це єдине місце в коді, де згадуються конкретні типи реалізацій
// (JsonTargetProvider, AnalyticalSolver, FileConfigLoader).
// Решта програми (зокрема MissionProcessor і main) працює лише
// з вказівниками на інтерфейси.
//
// Коли з'являться нові реалізації (TableSolver, SerialTargetProvider,
// HttpConfigLoader) — достатньо:
//   1) додати новий enumerator у відповідний enum class,
//   2) додати один case у відповідний createXxx().
// Решта програми не зміниться.
// ============================================================

#include "ITargetProvider.h"
#include "IBallisticSolver.h"
#include "IConfigLoader.h"

// --- Типи реалізацій (на майбутнє розширюваний enum) ---
enum class SolverType   { ANALYTICAL };
enum class ProviderType { JSON };
enum class LoaderType   { FILE };

// Фабричні функції повертають вказівники на ІНТЕРФЕЙСИ.
// Викликаючий бере на себе delete (бо клас, що володіє вказівником,
// один — наприклад, main() або MissionProcessor).
IBallisticSolver* createSolver(SolverType type);
ITargetProvider* createProvider(ProviderType type, const char* param);

// Для FileConfigLoader потрібні ДВА файли — config та ammo.
// Тому окрема сигнатура з двома параметрами.
IConfigLoader*   createLoader(LoaderType type,
                              const char* configPath,
                              const char* ammoPath);
