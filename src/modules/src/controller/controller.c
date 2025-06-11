#define DEBUG_MODULE "CONTROLLER"
#include "debug.h"

#include "cfassert.h"
#include "controller.h"
#include "controller_pid.h"
#include "controller_mellinger.h"
#include "controller_indi.h"
#include "controller_brescianini.h"
#include "controller_lee.h"

#include "autoconf.h"

#define DEFAULT_CONTROLLER ControllerTypePID
static ControllerType currentController = ControllerTypeAutoSelect;

static void initController();

typedef struct {
  void (*init)(void);
  bool (*test)(void);
  void (*update)(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const uint32_t tick);
  const char* name;
} ControllerFcns;

static ControllerFcns controllerFunctions[] = {
  {.init = 0, .test = 0, .update = 0, .name = "None"}, // Any
  {.init = controllerPidInit, .test = controllerPidTest, .update = controllerPid, .name = "PID"},
  {.init = controllerMellingerFirmwareInit, .test = controllerMellingerFirmwareTest, .update = controllerMellingerFirmware, .name = "Mellinger"},
  {.init = controllerINDIInit, .test = controllerINDITest, .update = controllerINDI, .name = "INDI"},
  {.init = controllerBrescianiniInit, .test = controllerBrescianiniTest, .update = controllerBrescianini, .name = "Brescianini"},
  {.init = controllerLeeFirmwareInit, .test = controllerLeeFirmwareTest, .update = controllerLeeFirmware, .name = "Lee"},
  #ifdef CONFIG_CONTROLLER_OOT
  {.init = controllerOutOfTreeInit, .test = controllerOutOfTreeTest, .update = controllerOutOfTree, .name = "OutOfTree"},
  #endif
};


void controllerInit(ControllerType controller) {
    // 1. Validation initiale (on le mantient)
    if (controller < 0 || controller >= ControllerType_COUNT) {
        return;
    }

    // 2. Si un correcteur est choisi (manuale)
    if (controller != ControllerTypeAutoSelect) {
        currentController = controller;
    } 
    // 3. Si non, utiliser app-config.txt/Kconfig
    else {
        #if defined(CONFIG_CONTROLLER_PID)
            currentController = ControllerTypePID;
        #elif defined(CONFIG_CONTROLLER_INDI)
            currentController = ControllerTypeINDI;
        #elif defined(CONFIG_CONTROLLER_MELLINGER)
            currentController = ControllerTypeMellinger;
        #elif defined(CONFIG_CONTROLLER_BRESCIANINI)
            currentController = ControllerTypeBrescianini;
        #elif defined(CONFIG_CONTROLLER_LEE)
            currentController = ControllerTypeLee;
        #elif defined(CONFIG_CONTROLLER_OOT)
            currentController = ControllerTypeOot;
        #else
            // 4. Si rien n'est défini, utiliser le correcteur par défaut
            DEBUG_PRINT("No controller defined, using default\n");
            currentController = DEFAULT_CONTROLLER;
        #endif
    }

    // 5. Initialisation
    initController();
    DEBUG_PRINT("Using %s (%d) controller\n", controllerGetName(), currentController);
}
ControllerType controllerGetType(void) {
  return currentController;
}

static void initController() {
  controllerFunctions[currentController].init();
}

bool controllerTest(void) {
  return controllerFunctions[currentController].test();
}

void controller(control_t *control, const setpoint_t *setpoint, const sensorData_t *sensors, const state_t *state, const stabilizerStep_t stabilizerStep) {
  controllerFunctions[currentController].update(control, setpoint, sensors, state, stabilizerStep);
}

const char* controllerGetName() {
  return controllerFunctions[currentController].name;
}
