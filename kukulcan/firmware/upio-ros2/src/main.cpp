#include <Arduino.h>
#include "RTE.h"

// Inicializamos la capa RTE
void setup() { RTE_Init(); }

// Mantenemos la capa RTE corriendo
void loop() { RTE_Run(); }

