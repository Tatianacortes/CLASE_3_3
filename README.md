# CLASE 3 III CORTE

¡Por supuesto! Aquí tienes el texto en formato de bloque de código Markdown listo para copiar y pegar en tu proyecto de GitHub:

# **ADRC: Active Disturbance Rejection Control**

## Introducción

**ADRC** (Active Disturbance Rejection Control) es una técnica de control propuesta por **Zhiqiang Gao** como alternativa al clásico **PID**. Esta técnica busca resolver los problemas del control PID, tales como:

- Control de sistemas no lineales.
- Mejora de la velocidad de respuesta.
- Control preciso de la ubicación de polos.

ADRC fue introducida alrededor de 2011 y está ganando popularidad, especialmente en **control de movimiento** (debido a su rápida respuesta) y **sistemas eléctricos**, donde es vital rechazar perturbaciones rápidamente para evitar daños a los componentes. También se utiliza en **sistemas eólicos**.

---

## Fundamentos del ADRC

ADRC es una técnica basada en el **espacio de estados**, que utiliza un **observador de estados extendido (ESO)** para estimar:

- Estados de la planta.
- Perturbaciones internas y externas.
- Dinámicas no lineales no modeladas.

Esto permite **rechazar perturbaciones** sin necesidad de modelos matemáticos complejos. Solo es necesario conocer:

- El **orden del sistema**.
- La **ganancia crítica o nominal**.

Por ejemplo, en un sistema no lineal, podemos modelar la parte lineal y dejar que el **ESO** se encargue de la no linealidad. Esto hace que ADRC sea **robusto** y adaptable a cambios en la planta.

---

## Características del ADRC

- El controlador puede comportarse como un **sistema integrador** (sin incluir acción integral explícita), eliminando el **error en estado estacionario** al tratarlo como una perturbación más.
- Inicialmente propuesto como un **controlador lineal (proporcional)**, pero puede extenderse a un controlador **no lineal** para manejar sistemas con no linealidades fuertes.

---

## Componentes del ADRC

1. **Generador de trayectorias**: Modelo matemático que representa la cinemática del sistema y perfiles de movimiento.
2. **Observador de estado extendido (ESO)**:
   - Estima estados, perturbaciones, errores y no linealidades.
3. **Controlador proporcional por retroalimentación de estados**.

---

## Ejemplo: Sistema No Lineal

La mayoría de los sistemas físicos (≈99%) son **no lineales**. Por ejemplo, en un tanque con área variable según la altura (paredes irregulares), para modelar el área respecto a la altura necesitaríamos una función matemática.

Sin embargo, en ADRC, podemos considerar el término **1/A(h)** como una constante, dejando que el **ESO** estime la no linealidad y la rechace como perturbación.

---

# **NADRC: Nonlinear ADRC**

En **NADRC**, se introducen conceptos adicionales:

- \( a_0, a_1 \): Parámetros físicos del sistema.
- \( b \): Ganancia crítica o nominal.
- \( w \): Perturbaciones.
- \( f \): Función no lineal.

El **ESO** estima los estados al comparar la salida real del sistema con la salida del modelo matemático. La diferencia es el **error de estimación**, que se corrige mediante retroalimentación para eliminar errores en estado estacionario. Si la planta cambia, el observador ajusta la estimación automáticamente.

### Consideraciones

- Las funciones en NADRC pueden ser complejas, ya que deben modelar la dinámica de las perturbaciones.  
- **No es común** usar funciones complicadas a menos que el sistema tenga **no linealidades muy fuertes**.

---

# **LADRC: Linear ADRC**

En **LADRC** (Linear ADRC):

- El **observador** no requiere una función compleja, sino solo **constantes**.
- Se deben calcular los coeficientes:
  - \( K_1, K_2 \) (Controlador)
  - \( L_1, L_2, L_3 \) (Observador)

Esto **reduce la complejidad** del sistema.

---

## Diseño del LADRC

1. El **observador de estados** se implementa sumando:
   - El modelo del sistema.
   - La matriz de ganancias de retroalimentación multiplicada por el error de estimación.
2. El modelo se puede obtener mediante:
   - Curva de reacción del proceso.
   - Herramientas de estimación de estados en MATLAB.
   - Aproximación del tiempo muerto para simplificar a un sistema de **segundo orden** (mínimo dos estados).

### Estimación de perturbaciones

- Se añade un estado adicional \( d \) para estimar las perturbaciones.
- La matriz se extiende para incluir \( d \).

---

## Ubicación de Polos

Los coeficientes \( \lambda \) (lambdas) multiplican los estados con error. Esto da lugar a un **polinomio característico** cuyo objetivo es:

- Hacer que el error tienda a cero rápidamente.
- Asegurar **estabilidad** (polos en el semiplano izquierdo).
- Ubicar polos **más a la izquierda** para mayor velocidad.
- Evitar componentes imaginarias para reducir oscilaciones.

---

## Resumen del Diseño

El diseño del controlador ADRC/NADRC/LADRC se centra en:

- **Rechazar perturbaciones**.
- Ubicar los polos deseados para obtener la dinámica deseada:
  - **Estabilidad**.
  - **Rapidez** de respuesta.
- La dinámica del **observador** debe ser **más rápida** que la del sistema, permitiendo rechazar perturbaciones de forma efectiva.


¿Te gustaría que le agregara un diagrama o algún pseudocódigo para complementar? 🚀


