# UJI ROBOTICS - ERC2024

### Normas de código
#### Basadas en [la guía de estilo de C++ de Google](https://google.github.io/styleguide/cppguide.html) y [la guía de estilo y estándares de C++ de la NASA](https://ntrs.nasa.gov/citations/20080039927)

* Usa 3 espacios para indentar
Si no se usan espacios, el código puede verse diferente en diferentes editores. Se puede configurar el editor para que al pulsar la tecla tab se inserten 3 espacios en lugar de un tabulador.

* Uso de espacios
```
// Luego de una coma o un punto y coma
int a, b, c;
for (i = 0; i < 10; i++)

// Luego de una keyword
for (...)
while (...)
if (...)

// Entre operadores condicionales
x = (a > b) ? a : b;
```

* Estructura de bucles y ramas
```
if (condition) {                   // Good - no spaces inside parentheses, space before brace.
  DoOneThing();                    // Good - two-space indent.
  DoAnotherThing();
} else if (int a = f(); a != 3) {  // Good - closing brace on new line, else on same line.
  DoAThirdThing(a);
} else {
  DoNothing();
}

// Good - the same rules apply to loops.
while (condition) {
  RepeatAThing();
}

// Good - the same rules apply to loops.
do {
  RepeatAThing();
} while (condition);

// Good - the same rules apply to loops.
for (int i = 0; i < 10; ++i) {
  RepeatAThing();
}
```

* (C++) Evita usar la keyword 'using'

* (C++) Usa la deducción de tipo solo cuando sea necesario
Utiliza 'auto' únicamente cuando el tipo de variable sea obvio, como en un template. Si no, especifica el tipo de variable.

* No utilices excepciones
Las excepciones pueden comportarse de manera inesperada y resultar en fugas de memoria o pérdida de datos.

* No uses recursividad
La recursividad es difícil de depurar y puede causar problemas de rendimiento. En su lugar, usa bucles.

* Limita todos los bucles
Los bucles deben tener un límite superior representado por un entero. Si no, es posible que se produzca un bucle infinito.
```
while condicion:
   ...
```
pasa a ser:
```
n = 0
while condicion and n < 100:
   ...
   n += 1
```

* (C++) Evita usar memoria dinámica
La memoria dinámica, con funciones como new, delete, malloc o free, es difícil de depurar y puede causar fugas de memoria. En su lugar, usa contenedores STL como std::vector, std::array, std::map, std::set, etc.

* Limita el tamaño de las funciones
Las funciones deben hacer una única tarea y no ser muy largas. Si no, es posible que sean difíciles de entender. Si una función es demasiado larga, divídela en funciones más pequeñas.

* (C++) Compila comprobando todos los avisos
Usa todos los avisos posibles:
```
g++ -Wall -Werror -Wextra -pedantic
```

### Conventional Commits
Sigue la estructura:
```
<tipo>(ámbito opcional): <descripción>
```

[más información](https://dev.to/achamorro_dev/conventional-commits-que-es-y-por-que-deberias-empezar-a-utilizarlo-23an)
