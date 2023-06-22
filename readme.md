#Manual:

Se crea una estructura que guarda todo lo necesario para crear una task en FreeRTOS +
su deadline y deadline relativo.

En la inicialización, se les asigna una prioridad en base de su deadline inicial y se 
lanza el planificador.

El gestor (tarea edf) se lanza cada 10 milis y sortea las tareas dependiendo de su deadline
relativo, y actualiza sus prioridades.

Cada task actualiza su deadline relativo en cada iteración de su bucle infinito, así el 
gestor lo ve todo actualizado.

