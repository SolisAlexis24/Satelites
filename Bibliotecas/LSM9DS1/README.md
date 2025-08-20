# Código relacionado a la IMU LSM9DS1

Este codigo implementa la logica basica para recolectar datos del LSM9DS1, orientada a un entorno multinucleo con el puerto I2C compartido por ambos nucleos

Si es necesario, acudir al directorio de ejemplos para conocer la implementacion de aplicaciones

Para incluir esta biblioteca en un proyecto hacer lo siguiente:

1. Crear un nuevo proyecto usando la extension de Raspberry Pi para Visual Studio Code

2. En el archivo CMakeLists.txt añadir/modificar las siguientes lineas:

    ```
    add_subdirectory(ruta/LSM9DS1)
    ```
    Donde ruta es la ruta absoluta o relativa al directorio donde se encuentra la biblioteca

    ```
    target_link_libraries(tu_proyecto
                        tus_bibliotecas
                        LSM9DS1) # <--- Lo que debes anadir
    ```

3. En las opciones de la extensión, presionar 'Clean Cmake' y después 'Configure Cmake'

4. Si todo salió bien, compilar y usar, sino, revisar el log de cmake para detectar el error (usualmente errores con la ruta del archivo)

***Nota***

Si el proyecto se generó sin ayuda de la extensión, realizar lo siguiente:

1. Crear el archivo CMakeLists.txt y personalizarlo segun las preferencias del usuario.
2. En dicho archivo añadir/modificar las siguientes líneas:
    ```
    add_subdirectory(ruta)
    ```
    Donde ruta es la ruta absoluta o relativa al directorio donde se encuentra la biblioteca

    ```
    target_link_libraries(tu_proyecto
                        tus_bibliotecas
                        LSM9DS1) # <--- Lo que debes añadir
    ```

3. Crear un directorio llamado build en el directorio del proyecto.
4. Abrir la terminal en el directorio build recien creado.
5. Ejecutar el comando ```cmake ..```, que generará los archivos de construcción del proyecto.

6. Si el proceso termina exitosamente, ejecutar ```make``` dentro del mismo directorio, esto compilará el proyecto y generará el archivo .u2f que se carga al microcontrolador. 

