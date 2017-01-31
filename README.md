# dynamic_marker
This package implements a dynamic fiducial marker system for automatic quadcopter landing.
This system control the display of a marker on a external screen and implements the decision process in order to select the type and size of the marker to be displayed.


# System description and required packages

TUD_COOP_UV
is used as the basis package for quadcopter marker tracking

Modified ar_sys package
for the detection of Aruco Markers with the implementation of services for dynamic reconfiguration of the markers to be detected.

Modified whycon package
for the detection of whycon markers with the implementation of services for dynamic reconfiguration of the markers to be detected.

Modified cob_fiducials
for the detection of pitag markers  with the implementation of services for dynamic reconfiguration of the markers to be detected.

TODO: Create a node in charge of marker recognition for all kinds of markers. (maybe not worth the time).



# TUTORIAL
## Computadora principal (corre ROS)
- terminal 1:
```
roscore
```

- terminal 2:
```
rosrun rosbridge_server rosbridge_websocket
```

- terminal 3: 
```
rosrun dynamic_marker decision_process
```

- terminal 4: dynamic reconfigure (para probar que funcione) 
```
rqt
```
## Computadora display (corre openframeworks)

En otra compu, o en la misma compu corre el codigo de openframeworks. Ve a la página de openframeworks para que entiendas como se usa. Descargalo y descomprime en tu home folder.

Debes primero instalar el addon ofxLibwebsockets, en la página web te dicen como hacerlo. Una vez que tienes el addon instalado y seguiste todas las instrucciones de instalacion de openframeworks debes:

1. hacer una carpeta dentro del directorio raiz de openframeworks llamada myprojects/dynamic_markers. Tambien puede ser myapps/dynamic_markers
2. dentro de la carpeta dynamic_markers debes hacer:
  ```
  git clone https://github.com/raultron/dynamic_marker_display.git
  ```
3. luego entra a myprojects/dynamic_markers/dynamic_marker_display y ejecuta en el terminal:
  ```
  make
  ```
4. Si compila bien puedes correrlo haciendo
  ```
  make RunRelease
  ```
5. Para editar el codigo puedes usar qtcreator, en la pagina de openframeworks te dicen como hacerlo. Recuerda que debes editar la dirección ip de la computadora a la que te vas a conectar en el archivo ofApp.c en la linea numero 11 reemplazando "riker" por la dirección ip de la compu que esta corriendo el roscore.

De todas maneras en el flash drive de mi compu te copie las carpetas de mis proyectos por si acaso te falta algo:

El código del display marker esta bajo esta ruta:

openframeworks/of_v0.9.8_linux64_release/myprojects/dynamic_markers/dynamic_marker_display
