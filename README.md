TRABAJO FIN DE GRADO
INGENIERÍA EN INFORMÁTICA

EUROBOT-Software de los Robots de la competición.



Autor
MIGUEL PRIETO SEQUERA

Directores
SANTORO CORRADO, 
Università degli Studi di Catania



Escuela Técnica Superior de Ingenierías Informática y de Telecomunicación
Granada 21, Julio de 2017


Resumen
Eurobot es un concurso internacional de robótica amateur, abierto a jóvenes, organizado en proyectos de estudiantes o en clubes independientes de ciencias juveniles. Los robots construidos para este concurso son robots autónomos.
A través de competencias técnicas y gestión de proyectos, el concurso Eurobot pone de relieve valores como el fair-play, la solidaridad, el intercambio de conocimientos y la creatividad. El objetivo principal de este evento es animar a los jóvenes a practicar la robótica con diversión, ofreciéndoles una inolvidable experiencia técnica y humana. Cada año, se define un nuevo tema, con acciones originales para realizar y un 100 \% robot hecho a sí mismo para construir. La gestión de proyectos, el intercambio de tareas, la autonomía, el espíritu de equipo y la experimentación son los valores fundamentales para lograr el propio proyecto y estar dispuestos a competir en el  D-Day.

Proyecto
En este documento expongo mi trabajo realizado junto al equipo UNICT Team de la Universidad de Catania (Italia)
 (http://unict-team.dmi.unict.it).
Nuestro equipo desarrolló y construyó dos robots los cuales compitieron del 24 al 28 de Mayo 2017, en la Roche-sur-Yon, Francia.
Mi trabajo se encarga del software de los Robots.  Todo el firmware(código) del proyecto realizado se encuentra en el siguiente repositorio GITHUB: https://github.com/miguelprieto/TFG 



CARTEL DE COMPETICIÓN. Se muestran de manera general los componentes hardware de cada robot.



Objetivo de la competición: 
Básicamente consiste en recolectar el máximo número de diversos objetos de la superficie de juego la cual emula la superficie lunar en un tiempo determinado.

Dichos objetos pueden ser de dos tipos: rocas(pelotas), módulos de estación lunar(cilindros de pequeñas dimensiones).
 
Las rocas se encontraran en los cráteres hexagonales y en las semis circunferencias de las esquinas. Estas deben vaciarse en el lugar de partida del robot número 1(el grande).*

Los cilindros se encuentran repartidos por el tablero y en 4 puntos específicos (cohetes) en los límites del tablero. Estos deben vaciarse en las 5 zonas rectangulares que se observan en el tablero.

Todo este recorrido debe realizarse en un tiempo establecido y los robots del equipo partirán en una mitad al azar del tablero (fíjese que es simétrico), ya que competirán concurrentemente con otro equipo de robots que se encontraran en la mitad opuesta.

Nota: Aunque se parta de una mitad en concreto se pueden recolectar los objetos del tablero completo sin importar en que mitad inicie el juego nuestro equipo.

*En las siguientes imágenes se muestra la superficie del tablero así como los distintos itenerarios de los robots.



IDEA PRINCIPAL: 
El equipo consta de dos Robot, uno grande y otro pequeño para distintos fines(metas)

El robot grande se encargará de recolectar el máximo número de rocas en 90 segundos y  de vaciarlas fuera del tablero desde el lugar de partida. 

El robot pequeño se encargará de recolectar el máximo número de cilindros en 70 segundos y de vaciarlos a su antojo en los puntos rectangulares establecidos para ello.
Una vez acabado el tiempo se procede a la puntuación de cada equipo según el número de cilindros y rocas recolectadas, las cuales solo obtendrán la puntuación acorde si son depositadas de manera correcta en los diversos putos habilitantes para ello.




Tablero de competición que simula la superficie lunar.



Componentes y estrategia Robot Grande: 



A continuación se detalla el grafo de la estrategia del robot grande, la cual se detalla en GOALS. Los goals son metas, las cuales están claramente establecidas y estructuradas en el siguiente enlace:
Código → https://github.com/miguelprieto/TFG/tree/master/Microbrain/grande


1. Salir del punto de partida, en el cual se encuentra una rampa.

2. Avanzar al punto “Crater 1” en el cual recolectará los rocas de dicho cráter.

3. Una vez recolectado el punto 2, avanzar a la semi esfera para mediante una apertura total de las palas del robot controladas por un servo coger el número restante de rocas. Sorteando los módulos cilindricos que se encuentran por el tablero los cuales recogerá el robot pequeño.

4. Retorno. Volver al punto de partida con todas la bolas recolectadas.

5. Expulsar dichas bolas hacia un recipiente externo ubicado fuera del tablero.



Componentes y estrategia Robot Pequeño: 

Análogamente se detalla el grafo de la estrategia del robot pequeño, la cual se detalla en GOALS. Los goals son metas, las cuales están claramente establecidas y estructuradas en el siguiente enlace:
Código → 
https://github.com/miguelprieto/TFG/tree/master/Microbrain/piccolo





1. El robot pequeño sale desde CAP (nótese que los puntos son simétricos, y se completarán según de la mitad del tablero que se asigne al equipo al azar antes de empezar y que determinará el color del equipo ( amarillo o azul).

2. Recolecta los cilindros dispersos por el tablero cumpliendo dos objetivos, la recolecta en sí y dejar paso libre al robot grande. (El robot pequeño consta de 4 ventosas las cuales pueden recoger 4 cilindros, una vez recolectados los vacía en la BASE 1).

3. Una vez vaciados los cilindros en la base uno se dirige al DISP_1 (dispensador, “cohete” que suministra cilindros.

4. Recogerá otros 4 cilindros y se dispondrá a vaciarlos en la BASE_2.

5. *Si hubiese un obstáculo en la BASE_2 que impidiese el correcto depósito de los cilindros se dispondría el depósito en la BASE_3. (Rama central).



En el siguiente enlace se muestra un video con la prueba  de la anterior estrategia en desarrollo detallada en el párrafo anterior:
https://github.com/miguelprieto/TFG/blob/master/video/pruebas.mp4

*Este documento expone un resumen del trabajo realizado a nivel de estragegia.
*No entro en explicaciones de código el cual ya ha sido expuesto y evaluado en la Università degli Studi di Catania.


Yo, MIGUEL PRIETO SEQUERA alumno de la titulación GRADO EN INGENIERÍA INFORMÁTICA de la Escuela Técnica Superior de Ingenierías  Informática y de Telecomunicación de la Universidad de Granada, con DNI 77139481B, autorizo la ubicación de la siguiente copia de mi Trabajo Fin de Grado en la biblioteca del centro para que pueda ser consultada por las personas que lo deseen.


Fdo: MIGUEL PRIETO SEQUERA
Granada a 21 de Julio de 2017 


Agradecimientos
A la Università degli Studi di Catania por su acogida, concretamente a Corrado Santoro y a Fran Ruiz (SUBDIRECTOR DE ESCUELA TECNICA SUPERIOR DE INTERNACIONALIZACIÓN) por su ayuda y gestión.

