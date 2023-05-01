%% Cinemática inversa con el software
close all
clear
%tamaño de los eslabones
l1=0.077;
l2=0.128;
l3=0.024;
l4=0.124;
l5=0.126;
%como el Open tiene un offset en la articulación 2 y 3 se toma la distancia
% de la hipotenusa que generan los eslabones de esas articulaciones 
lx=0.130;
%Articulaciones del robot con parámetros de Denavit Hartenberg
A(1)= Link([0 l1 0 pi/2 0]); %Generar cada una de las articulaciones 
A(2)= Link([0 0 lx 0 0]);%th,d,a,alpha, tipo art; 0=rot 1=pris
%Para poder graficar correctamente el robot, se tiene que agregar un offset
%a q2 debido a que se tiene que tomar en cuenta que está tomando la
%distancia de la hipotenusa de los eslabones de las articulaciones 2 y 3
A(2).offset=(deg2rad(79.38))
A(3)= Link([0 0 l4 0 0]);%Tercer articulación
%Se debe regresar el ángulo agregado debido a que no se tiene otro offset
A(3).offset=(deg2rad(-79.38)) % Los grados tienen que estar en radianes
A(4)= Link([0 0 l5 0 0]);%quinta articulación
% Crear objeto SerialLink para el robot
Robot1= SerialLink(A, 'name', 'RRRR');
%hacemos una matriz con la posición cero del robot
q=[0 0 0 0]
%se crea T que va a tener la matriz de transformación homogénea del efector
%final y ALL va a tener la multiplicación de las matrices
[T,ALL]=Robot1.fkine(q);
T %plotear la matriz del efector final
% Muestra la interfaz gráfica del robot
Robot1.teach
%matriz que tiene la posición de la matriz T
pos=T.t
%Con el comando ikine se encuentra la cinemática inversa.
%mask es para poner cuantos grados de libertad tiene, en este caso son 4
%por eso tiene cuatro unos que son las articulaciones que se ocupan.
cd=Robot1.ikine(T,'mask', [1 1 1 1 0 0])
% Se plotea otra figura
figure
%La nueva figura va a tener los valores que se obtuvieron con el cálculo de
%la cinemática inversa.
Robot1.plot(cd)