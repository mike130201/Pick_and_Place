%% MATLAB Simulación del robot en Matlab.
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
% Matrices de transformación homogenea de cada articulación
T1 = Robot1.A(1,q);
T2 = Robot1.A(2,q);
T3 = Robot1.A(3,q);
T4 = Robot1.A(4,q);
%matriz que tiene la posición de la matriz T
pos=T.t
%% Cinemática Directa con Simbólico
%Se usan como simbólicos los parámetros de Denavit Hartenberg en este caso
%del 0 al 3
syms theta0 alpha0 d0 a0 theta1 alpha1 d1 a1 theta2 alpha2 d2 a2 theta3 alpha3 d3 a3  
%Para las matrices se hizo la multiplicación de cada parámetro
R1t=trotz(theta0);%se sustituyen los parámetros del simbólico
t1z=transl(0,0,d0);
R1a=trotx(alpha0);
t1x=transl(a0,0,0);
s0Ts1=R1t*t1z*R1a*t1x;%todo se multiplica para tener cada una de las matrices
% de transformación homogénea

%se repite el mismo proceso para cada una de las articulaciones.
R2t=trotz(theta1);
t2z=transl(0,0,d1);
R2a=trotx(alpha1);
t2x=transl(a1,0,0);
s1Ts2=R2t*t2z*R2a*t2x; 

R3t=trotz(theta2);%Articulación 3
t3z=transl(0,0,d2);
R3a=trotx(alpha2);
t3x=transl(a2,0,0);
s2Ts3=R3t*t3z*R3a*t3x;

R4t=trotz(theta3);%Articulación 4
t4z=transl(0,0,d3);
R4a=trotx(alpha3);
t4x=transl(a3,0,0);
s3Ts4=R4t*t4z*R4a*t4x;

%evaluamos la matriz
Matriz=s0Ts1*s1Ts2*s2Ts3*s3Ts4
%Esta matriz está completamente en simbólico
%evaluando la cinematica directa

%Se tienen que agregar todos los valores de las variables para que pueda
%ser sustiuido y que se puedan encontrar los valores que se esperan.

%Valores de la primera articulación
theta0=0;
alpha0=pi/2; 
d0=0.077; 
a0=0; 
%Valores de la segunda articulación
theta1=deg2rad(79.38); 
alpha1=0; 
d1=0; 
a1=0.130; 
%Valores de la tercera articulación
theta2=deg2rad(-79.38); 
alpha2=0; 
d2=0;
a2=0.124;
%Valores de la cuarta articulación
theta3=0; 
alpha3=0; 
d3=0;
a3=0.126;
%Se evalua cada uno de los valores de las articulaciones para poder
%encontrar sus transformaciones
aTb=eval(s0Ts1)
bTc=eval(s1Ts2)
cTd=eval(s2Ts3)
dTe=eval(s3Ts4)
%Finalmente se multiplican todas con el fin de comprobar con la matriz T
%generada en la parte previa
aTe=aTb*bTc*cTd*dTe
%Como se puede apreciar, sí son las mismas entonces se puede decir que el
%cálculo fue el correcto.

%% Cinemática inversa
%se nombra la matriz noap que es la del TCP
noap=T
%Se nombran en simbólico el número de articulaciones que se van a ocupar
syms q1 q2 q3 q4
%Se declaran las matrices de transfromación de cada articulación
aAb=trotz(q1)*transl([0 0 l1])*transl([0 0 0])*trotx(pi/2);
%Segunda articulación con el offset declarado
bAc=trotz(q2+deg2rad(79.38))*transl([0 0 0])*transl([lx 0 0])*trotx(0);
%Tercera articulación con el offset declarado
cAd=trotz(q3+deg2rad(-79.38))*transl([0 0 0])*transl([l4 0 0])*trotx(0);
%Cuarta articulación en simbólico
dAe=trotz(q4)*transl([0 0 0])*transl([l5 0 0])*trotx(0);
%Declaramos la matriz noap en simbólico para hacer el método
syms nx ny nz ox oy oz ax ay az px py pz
%Se agrega a la matriz noap la última columna para que sea homogénea
noap=[nx ox ax px;ny oy ay py;nz oz az pz;0 0 0 1]
%NOTA:Se tiene que privilegiar la posición antes de la orientación  
% Paso 1

aAb*bAc*cAd*dAe==noap;
inv(aAb)*noap==bAc*cAd*dAe;
p1izq=inv(aAb)*noap;
p1izq=simplify(p1izq);
p1der=bAc*cAd*dAe;
p1der=simplify(p1der);
p1izq==p1der
%Seleccionamos la ecuación de la columna 3 y fila 4 priorizando la posición 
ec1=p1izq(3,4)==p1der(3,4)
%q1
%Se sustituyen con la matriz noap las variables para resolver la ecuación
px=pos(1)
py=0
pz=pos(3)
%Se selecciona solución con la ecuación trigonométrica 
a=-py;
b=px;
%se selecciona la solución 4 de la tabla de ecuaciones trigonométricas
%Con la ecuación se obtienen 2 soluciones
q1_sol1=atan2(a,-b)
q1_sol2=atan2(-a,b)
%Se hace una evaluación para comprobar que si es corresponde a la matriz de
%transformación de la primera articulación
q1=q1_sol2;
%aAb1 tiene la primera matriz de transformación con el valor de las
%articulaciones encontradas
aAb1=eval(aAb)
    
% Paso 2
%se busca encontrar otra incognita que es q2.
inv(bAc)*inv(aAb)*noap==cAd*dAe;
p2izq=inv(bAc)*inv(aAb)*noap;
p2izq=simplify(p2izq);
p2der=cAd*dAe;
p2der=simplify(p2der);
p2izq==p2der;
%Una vez teniendo igualadas las ecuaciones de los pasos se elige 1 que
%tenga solamente q1 y q2.
ec2=p2izq(1,3)==p2der(1,3)
% hay que evaluar con q1_sol1 q1=0
q1=q1_sol2
ax=0;
ay=-1;
az=0;
q2_sol1=eval(ec2)
%se selecciona la solución 4 de la tabla de ecuaciones trigonométricas, no
%obstante el resultado de la operación es 0
a=0;
b=0;
%Con la ecuación se obtienen 2 soluciones
q2_sol1=atan2(a,-b)
q2_sol2=atan2(-a,b)
%evaluar todo el resultado que es q2=0
q1=q1_sol2;
q2=q2_sol1
%esta matriz con la configuración de q2 nos va a dar la segunda MTH.
bAc1=eval(bAc)
    
% Paso 3
%se busca encontrar otra incognita que es q3.
inv(cAd)*inv(bAc)*inv(aAb)*noap==dAe;
p3izq=inv(cAd)*inv(bAc)*inv(aAb)*noap
p3izq=simplify(p3izq);
p3der=dAe;
p3der=simplify(p3der);
p3izq==p3der;
%Una vez teniendo las ecuaciones igualadas se selecciona una para trabajar
ec3=p3izq(1,3)==p3der(1,3)
%Se evaluan las q que ya se tienen q1=0 y q2=0
q1=q1_sol2
q2=q2_sol1
%se evalua con las incognitas encontradas
q3_sol1=eval(ec3)
%De igual forma en el paso 2, se vuelve 0, en este caso se elige una
%ecuación que es la 4 pero no afecta al resultado (eso es principalmente
%para hacerlo con el otro q1 que es cuando está en -3.1416)
a=0;
b=0;
%se obtienen las soluciones 
q3_sol1=atan2(a,-b)
q3_sol2=atan2(-a,b)
q3=q3_sol1
%Evaluamos para comprobar que corresponda a las matrices
cAd1=eval(cAd)
%Como ya se tienen las otras incognitas, se puede encontrar la incognita 
% resultante que es q4. 
ec4=p3izq(1,4)==p3der(1,4)
%se le da el valor de las q's pasadas para poder evaluar
q1=q1_sol2
q2=q2_sol1
q3=q3_sol1
%evaluamos la ecuación
q4_sol1=eval(ec4)
%se seleccionó la ecuación 2
b=1;
%los resultados son los siguientes:
q4_sol1=atan2(sqrt(1-b^2),b)
q4_sol2=atan2(-sqrt(1-b^2),b)
%q4=0 en ambas respuestas, entonces se sustituye.
q4=q4_sol1;
%evaluamos la última matriz con los resultados encontrados
dAe1=eval(dAe)
%Finalmente, se multiplican todas las matrices encontradas respecto a cada
%q que se encontró, el resultado coincide con la matriz del TCP, esto
%indica que es correcto
aAb1*bAc1*cAd1*dAe1

%% Cinemática inversa con el software
%Con el comando ikine se encuentra la cinemática inversa.
%mask es para poner cuantos grados de libertad tiene, en este caso son 4
%por eso tiene cuatro unos que son las articulaciones que se ocupan.
cd=Robot1.ikine(T,'mask', [1 1 1 1 0 0])
% Se plotea otra figura
figure
%La nueva figura va a tener los valores que se obtuvieron con el cálculo de
%la cinemática inversa.
Robot1.plot(cd)