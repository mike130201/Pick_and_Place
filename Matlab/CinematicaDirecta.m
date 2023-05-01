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
