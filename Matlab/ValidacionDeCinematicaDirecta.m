%%Para saber la cinemática directa, se usa el fkine
%hacemos una matriz con la posición cero del robot
q=[0 0 0 0]
%se crea T que va a tener la matriz de transformación homogénea del efector
%final y ALL va a tener la multiplicación de las matrices
[T,ALL]=Robot1.fkine(q);
T %plotear la matriz del efector final
% Matrices de transformación homogénea de cada articulación
T1 = Robot1.A(1,q);
T2 = Robot1.A(2,q);
T3 = Robot1.A(3,q);
T4 = Robot1.A(4,q);
