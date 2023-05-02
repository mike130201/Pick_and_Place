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
