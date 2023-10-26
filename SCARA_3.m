           %--------Universidad De Guadalajara--------%
%--------Centro Universitario De Ciencias Exactas E Ingenierias--------%
              %--------Sistemas Roboticos 1--------%
       %--------Pacheco Quintero Marco Antonio--------%

clear all; close all; clc;

L1 = Revolute('a',0.15,'alpha',0,'d',0.8);
L2 = Revolute('a',0.4,'alpha',pi,'d',0);               %Constrimos la tabla DH correspondiente al manipulador.
L3 = Prismatic('a',0,'alpha',0,'theta',0);
L4 = Revolute('a',0,'alpha',0,'d',0.2);
bot = SerialLink([L1 L2 L3 L4],'name','SCARA');

td = [0.3889; 0.3889; 0.4];                              %Posicion del actuador final deseado. 
%td = [0.5; 0.2; 0.1];

alpha = -pi/2;
%alpha = pi/2;
                                          
Rd = [cos(alpha) sin(alpha) 0;                            %Orientacion del actuador final deseado. 
     sin(alpha) -cos(alpha) 0; 
     0 0 -1];
 
q = [0; pi/4; 0; 0];                                          %Vector q inicial propuesto.
K = [1 0 0 0 0 0;
     0 1 0 0 0 0;
     0 0 1 0 0 0;
     0 0 0 1 0 0;
     0 0 0 0 1 0;
     0 0 0 0 0 1];                                            %matriz de ganancias K.

t = 0.1;                   %Paso para cada instante de tiempo.

N = 150;                   %Numero de iteraciones.
Q = zeros(N,4);            %En esta matriz guardaremos las posiciones resuntantes de cada articulacion.
Q_d = zeros(N,4);          %En esta matriz guardaremos las velocidades resuntantes de cada articulacion.

M = zeros(N,1);            %Vector donde se guardaran los indices de manipulabilidad para cada iteracion.

for i=1:N
    
    Tq_i = bot.fkine(q);             %Obtenemos la matriz de transformacion para el vector q actual.
    v = td - Tq_i(1:3,4);            %Error entre la posicion final deseada y la posicion actual del efector final.
    w = (1/2)*(cross(Tq_i(1:3,1),Rd(1:3,1)) + cross(Tq_i(1:3,2),Rd(1:3,2)) + cross(Tq_i(1:3,3),Rd(1:3,3)));
    e = [v; w];
    
    J = jacobian(bot,q);             %Construimos jacobiano.
    q_d = pinv(J)*(K*e);             %Obtenemos la cinematica diferencial para el actual.
    
    q = q + q_d*t;                   %Obtenemos el paso de integracion para el actual.
     
    Q(i,:) = q';                     %Guardamos las posiciones para la iteracion actual.
    Q_d(i,:) = q_d';                 %Guardamos las velocidades para la iteracion actual.
    
    M(i) = abs(sqrt(det(J'*J)));     %Indice de manipulabilidad. Manipulador redundante.
end

fprintf('Matriz de transformacion final\n');
disp(Tq_i);
                                                                     %Mostramos los resultados de posicion obtenidos.
fprintf('Comparando con el vector de posicion y orientacion deseados: \n');
display(td);
display(Rd);

figure (1)         %Graficamos las posiciones de cada articulacion conforme el tiempo.
hold on
plot(Q(:,1), 'LineWidth', 2, 'MarkerSize', 10);
plot(Q(:,2), 'LineWidth', 2, 'MarkerSize', 10);
plot(Q(:,3), 'LineWidth', 2, 'MarkerSize', 10);
plot(Q(:,4), 'LineWidth', 2, 'MarkerSize', 10);
title('Posiciones');
legend('q_1', 'q_2', 'q_3','q_4');
xlabel('iteraciones');
ylabel('radianes | metros');
grid on

figure(2)          %Graficamos las velocidades de cada articulacion conforme el tiempo.
hold on
plot(Q_d(:,1), 'LineWidth', 2, 'MarkerSize', 10);
plot(Q_d(:,2), 'LineWidth', 2, 'MarkerSize', 10);
plot(Q_d(:,3), 'LineWidth', 2, 'MarkerSize', 10);
plot(Q_d(:,4), 'LineWidth', 2, 'MarkerSize', 10);
title('Velocidades');
legend('q_1', 'q_2', 'q_3','q_4');
xlabel('iteraciones');
ylabel('radianes/s | metros/s');
grid on

figure(3)
plot(M, 'LineWidth', 2, 'MarkerSize', 10);
title('Manipulabilidad');
xlabel('iteraciones');
ylabel('\mu');
grid on