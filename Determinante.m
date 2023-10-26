           %--------Universidad De Guadalajara--------%
%--------Centro Universitario De Ciencias Exactas E Ingenierias--------%
              %--------Sistemas Roboticos 1--------%
       %--------Pacheco Quintero Marco Antonio--------%

clear all; close all; clc;

a_1 = sym('a_1');                  %Creamos las variables simbolicas necesarias.
a_2 = sym('a_2');
d_1 = sym('d_1');
d_4 = sym('d_4');
alpha_2 = sym('alpha_2');

theta_1 = sym('theta_1');
theta_2 = sym('theta_2');
d_3 = sym('d_3');
theta_4 = sym('theta_2');

L1 = Revolute('a',a_1,'alpha',0,'d',d_1);
L2 = Revolute('a',a_2,'alpha',alpha_2,'d',0);               %Constrimos la tabla DH correspondiente al manipulador.
L3 = Prismatic('a',0,'alpha',0,'theta',0);
L4 = Revolute('a',0,'alpha',0,'d',d_4);
bot = SerialLink([L1 L2 L3 L4],'name','SCARA');

T01 = bot.A(1,[theta_1 theta_2 d_3 theta_4]);               %Creamos las matrices de transformacion necesarias
T02 = simplify(bot.A(1:2,[theta_1 theta_2 d_3 theta_4]));
T03 = simplify(bot.A(1:3,[theta_1 theta_2 d_3 theta_4]));
T04 = simplify(bot.A(1:4,[theta_1 theta_2 d_3 theta_4]));

t0 = [0; 0; 0];      
t1 = T01(1:3,4);
t2 = T02(1:3,4);                                              
t3 = T03(1:3,4);
t4 = T04(1:3,4);                                            %Obtenemos los vectores de traslacion y rotacion
                                                            %necesarios para construir la matriz jacobiana.

z0 = [0; 0; 1];
z1 = T01(1:3,3);
z2 = T02(1:3,3);
z3 = T03(1:3,3);

Jv = [cross(z0,t4-t0) cross(z1,t4-t1) z2 cross(z3,t4-t3)];    %Construimos la matriz jacobiana para velocidades lineales.
Jw = [z0 z1 [0; 0; 0] z3];                                    %Construimos la matriz jacobiana para velocidades angulares.

J = simplify([Jv; Jw]);               %Concatenamos las matrices anteriores para formar la matriz jacobiana completa.
J = subs(J,alpha_2,pi);               %Sustituimos alpha_2 con su valor numerico para obtener una
                                      %matriz jacobiana mas corta.
Det = simplify(det(J(1:3,1:3)));