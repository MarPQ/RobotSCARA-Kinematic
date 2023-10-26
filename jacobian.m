function J = jacobian(bot,q)

theta_1 = q(1);
theta_2 = q(2);
d_3 = q(3);
theta_4 = q(4);

T01 = bot.A(1,[theta_1 theta_2 d_3 theta_4]);               %Creamos las matrices de transformacion necesarias
T02 = bot.A(1:2,[theta_1 theta_2 d_3 theta_4]);
T03 = bot.A(1:3,[theta_1 theta_2 d_3 theta_4]);
T04 = bot.A(1:4,[theta_1 theta_2 d_3 theta_4]);

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

J = [Jv; Jw];               %Concatenamos las matrices anteriores para formar la matriz jacobiana completa.

end