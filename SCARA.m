syms IL1 IL2 IL4 ML1 ML2 ML3 kr1 kr2 a1 a2 l1 l2 l3 ang1 ang2 ang4 d3 mm1 mm2 Im1 Im2
%Jacobian Matrix
Jp_L1=[-l1*sin(ang1),0; l1*cos(ang1), 0; 1,0];
Jp_L2=[-a1*sin(ang1)-l2*sin(ang1+ang2),-l2*sin(ang1+ang2);a1*cos(ang1)+l2*cos(ang1+ang2), l2*cos(ang1+ang2); 1,0];
Jp_L4=[-a1*sin(ang1)-a2*sin(ang1+ang2),  -a2*sin(ang1+ang2),0,0;
    -a1*cos(ang1)-a2*cos(ang1+ang2), -a2*cos(ang1+ang2),0,0;
    0,0,1,0];

Jo_l1=[0,0;0,0;1,0];
Jo_l2=[0,0;0,0;1,1];
Jo_l4=[0,0,0,0;0,0,0,0;1,1,0,1];
Jp_m1=[0,0;0,0;0,0];
Jp_m2=[-a1*sin(ang1),0;-a1*cos(ang1),0;0 0];
Jo_m1=[0,0;0,0;kr1,0];
Jo_m2=[0,0;0,0;1,kr2];
%indivual link inertia matrix
B2link1= ML1*Jp_L1'*Jp_L1 + IL1*Jo_l1'*Jo_l1 + mm1*Jp_m1'*Jp_m1 +  Im1*Jo_m1'*Jo_m1;
B2link2= ML2*Jp_L2'*Jp_L2 + IL2*Jo_l2'*Jo_l2 + mm2*Jp_m2'*Jp_m2 +  Im2*Jo_m2'*Jo_m2;
B2link4= ML3*Jp_L4'*Jp_L4 + IL4*Jo_l4'*Jo_l4;
S1 = simplify(B2link1);
S2 = simplify(B2link2);
S4=simplify(B2link4);
St=S1+S2;
% non zero value
M11=IL1 + ML1*l1^2 + kr1^2*Im1 +IL2 +ML2*(a1^2 +l2^2+2*a1*l2*cos(ang2))+Im2+mm2*a1^2;
M12=IL2+ML2*(l2^2+a1*l2*cos(ang2))+kr2*Im2;
M22=IL2+ML2*l2^2+kr2^2*Im2;
M_11=simplify(M11);
M_12=simplify(M12);
M_22=simplify(M22);
%inertia matrix for first two link
B11=[M_11, M_12, 0,0;
    M_12, M_22, 0,0;
    0, 0, 0,0;
    0,0,0,0];
%Link 4, non zero value
n11=ML3*(a1^2+a2^2+2*a1*a2*cos(ang2))+IL4;
n12=ML3*(a2^2+a1*a2*cos(ang2))+IL4;
n21=n12;
n14=IL4;
n41=n14;
n22=ML3*a2^2+IL4;
n24=IL4;
n42=n24;
n33=ML3;
n44=IL4;
%ineria matrix for link 4
B22=[n11,n12,0,n14;
    n21,n22,0,n24;
    0,0,n33,0;
    n41,n42,0,n44];
%inertia matrix
B=B11+B22;
%after simplification
B_Sim=simplify(B)
%%%%%
%%%%%
%Matrix C
%%%%%
%%%%%%
K=-ML2*a1*l2*sin(ang1)-ML3*a1*l2*sin(ang1);
syms dang1 dang2
C=[K*(dang2), K*(dang1+dang2),0,0;
    -K*(dang1),0,0,0;
    0,0,0,0;
    0,0,0,0];
C_sim=simplify(C)

%%%%Kinectic Energy
syms dPl1 dPl2 dPl4 omg1 omg2 omg4 Jl1 Jl2 Jl4 ML4
K1=0.5*ML1*dPl1'*dPl1 +0.5*omg1'*Jl1*omg1;
K2=0.5*ML2*dPl2'*dPl2 +0.5*omg2'*Jl2*omg2;
K4=0.5*ML4*dPl4'*dPl4 +0.5*omg4'*Jl4*omg4;

K_total=K1+K2+K4;
K_sim=simplify(K_total)
%%%Potenial Energy
syms g Pl1 Pl2 Pl3
U1=-ML1*g'*Pl1;
U2=-ML2*g'*Pl2;
U4=-ML3*g'*Pl3;
U_total=U1+U2+U4;
U_sim=simplify(U_total)

% G matrix    g is 9.8m/s^2
syms g
g2=[0;0;-ML3*g;0]














