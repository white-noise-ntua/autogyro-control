function [TMOMENT] = fins(phi_deg, heading)
rho = 1.225;    %air density


%%%     fin characteristics             %%%
%%%     a   Cl      Cd          Cm      %%%

fin = [ 0	0       0.01694     0
        1	0.2578	0.01574	-0.0309
        2	0.3736	0.01444	-0.0294
        3	0.4545	0.0144	-0.0214
        4	0.5365	0.01519	-0.0145
        5	0.6141	0.01674	-0.0077
        6	0.6863	0.01958	-0.0008
        7	0.7631	0.02351	0.0051
        8	0.8481	0.02881	0.0093
        9	0.923	0.03554	0.0147
        10	0.9669	0.04588	0.0217
        11	0.9273	0.06134	0.0309
        12	0.7976	0.08983	0.0208
        14	0.7012	0.1665	-0.017
        15	0.7034	0.18076	-0.0262
        16	0.7097	0.19371	-0.0356
        17	0.7276	0.20744	-0.0449
        18	0.7559	0.22128	-0.0527
        21	0.766	0.313	-0.077
        22	0.799	0.328	-0.084
        23	0.824	0.342	-0.096
        25	0.882	0.371	-0.114
        26	0.913	0.388	-0.121
        27	0.938	0.398	-0.134
        28	0.967	0.416	-0.141
        29	0.991	0.425	-0.155
        30	1.016	0.438	-0.164
        31	1.04	0.449	-0.176
        32	1.063	0.461	-0.186
        33	1.084	0.473	-0.197
        34	1.106	0.484	-0.208
        36	1.143	0.503	-0.231
        37	1.16	0.512	-0.242
        38	1.174	0.52	-0.253
        39	1.189	0.528	-0.265
        40	1.201	0.535	-0.277
        41	1.211	0.542	-0.288
        90	0       1.2     -.800   ];
chord = 0.06;
span = 0.035;     
A = span*chord; % fin area
Rmin = 0.053;
Rmax = 0.088;
fin_dist = 1/2*(Rmax^2-Rmin^2)/(Rmax-Rmin);    %distance from the center of the fin to Z axis 
h = 0.2;    % vertical distance of fins from CG 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

phi = phi_deg ./180*pi;

p = heading(1);
r = heading(2);
y = heading(3);

% Should be identity matrix
RM = transpose([cos(y) -sin(y) 0; sin(y) cos(y) 0; 0 0 1]*[cos(r) 0 sin(r); 0 1 0; -sin(r) 0 cos(r)]*[1 0 0; 0 cos(p) -sin(p); 0 sin(p) cos(p)]);
V = RM*[0;0;15]; %free strean vel

% Frame vectors for fins
n1 = [1;0;0];
n2 = [cos(2*pi/3) -sin(2*pi/3) 0; sin(2*pi/3) cos(2*pi/3) 0; 0 0 1]*n1;
n3 = [cos(2*pi/3) -sin(2*pi/3) 0; sin(2*pi/3) cos(2*pi/3) 0; 0 0 1]*n2;
n = [n1,n2,n3];


a = zeros(3,1);
L = zeros(3,3);
D = zeros(3,3);
M = zeros(3,3);

% Total lift drag and moments
TLIFT   = zeros(3,1);
TDRAG   = zeros(3,1);
TMOMENT = zeros(3,1);

% Loop over each fin
 for i = 1:3
     v = V - (transpose(n(:,i))*V)*n(:,i);
     a(i) = acos(([0,0,1]*v)/norm(v))+phi(i);
     e = cross(v,n(:,i))./norm(cross(n(:,i),v));
     L(:,i) = sign(a(i)).*interp1(fin(:,1),fin(:,2),abs(a(i)))*rho*A/2*norm(v)^2.*e;     
     D(:,i) = interp1(fin(:,1),fin(:,3),abs(a(i)))*rho*A/2*norm(v)^2.*v/norm(v);
     r = n(:,i).*fin_dist+ h.*[0;0;1]; 
     M(:,i) = cross(r,L(:,i))+ cross(r,D(:,i))+sign(a(i)).*interp1(fin(:,1),fin(:,4),abs(a(i)))*rho*A*chord/2*norm(v)^2*n(:,i);     
 end
 
 
 for i = 1:3
     TLIFT(i)   = sum(L(i,:));
     TDRAG(i)   = sum(D(i,:));
     TMOMENT(i) = sum(M(i,:));
 end
      
end