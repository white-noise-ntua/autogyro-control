
heading = [0,0,0];
TargetMoment = [ 0.0264;-0.0152; 0.0000];
phi = [0, 0,0];
phi_d = phi;
Dstep = 1e-2;
D = zeros(3,3);
M = fins(phi,heading);
counter = 1;
%phi_old = ones(3,1).*80;
while(norm(M-TargetMoment)>= 0.01)
%    phi_old = phi;    
    M = fins(phi,heading);
    for j = 1:3
        phi_d = phi;
        phi_d(j) = phi(j)+Dstep;
        D(j,:) = (fins(phi_d,heading)-fins(phi,heading))./Dstep;        
    end
delta_phi = transpose(linsolve(D,-M+TargetMoment));
phi = phi + delta_phi;

    fprintf('iter %2i, phi %6.2f  %6.2f %6.2f, delta phi, %6.4f  %6.4f   %6.4f' ,counter,phi',delta_phi');
fprintf('\n');
counter = counter +1;
end
