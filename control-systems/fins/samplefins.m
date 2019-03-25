N = 40;
X = zeros((2*N + 1)^3, 3);
Y = zeros((2*N + 1)^3, 3);

counter = 1;
for phi_1 = -N : N;
    for phi_2 = -N : N;
        for phi_3 = -N: N;
            Y(counter, :) = fins([phi_1 phi_2 phi_3], [0 0 0])';
            X(counter, :) = [phi_1 phi_2 phi_3 ];
            
            counter = counter + 1;
        end
    end
end

csvwrite('moments.csv', Y);
csvwrite('angles.csv', X);