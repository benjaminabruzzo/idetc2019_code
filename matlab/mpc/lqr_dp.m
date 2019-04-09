% @now
% P(N) = 0
% for t = N-1:-1:now
% K(i) = -(...)^(-1) * B' * P(i+1)
% P(i) = Q + K'(i)*R)*K(i)-A'...
% u_k = K
clear A B Q R N P K
clc

x = [10 0]';
A = [1 1 ; 0 1];
B = [0 0 ; 0 1];
Q = eye(2);
R = eye(2);

N = 20;
P{N} = zeros(size(B*B'));


for i = (N-1):-1:1
    K{i} = -(B'*P{(i + 1)}*B + R)^(-1)*B'*P{(i + 1)};
    P{i} = Q+A'*P{i+1}*A - A'*P{(i + 1)}*B*(B'*P{(i + 1)}*B + R)^(-1)* B*P{(i + 1)}*A;
end; clear i


clear X_i x_j 

X_i{1} = A*x+B*K{1}*x;
for j = 2:N
    X_i{j} = (A+B*K{j-1})*X_i{j-1};
end


for j = 1:N
%     disp(['X_i{' num2str(j) '} = ']); disp(X_i{j})
    x_j(j,:) = X_i{j}';
end


figure(1); clf
    hold on
        plot(x_j(:,1), 'rx')
        plot(x_j(:,2), 'bo')
    hold off
    grid on
    xlabel('N')



% for m = 1:5
%     for i = (N-1):-1:1
%         K{i} = -(B'*P{(i + 1)}*B + R)^(-1)*B'*P{(i + 1)};
%         P{i} = Q+A'*P{i+1}*A - A'*P{(i + 1)}*B*(B'*P{(i + 1)}*B + R)^(-1)* B*P{(i + 1)}*A;
%     end; clear i
%     disp("K{1} = "); disp(K{1})
%     disp("P{1} = "); disp(P{1})
% end
