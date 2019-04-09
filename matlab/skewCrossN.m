function [Matrix] = skewCrossN(NByThreeVector)
%% skewCrossN(NByThreeVector) 
% computes the skew cross product of a N-long set of row vectors

[N,m] = size(NByThreeVector);
   Matrix = zeros(3,3,N);

   for idx = 1:N
      NByThreeVector(idx,:) = NByThreeVector(idx,:)/norm(NByThreeVector(idx,:));
   end
   
%    Matrix(1,1,:) =  zeros(N,1);
%    Matrix(1,2,:) =  NByThreeVector(:,3);
%    Matrix(1,3,:) = -NByThreeVector(:,2);
% 
%    Matrix(2,1,:) = -NByThreeVector(:,3);
%    Matrix(2,2,:) =  zeros(N,1);
%    Matrix(2,3,:) =  NByThreeVector(:,1);
% 
%    Matrix(3,1,:) =  NByThreeVector(:,2);
%    Matrix(3,2,:) = -NByThreeVector(:,1);
%    Matrix(3,3,:) =  zeros(N,1);

   Matrix(1,1,:) =  zeros(N,1);
   Matrix(1,2,:) = -NByThreeVector(:,3);
   Matrix(1,3,:) =  NByThreeVector(:,2);

   Matrix(2,1,:) =  NByThreeVector(:,3);
   Matrix(2,2,:) =  zeros(N,1);
   Matrix(2,3,:) = -NByThreeVector(:,1);

   Matrix(3,1,:) = -NByThreeVector(:,2);
   Matrix(3,2,:) =  NByThreeVector(:,1);
   Matrix(3,3,:) =  zeros(N,1);

end