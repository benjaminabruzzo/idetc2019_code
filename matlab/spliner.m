function [spline, vector_newtime, diff]  = spliner(timeofvector, vectorToBeSplined, timeOfRefVector, referenceVector)
    %% spline data and compare to ref
    % timeofvector : time vector corresponding to vector that will be splined
    % vectorToBeSplined : data vector to be splined
    % timeOfRefVector : time vector corresponding to times which will be interpolated
    % referenceVector : data vector corresponding to reference time to compute diff

    spline = csapi(timeofvector,vectorToBeSplined);
    vector_newtime = fnval(spline, timeOfRefVector);
    diff = vector_newtime - referenceVector;
end
