function out = LEDextrema(in)
%% compute extrema and other from vector
% in  : position1 - position2
% out : [mean max min maxmindiff std var] 
    
    in2 = in .* in;
    normin = sqrt(sum(in2,2));
    
    out.data = in;
    out.norm = normin;
    out.mean = mean(normin);
    out.max = max(normin);
    out.min = min(normin);
    out.diff = abs(max(normin) - min(normin));
    out.std = std(normin);
    out.var = var(normin);
    out.offmean = normin - mean(normin);
    
    std_scale = 1;
    
    A = (out.norm<=(out.mean+(std_scale*out.std)));
    B = (out.norm>=(out.mean-(std_scale*out.std)));
    out.pmScaledSTD = A.*B;
    
    
% A = [tbstereo.RG.norm<=(tbstereo.RG.mean+tbstereo.RG.std) ...
%     tbstereo.RG.norm>=(tbstereo.RG.mean-tbstereo.RG.std) ...
%     tbstereo.BG.norm<=(tbstereo.BG.mean+tbstereo.RG.std) ...
%     tbstereo.BG.norm>=(tbstereo.BG.mean-tbstereo.RG.std)];

    

end
% 
% lRB2 = lRB .* lRB;
% lRG2 = lRG .* lRG;
% lBG2 = lBG .* lBG;
% 
% normRB = sqrt(sum(lRB2,2));
% normRG = sqrt(sum(lRG2,2));
% normBG = sqrt(sum(lBG2,2));
% 
% RB = extrema(normRB);
% RG = extrema(normRG);
% BG = extrema(normBG);
