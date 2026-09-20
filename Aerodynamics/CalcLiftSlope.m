function slope = CalcLiftSlope(AR, CL_alpha)
    arguments
        AR
        CL_alpha = 2*pi;
    end

    slope = pi*AR / (1+sqrt(1+(pi*AR/(CL_alpha))^2));
end