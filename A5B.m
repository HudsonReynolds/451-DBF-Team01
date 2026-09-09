function A5B(data)

% top level control plot

AircraftScissorPlot(data);


% %% Downwash Gradient
% DeltaEOverDeltaAlpha = Kappa * CL_Alpha_Wing / pi / AR;
% 
% %% Neutral Point
% x_n = x_ac + ((CL_Alpha_Tail) * (1 - DeltaEOverDeltaAlpha) * V_H) / (CL_Alpha_Wing + S_t / S_w * CL_Alpha_Tail * (1 - DeltaEOverDeltaAlpha));
% 
% %% Static Margin
% SM = .15;
% x_cg = x_n - SM;
% Length_tail = 0.4; %40 cm, 0.4m
% 
% %% Takeoff Rotation
% St_S = (CMO_Wing + CL_Rot * (x_cg - x_ac) - CM_EquivalentRotate) / (CLNoseUp_Tail * ((Length_tail / c) - x_cg + x_ac));
% 
% %% Stall Recovery
% St_S = (CMO_Wing + CL_Max * (x_cg - x_ac) - CM_requiredRecovery) / (CL_NoseDown_Tail * ((Length_tail / c) - x_cg + x_ac));
% 
% %% Stability Limit
% St_S = (x_cg - x_ac + SM) / ((1-DeltaEOverDeltaAlpha) * (l_t / c) - (x_cg - x_ac + SM));
% 
% %% Tail Volume
% V_H = S_t * L_t / S_w / c;

TrimAircraft(data); 

end