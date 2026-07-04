function [v,w,state] = lqrController(ref,state,cfg)
%LQRCONTROLLER Toolbox-free discrete LQR centerline tracking.

if isempty(state)
    state.lastGain = zeros(1,2);
end

vLinearization = max(ref.v,0.20);
A = [0 vLinearization;0 0];
B = [0;1];
Ad = eye(2) + A*cfg.dt;
Bd = B*cfg.dt;

Q = diag([14.0 7.0]);
R = 0.75;
P = Q;
for k = 1:80
    S = R + Bd'*P*Bd;
    Pnew = Ad'*P*Ad - Ad'*P*Bd*(S\(Bd'*P*Ad)) + Q;
    if norm(Pnew-P,'fro') < 1e-9
        P = Pnew;
        break;
    end
    P = Pnew;
end

K = (R + Bd'*P*Bd)\(Bd'*P*Ad);
state.lastGain = K;
errorState = [ref.lateralError;ref.headingError];
w = ref.w - K*errorState;

trackingSeverity = min(abs(ref.lateralError)/0.55 + ...
    abs(ref.headingError)/0.80,1);
v = (ref.v + 0.50*ref.longitudinalError)* ...
    max(0.45,1-0.50*trackingSeverity);

v = clampValue(v,0,cfg.maxLinearVelocity);
w = clampValue(w,-cfg.maxAngularVelocity,cfg.maxAngularVelocity);
end

function value = clampValue(value,lower,upper)
value = min(max(value,lower),upper);
end
