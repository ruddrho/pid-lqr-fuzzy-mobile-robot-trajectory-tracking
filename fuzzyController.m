function [v,w,state] = fuzzyController(ref,state,cfg)
%FUZZYCONTROLLER Toolbox-free Sugeno-style centerline controller.

if isempty(state)
    state.filteredOmega = ref.w;
end

normalizedLateral = clampValue(ref.lateralError/0.50,-1,1);
normalizedHeading = clampValue(ref.headingError/0.65,-1,1);
centers = [-1 -0.5 0 0.5 1];
muLateral = membershipVector(normalizedLateral,centers);
muHeading = membershipVector(normalizedHeading,centers);

numerator = 0;
denominator = 0;
for i = 1:numel(centers)
    for j = 1:numel(centers)
        weight = min(muLateral(i),muHeading(j));
        singleton = -2.30*(0.95*centers(i) + 1.20*centers(j));
        singleton = clampValue(singleton,-2.80,2.80);
        numerator = numerator + weight*singleton;
        denominator = denominator + weight;
    end
end

steeringCorrection = numerator/max(denominator,eps);
rawOmega = ref.w + steeringCorrection;
state.filteredOmega = 0.70*state.filteredOmega + 0.30*rawOmega;
w = state.filteredOmega;

severity = min(0.65*abs(normalizedLateral) + ...
    0.80*abs(normalizedHeading),1);
v = (ref.v + 0.40*ref.longitudinalError)* ...
    max(0.40,1-0.52*severity);

v = clampValue(v,0,cfg.maxLinearVelocity);
w = clampValue(w,-cfg.maxAngularVelocity,cfg.maxAngularVelocity);
end

function mu = membershipVector(value,centers)
width = 0.5;
mu = max(0,1-abs(value-centers)/width);
if value <= -1
    mu(1) = 1;
elseif value >= 1
    mu(end) = 1;
end
if sum(mu) <= eps
    [~,index] = min(abs(centers-value));
    mu(index) = 1;
end
end

function value = clampValue(value,lower,upper)
value = min(max(value,lower),upper);
end
