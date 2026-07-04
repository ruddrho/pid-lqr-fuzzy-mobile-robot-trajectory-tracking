function [v,w,state] = pidController(ref,state,cfg)
%PIDCONTROLLER Centerline PID tracking using lateral and heading errors.

dt = cfg.dt;
if isempty(state)
    state.integralLateral = 0;
    state.integralHeading = 0;
    state.previousLateral = ref.lateralError;
    state.previousHeading = ref.headingError;
    state.filteredDLateral = 0;
    state.filteredDHeading = 0;
end

state.integralLateral = clampValue( ...
    state.integralLateral + ref.lateralError*dt,-0.50,0.50);
state.integralHeading = clampValue( ...
    state.integralHeading + ref.headingError*dt,-0.40,0.40);

rawDLateral = (ref.lateralError-state.previousLateral)/dt;
rawDHeading = (ref.headingError-state.previousHeading)/dt;
state.filteredDLateral = 0.78*state.filteredDLateral + 0.22*rawDLateral;
state.filteredDHeading = 0.78*state.filteredDHeading + 0.22*rawDHeading;

KpLat = 2.40; KiLat = 0.06; KdLat = 0.22;
KpHead = 2.80; KiHead = 0.04; KdHead = 0.12;

lateralCorrection = KpLat*ref.lateralError + ...
    KiLat*state.integralLateral + KdLat*state.filteredDLateral;
headingCorrection = KpHead*ref.headingError + ...
    KiHead*state.integralHeading + KdHead*state.filteredDHeading;

w = ref.w - lateralCorrection - headingCorrection;
trackingSeverity = min(abs(ref.lateralError)/0.55 + ...
    abs(ref.headingError)/0.80,1);
v = (ref.v + 0.45*ref.longitudinalError)* ...
    max(0.42,1-0.48*trackingSeverity);

v = clampValue(v,0,cfg.maxLinearVelocity);
w = clampValue(w,-cfg.maxAngularVelocity,cfg.maxAngularVelocity);
state.previousLateral = ref.lateralError;
state.previousHeading = ref.headingError;
end

function value = clampValue(value,lower,upper)
value = min(max(value,lower),upper);
end
