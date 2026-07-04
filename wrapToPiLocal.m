function angle = wrapToPiLocal(angle)
%WRAPTOPILOCAL Wrap angle to [-pi,pi).
angle = mod(angle + pi,2*pi) - pi;
end
