function T = arckinematics(c)
% ARCKINEMATICS Takes as input a vector c = [k, phi, l] and returns the corresponding
% homogenous transformation matrix T and the arc link as a
% sequence of points

k   = c(1);
phi = c(2);
l   = c(3);

% xRot = [0 -1 0 0;
%         1 0 0 0;
%         0 0 0 0;
%         0 0 0 0];
% 
% xInp = @(k) [0 0 k 0;
%              0 0 0 0;
%             -k 0 0 1;
%              0 0 0 0];
% 
% T = expm(xRot * phi) * expm(xInp(k) * l);

T = [cos(phi) * cos(k*l), -sin(phi), cos(phi) * sin(k*l), cos(phi)/k * (1 - cos(k*l));
     sin(phi) * cos(k*l),  cos(phi), sin(phi) * sin(k*l), sin(phi)/k * (1 - cos(k*l));
    -sin(k*l), 0, cos(k*l), sin(k*l)/k;
    0, 0, 0, 1];
     
% link = make_link(c);

end

