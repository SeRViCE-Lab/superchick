function Lt=D6_to_leg_length(xyz,psi,j_w,j_u_2)

% xyz: the center of top panel in the inertial frame.
% psi: Euler angle (alp,bet,gam) in 123 convention. alp is the angle of the first rotation
% j_w: the joints in the bottom panel represented in the inertial frame.
% j_u_2: the joints in the top panel represented in the top panel frame.

% A vector expressed in (x,y,z) as v, and expressed in (X,Y,Z) as V.
% v=DCM*V.

ax=psi(1);ay=psi(2);az=psi(3); % angles around X, Y, and Z respectively

cx=cos(ax);sx=sin(ax); % cosine and sine of angle around X (alpha, pitch)
cy=cos(ay);sy=sin(ay); % cosine and sine of angle around Y (beta, roll)
cz=cos(az);sz=sin(az); % cosine and sine of angle around Z (gamma, yaw)
 
% 123 rotation matrix
DCM =[
  cy*cz, cx*sz + cz*sx*sy, sx*sz - cx*cz*sy
 -cy*sz, cx*cz - sx*sy*sz, cz*sx + cx*sy*sz
     sy,           -cy*sx,            cx*cy];

j_u=DCM'*j_u_2+repmat(xyz,1,6);
tt=j_w-j_u; Lt=sqrt(sum(tt.*tt));
