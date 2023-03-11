function f = f_num_slip(in1,in2,in3)
%F_NUM_SLIP
%    F = F_NUM_SLIP(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.6.
%    28-Oct-2020 16:39:50

i_l = in3(:,4);
i_r = in3(:,3);
l = in3(:,1);
u1 = in2(1,:);
u2 = in2(2,:);
x3 = in1(3,:);
t2 = i_l-1.0;
t3 = i_r-1.0;
t4 = t2.*u2;
t5 = t3.*u1;
t6 = t4+t5;
f = [t6.*cos(x3).*(-1.0./2.0);t6.*sin(x3).*(-1.0./2.0);(t4-t5)./(l.*2.0)];
