function A_d = A_d_num_ext(in1,in2,in3)
%A_D_NUM_EXT
%    A_D = A_D_NUM_EXT(IN1,IN2,IN3)

%    This function was generated by the Symbolic Math Toolbox version 8.5.
%    02-Sep-2020 16:33:07

h = in3(:,2);
l = in3(:,1);
u_lin1 = in2(1,:);
u_lin2 = in2(2,:);
x_lin3 = in1(3,:);
x_lin4 = in1(4,:);
x_lin5 = in1(5,:);
t2 = cos(x_lin3);
t3 = sin(x_lin3);
t4 = 1.0./l;
t5 = x_lin4-1.0;
t6 = x_lin5-1.0;
t7 = t5.*u_lin1;
t8 = t6.*u_lin2;
t9 = t7+t8;
A_d = reshape([1.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,(h.*t3.*t9)./2.0,h.*t2.*t9.*(-1.0./2.0),1.0,0.0,0.0,h.*t2.*u_lin1.*(-1.0./2.0),h.*t3.*u_lin1.*(-1.0./2.0),h.*t4.*u_lin1.*(-1.0./2.0),1.0,0.0,h.*t2.*u_lin2.*(-1.0./2.0),h.*t3.*u_lin2.*(-1.0./2.0),(h.*t4.*u_lin2)./2.0,0.0,1.0],[5,5]);