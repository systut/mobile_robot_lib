% *************************************************************************
%
% Function "Symbolic computation of the equations of motion"
%
% This code is for a differential drive model
%
% The script auto-generates MATLAB functions to describe the kinematics and
% dynamics. 
%
% INPUT:  - NONE
% OUTPUT: - NONE
% FILES:  
%
% where
% q = [x y phi].'
% is the vector of the generalized coordinates,
% dqdt = [dx dy dphi].'
% is the vector of generalized speeds, and
% p = [vr vl].'
% is a vector of system parameters.
%
%   Tran Viet Thanh tran.viet.thanh.du@tut.jp
%   Matlab R2018a                                                            31
%   (uses the symbolic math toolbox)   
%   10/03/2023
%   v21
function SymbolicComputationOfEoM()


    %% Definitions
    % Generalized coordinates ...
    %[x; y; theta]
    x = sym('x',[3,1]); 

    % initiate robot input - input
    %[vr,vl];
    u = sym('u',[2,1]); 


    %% KINEMATICS
    v = p * [1/2 1/2];

    w = p * [1/(2*l) -1/(2*l)];

    f = [cos(phi)*v;
         sin(phi)*v;
         w];

    A = [];

    B = [];


    %% Create MATLAB-functions:
    % identify the current file location, to place all functions there
    filename = mfilename('fullpath');
    [filepath,~,~] = fileparts(filename);
    % dummy variable for obj, so that these can be used within the CLASS
    syms obj real
    % for dynamics:
    matlabFunction(f,'file',[filepath,'/DynamicFunction'],'vars',{obj, q, p});
    matlabFunction(A,'file',[filepath,'/StateMatrix'],'vars',{obj, q, p});
    matlabFunction(B,'file',[filepath,'/ControlMatrix'],'vars',{obj, q, p});
end