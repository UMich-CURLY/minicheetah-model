% TODO

function ExportJacobians_IMU(obj, export_function, export_path)
% Computes the Manipulator Jacobians to be used for state estimation (IMU to contact)
%
%   Author: Dingyi Sun
%   Date:   01/07/2020
%
% Encoder Vector
encoders = SymVariable(obj.States.x(7:end));

% Frames
body_frame = CoordinateFrame(...
    'Name','Body',...
    'Reference',obj.Joints(getJointIndices(obj,'BaseRotX')),...
    'Offset',[0, 0, 0],...
    'R',[0, 0, 0]);

H_WB = body_frame.computeForwardKinematics; % assume the body frame is the IMU frame
H_WFL = obj.ContactPoints.FrontLeftFoot.computeForwardKinematics;
H_WFR = obj.ContactPoints.FrontRightFoot.computeForwardKinematics;
H_WHL = obj.ContactPoints.HindLeftFoot.computeForwardKinematics;
H_WHR = obj.ContactPoints.HindRightFoot.computeForwardKinematics;

H_B1FL = H_WB\H_WFL; H_B1FL = subs(H_B1FL, obj.States.x(1:6), zeros(6,1));
H_B1FR = H_WB\H_WFR; H_B1FR = subs(H_B1FR, obj.States.x(1:6), zeros(6,1));
H_B1HL = H_WB\H_WHL; H_B1HL = subs(H_B1HL, obj.States.x(1:6), zeros(6,1));
H_B1HR = H_WB\H_WHR; H_B1HR = subs(H_B1HR, obj.States.x(1:6), zeros(6,1));

% Compute Jacobians

% World to body frame (body and spatial Jacobians)
Jb_WB = body_frame.computeBodyJacobian(18);
Jb_WB = Jb_WB([4:6,1:3],:);

Js_WB = body_frame.computeSpatialJacobian(18);
Js_WB = Js_WB([4:6,1:3],:);

% World to Left Contact and Base to Right Contact (body and spatial Jacobians)
Jb_WFL = obj.ContactPoints.FrontLeftFoot.computeBodyJacobian(18);
Jb_WFL = Jb_WFL([4:6,1:3],:);

Js_WFL = obj.ContactPoints.FrontLeftFoot.computeSpatialJacobian(18);
Js_WFL = Js_WFL([4:6,1:3],:);

Jb_WFR = obj.ContactPoints.FrontRightFoot.computeBodyJacobian(18);
Jb_WFR = Jb_WFR([4:6,1:3],:);

Js_WFR = obj.ContactPoints.FrontRightFoot.computeSpatialJacobian(18);
Js_WFR = Js_WFR([4:6,1:3],:);

Jb_WHL = obj.ContactPoints.HindLeftFoot.computeBodyJacobian(18);
Jb_WHL = Jb_WHL([4:6,1:3],:);

Js_WHL = obj.ContactPoints.HindLeftFoot.computeSpatialJacobian(18);
Js_WHL = Js_WHL([4:6,1:3],:);

Jb_WHR = obj.ContactPoints.HindRightFoot.computeBodyJacobian(18);
Jb_WHR = Jb_WHR([4:6,1:3],:);

Js_WHR = obj.ContactPoints.HindRightFoot.computeSpatialJacobian(18);
Js_WHR = Js_WHR([4:6,1:3],:);

% Compute VectorNav to Contact Jacobians
Jb_BFL = -Adjoint(inv(H_B1FL))*Jb_WB + Jb_WFL;
Jb_BFL = subs(Jb_BFL, obj.States.x(1:6), zeros(6,1));
Jb_BFL = Jb_BFL(:,7:end);

Js_BFL = -Adjoint(inv(H_B1FL))*Js_WB + Js_WFL;
Js_BFL = subs(Js_BFL, obj.States.x(1:6), zeros(6,1));
Js_BFL = Js_BFL(:,7:end);

Jb_BFR = -Adjoint(inv(H_B1FR))*Jb_WB + Jb_WFR;
Jb_BFR = subs(Jb_BFR, obj.States.x(1:6), zeros(6,1));
Jb_BFR = Jb_BFR(:,7:end);

Js_BFR = -Adjoint(inv(H_B1FR))*Js_WB + Js_WFR;
Js_BFR = subs(Js_BFR, obj.States.x(1:6), zeros(6,1));
Js_BFR = Js_BFR(:,7:end);

Jb_BHL = -Adjoint(inv(H_B1HL))*Jb_WB + Jb_WHL;
Jb_BHL = subs(Jb_BHL, obj.States.x(1:6), zeros(6,1));
Jb_BHL = Jb_BHL(:,7:end);

Js_BHL = -Adjoint(inv(H_B1HL))*Js_WB + Js_WHL;
Js_BHL = subs(Js_BHL, obj.States.x(1:6), zeros(6,1));
Js_BHL = Js_BHL(:,7:end);

Jb_BHR = -Adjoint(inv(H_B1HR))*Jb_WB + Jb_WHR;
Jb_BHR = subs(Jb_BHR, obj.States.x(1:6), zeros(6,1));
Jb_BHR = Jb_BHR(:,7:end);

Js_BHR = -Adjoint(inv(H_B1HR))*Js_WB + Js_WHR;
Js_BHR = subs(Js_BHR, obj.States.x(1:6), zeros(6,1));
Js_BHR = Js_BHR(:,7:end);
% ---- Export Functions ----
export_function(Jb_BFL, 'Jb_Body_to_FrontLeftFoot', export_path, encoders);
export_function(Jb_BFL(1:3,:), 'Jwb_Body_to_FrontLeftFoot', export_path, encoders);
export_function(Jb_BFL(4:6,:), 'Jvb_Body_to_FrontLeftFoot', export_path, encoders);

export_function(Js_BFL, 'Js_Body_to_FrontLeftFoot', export_path, encoders);
export_function(Js_BFL(1:3,:), 'Jws_Body_to_FrontLeftFoot', export_path, encoders);
export_function(Js_BFL(4:6,:), 'Jvs_Body_to_FrontLeftFoot', export_path, encoders);

export_function(Jb_BFR, 'Jb_Body_to_FrontRightFoot', export_path, encoders);
export_function(Jb_BFR(1:3,:), 'Jwb_Body_to_FrontRightFoot', export_path, encoders);
export_function(Jb_BFR(4:6,:), 'Jvb_Body_to_FrontRightFoot', export_path, encoders);

export_function(Js_BFR, 'Js_Body_to_FrontRightFoot', export_path, encoders);
export_function(Js_BFR(1:3,:), 'Jws_Body_to_FrontRightFoot', export_path, encoders);
export_function(Js_BFR(4:6,:), 'Jvs_Body_to_FrontRightFoot', export_path, encoders);

export_function(Jb_BHL, 'Jb_Body_to_HindLeftFoot', export_path, encoders);
export_function(Jb_BHL(1:3,:), 'Jwb_Body_to_HindLeftFoot', export_path, encoders);
export_function(Jb_BHL(4:6,:), 'Jvb_Body_to_HindLeftFoot', export_path, encoders);

export_function(Js_BHL, 'Js_Body_to_HindLeftFoot', export_path, encoders);
export_function(Js_BHL(1:3,:), 'Jws_Body_to_HindLeftFoot', export_path, encoders);
export_function(Js_BHL(4:6,:), 'Jvs_Body_to_HindLeftFoot', export_path, encoders);

export_function(Jb_BHR, 'Jb_Body_to_HindRightFoot', export_path, encoders);
export_function(Jb_BHR(1:3,:), 'Jwb_Body_to_HindRightFoot', export_path, encoders);
export_function(Jb_BHR(4:6,:), 'Jvb_Body_to_HindRightFoot', export_path, encoders);

export_function(Js_BHR, 'Js_Body_to_HindRightFoot', export_path, encoders);
export_function(Js_BHR(1:3,:), 'Jws_Body_to_HindRightFoot', export_path, encoders);
export_function(Js_BHR(4:6,:), 'Jvs_Body_to_HindRightFoot', export_path, encoders);
end


function [ Ad ] = Adjoint( X )
%ADJOINT_SE3 Computes the adjoint of SE(3)
Ad = [X(1:3,1:3), zeros(3);
      skew(X(1:3,4))*X(1:3,1:3), X(1:3,1:3)];
end

function [Ax] = skew(v)
% Convert from vector to skew symmetric matrix
Ax = [    0, -v(3),  v(2);
       v(3),     0, -v(1);
      -v(2),  v(1),     0];
end
