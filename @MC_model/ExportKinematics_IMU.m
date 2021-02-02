% TODO

function ExportKinematics_IMU(obj, export_function, export_path)
% Computes the Forward Kinematics to be used for state estimation (IMU to contact)
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

% Export Functions

export_function(H_B1FL, 'H_Body_to_FrontLeftFoot', export_path, encoders);
export_function(H_B1FL(1:3,1:3), 'R_Body_to_FrontLeftFoot', export_path, encoders);
export_function(H_B1FL(1:3,end), 'p_Body_to_FrontLeftFoot', export_path, encoders);
export_function(jacobian(H_B1FL(1:3,end),encoders), 'Jp_Body_to_FrontLeftFoot', export_path, encoders);

export_function(H_B1FR, 'H_Body_to_FrontRightFoot', export_path, encoders);
export_function(H_B1FR(1:3,1:3), 'R_Body_to_FrontRightFoot', export_path, encoders);
export_function(H_B1FR(1:3,end), 'p_Body_to_FrontRightFoot', export_path, encoders);
export_function(jacobian(H_B1FR(1:3,end),encoders), 'Jp_Body_to_FrontRightFoot', export_path, encoders);

export_function(H_B1HL, 'H_Body_to_HindLeftFoot', export_path, encoders);
export_function(H_B1HL(1:3,1:3), 'R_Body_to_HindLeftFoot', export_path, encoders);
export_function(H_B1HL(1:3,end), 'p_Body_to_HindLeftFoot', export_path, encoders);
export_function(jacobian(H_B1HL(1:3,end),encoders), 'Jp_Body_to_HindLeftFoot', export_path, encoders);

export_function(H_B1HR, 'H_Body_to_HindRightFoot', export_path, encoders);
export_function(H_B1HR(1:3,1:3), 'R_Body_to_HindRightFoot', export_path, encoders);
export_function(H_B1HR(1:3,end), 'p_Body_to_HindRightFoot', export_path, encoders);
export_function(jacobian(H_B1HR(1:3,end),encoders), 'Jp_Body_to_HindRightFoot', export_path, encoders);

end


