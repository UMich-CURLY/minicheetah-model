% Last edited in 01.06 2020
classdef MC_model < RobotLinks
    
    properties
        ContactPoints = struct;
        OtherPoints = struct;

    end
    
    methods
        
        function obj = MC_model(urdf)
            
            % Floating base model            
            base(6) = struct(); % 6-DOF base coordinates
            
            % the name of the base dofs 
            [base(1:6).Name] = deal('BasePosX','BasePosY','BasePosZ','BaseRotZ','BaseRotY','BaseRotX'); 
            
            % the type of the base dofs
            [base(1:3).Type] = deal('prismatic'); % the first three are prismatic joints
            [base(4:6).Type] = deal('revolute');  % the last three are revolute joints
            
            % the origin are all zeros
            [base.Offset] = deal([0,0,0]);
            [base.R] = deal([0,0,0]);
            
            % the axis of the base dofs
            [base(1:3).Axis] = deal([1,0,0],[0,1,0],[0,0,1]);
            [base(4:6).Axis] = deal([0,0,1],[0,1,0],[1,0,0]);
            
            % the parent link of the base dofs
            [base.Parent] =  deal('Origin', 'BasePosX','BasePosY','BasePosZ','BaseRotZ','BaseRotY'); 
            
            % the child link of the base dofs
            [base.Child] = deal('BasePosX','BasePosY','BasePosZ','BaseRotZ','BaseRotY','');

            % Set base DOF limits (Joint limits are read from URDF)
            [limit(1:6).effort] = deal(0);
            [limit(1:6).lower] = deal(-inf, -inf, -inf, -pi, -pi, -pi);
            [limit(1:6).upper] = deal(inf, inf, inf, pi, pi, pi);
            [limit(1:6).velocity] = deal(inf);
            
            for i=1:length(base)
                base(i).Limit = limit(i);
            end
            
            
            % load model from the URDF file
            config = struct();
            config_file = GetFullPath(urdf);
            [config.name, config.links, config.joints, config.transmissions] = ros_load_urdf(config_file);
            obj = obj@RobotLinks(config, base, [], 'removeFixedJoints', true);
            obj.ConfigFile = config_file;
                        
            %% define contact frames
            Rz = @(th) [cos(th), -sin(th), 0; sin(th), cos(th), 0; 0,0,1];
            Ry = @(th) [cos(th), 0, sin(th); 0, 1, 0; -sin(th), 0, cos(th)];
            Rx = @(th) [1,0,0; 0, cos(th), -sin(th); 0, sin(th), cos(th)];
                
            % front left foot
            fl_knee_frame = obj.Joints(getJointIndices(obj, 'thigh_fl_to_knee_fl_j'));
            H = double(subs(fl_knee_frame.computeForwardKinematics, obj.States.x, zeros(18,1)));
%             R = H(1:3,1:3)'*Rz(-pi/2)*Rx(deg2rad(50));
%             R = Rz(deg2rad(140))*Rx(-pi/2);
            R = eye(3);

            obj.ContactPoints.FrontLeftFoot = CoordinateFrame(...
                'Name','FrontLeftFoot',...
                'Reference',fl_knee_frame,...
                'Offset',[0, 0, -0.2075],...    % 0.19(shank length) + 0.0175(ball radius)
                'R',R... 
                );
            
            % front right foot
            fr_knee_frame = obj.Joints(getJointIndices(obj, 'thigh_fr_to_knee_fr_j'));
            H = double(subs(fr_knee_frame.computeForwardKinematics, obj.States.x, zeros(18,1)));
%             R = H(1:3,1:3)'*Rz(-pi/2)*Rx(deg2rad(50));
%             R = Rz(deg2rad(140))*Rx(-pi/2);
            R = eye(3);

            obj.ContactPoints.FrontRightFoot = CoordinateFrame(...
                'Name','FrontRightFoot',...
                'Reference',fr_knee_frame,...
                'Offset',[0, 0, -0.2075],...
                'R',R... 
                );
            % hind left foot
            hl_knee_frame = obj.Joints(getJointIndices(obj, 'thigh_hl_to_knee_hl_j'));
            H = double(subs(hl_knee_frame.computeForwardKinematics, obj.States.x, zeros(18,1)));
%             R = H(1:3,1:3)'*Rz(-pi/2)*Rx(deg2rad(50));
%             R = Rz(deg2rad(140))*Rx(-pi/2);
            R = eye(3);

            obj.ContactPoints.HindLeftFoot = CoordinateFrame(...
                'Name','HindLeftFoot',...
                'Reference',hl_knee_frame,...
                'Offset',[0, 0, -0.2075],...
                'R',R... 
                );
            % hind right foot
            hr_knee_frame = obj.Joints(getJointIndices(obj, 'thigh_hr_to_knee_hr_j'));
            H = double(subs(hr_knee_frame.computeForwardKinematics, obj.States.x, zeros(18,1)));
%             R = H(1:3,1:3)'*Rz(-pi/2)*Rx(deg2rad(50));
%             R = Rz(deg2rad(140))*Rx(-pi/2);
            R = eye(3);

            obj.ContactPoints.HindRightFoot = CoordinateFrame(...
                'Name','HindRightFoot',...
                'Reference',hr_knee_frame,...
                'Offset',[0, 0, -0.2075],...
                'R',R... 
                );
            %% Define other frames
%             IMU_frame = obj.Joints(getJointIndices(obj, 'BaseRotX'));        
        end
        
        function ExportKinematics(obj, export_function, export_path)
            % Generates code for forward kinematics
            
            if ~exist(export_path,'dir')
                mkdir(export_path);
                addpath(export_path);
            end
            
            % Compute positions of all joints
            for i = 1:length(obj.Joints)
                % Create fixed frame on link after rotation of joint
                % (same origin as joint)
                frame = CoordinateFrame(...
                    'Name',obj.Joints(i).Name,...
                    'Reference',obj.Joints(i),...
                    'Offset',[0, 0, 0],...
                    'R',[0, 0, 0]);
                H = frame.computeForwardKinematics;
                p = H(1:3,end);
                J = jacobian(p, obj.States.x);
                R = H(1:3,1:3);
                export_function(p, ['p_', obj.Joints(i).Name], export_path, obj.States.x);
                export_function(J, ['Jp_', obj.Joints(i).Name], export_path, obj.States.x);
                export_function(H, ['H_', obj.Joints(i).Name], export_path, obj.States.x);
                export_function(R, ['R_', obj.Joints(i).Name], export_path, obj.States.x);
            end
            
            % Compute positions of contact points
            cp_fields = fields(obj.ContactPoints);
            for i = 1:length(cp_fields)
                p = obj.ContactPoints.(cp_fields{i}).computeCartesianPosition;
                J = jacobian(p, obj.States.x);
                H = obj.ContactPoints.(cp_fields{i}).computeForwardKinematics;
                R = H(1:3,1:3);
                export_function(p, ['p_', obj.ContactPoints.(cp_fields{i}).Name], export_path, obj.States.x);
                export_function(J, ['Jp_', obj.ContactPoints.(cp_fields{i}).Name], export_path, obj.States.x);
                export_function(H, ['H_', obj.ContactPoints.(cp_fields{i}).Name], export_path, obj.States.x);
                export_function(R, ['R_', obj.ContactPoints.(cp_fields{i}).Name], export_path, obj.States.x);
            end
            
            % Compute positions of other points
            op_fields = fields(obj.OtherPoints);
            for i = 1:length(op_fields)
                p = obj.OtherPoints.(op_fields{i}).computeCartesianPosition;
                J = jacobian(p, obj.States.x);
                H = obj.OtherPoints.(op_fields{i}).computeForwardKinematics;
                R = H(1:3,1:3);
                export_function(p, ['p_', obj.OtherPoints.(op_fields{i}).Name], export_path, obj.States.x);
                export_function(J, ['Jp_', obj.OtherPoints.(op_fields{i}).Name], export_path, obj.States.x);
                export_function(H, ['H_', obj.OtherPoints.(op_fields{i}).Name], export_path, obj.States.x);
                export_function(R, ['R_', obj.OtherPoints.(op_fields{i}).Name], export_path, obj.States.x);
            end
            
        end
    end
    
    
end