%% IAA_Theory_STS_GitHub.m
% Written by Sarah A Roelker, PhD

clc; close all; clear all;

%% Establish OpenSim Variables
% Sagittal Plane Coordinates in OpenSim IAA Model
sag_coords = {'hip_flexion_r','knee_angle_r','ankle_angle_r','hip_flexion_l','knee_angle_l','ankle_angle_l','lumbar_extension'};
sag_coords_rightonly = {'ankle_angle_r', 'knee_angle_r', 'hip_flexion_r', 'lumbar_extension'}; % right lower extremity angles + lumbar extension

% Muscles in OpenSim IAA Model
all_muscles = {'glut_med1_r' 'glut_med2_r' 'glut_med3_r' 'semimem_r' 'semiten_r' 'bifemlh_r' 'bifemsh_r' 'glut_max1_r'...
    'glut_max2_r' 'glut_max3_r' 'iliacus_r' 'psoas_r' 'rect_fem_r' 'vas_med_r' 'vas_int_r' 'vas_lat_r' 'med_gas_r' 'lat_gas_r' 'soleus_r' 'tib_ant_r'...
    'glut_med1_l' 'glut_med2_l' 'glut_med3_l' 'semimem_l' 'semiten_l' 'bifemlh_l' 'bifemsh_l' 'glut_max1_l'...
    'glut_max2_l' 'glut_max3_l' 'iliacus_l' 'psoas_l' 'rect_fem_l' 'vas_med_l' 'vas_int_l' 'vas_lat_l' 'med_gas_l' 'lat_gas_l' 'soleus_l' 'tib_ant_l'...
    'ercspn_r' 'ercspn_l' 'intobl_r' 'intobl_l' 'extobl_r' 'extobl_l'};

% Muscles Organized by Coordinate
coord_muscles.hip_flexion_r = {'glut_med1_r' 'iliacus_r' 'psoas_r' 'rect_fem_r' 'bifemlh_r' 'glut_max1_r' 'glut_max2_r' 'glut_max3_r' 'glut_med3_r' 'semimem_r' 'semiten_r'};
coord_muscles.knee_angle_r = {'bifemlh_r' 'bifemsh_r' 'lat_gas_r' 'med_gas_r' 'semimem_r' 'semiten_r' 'rect_fem_r' 'vas_int_r' 'vas_lat_r' 'vas_med_r'};
coord_muscles.ankle_angle_r = {'tib_ant_r' 'lat_gas_r' 'med_gas_r' 'soleus_r'};
coord_muscles.hip_flexion_l = {'glut_med1_l' 'iliacus_l' 'psoas_l' 'rect_fem_l' 'bifemlh_l' 'glut_max1_l' 'glut_max2_l' 'glut_max3_l' 'glut_med3_l' 'semimem_l' 'semiten_l'};
coord_muscles.knee_angle_l = {'bifemlh_l' 'bifemsh_l' 'lat_gas_l' 'med_gas_l' 'semimem_l' 'semiten_l' 'rect_fem_l' 'vas_int_l' 'vas_lat_l' 'vas_med_l'};
coord_muscles.ankle_angle_l = {'tib_ant_l' 'lat_gas_l' 'med_gas_l' 'soleus_l'};
coord_muscles.lumbar_extension = {'ercspn_l' 'ercspn_r' 'extobl_l' 'extobl_r' 'intobl_l' 'intobl_r'};

% Coordinate Moments
moments = {'pelvis_tilt', 'pelvis_list', 'pelvis_rotation', 'pelvis_tx', 'pelvis_ty', 'pelvis_tz',...
    'hip_flexion_r','hip_adduction_r','hip_rotation_r','knee_angle_r','ankle_angle_r','subtalar_angle_r','mtp_angle_r',...
    'hip_flexion_l','hip_adduction_l','hip_rotation_l','knee_angle_l','ankle_angle_l','subtalar_angle_l','mtp_angle_l', 'lumbar_extension', 'lumbar_bending', 'lumbar_rotation'};

%% Folder and File Identifiers
home = 'Insert Location Where This Code is Saved';
results_folder = 'Insert Location Where The Results Files Will Be Stored';
kinematics_files_folder = 'Insert Location Where The Default & Input Kinematics Files Are Stored';
model_file = 'IAA_Theory_Model_gait2392.osim'; % OpenSim Model

%% KINEMATIC STATES
% default = [0, 0, 0, 0]; % theta0, theta1, theta2, theta3 - Default position
% STS Phase 2 Kinematic State Ranges For IAA Model Angles
qP2_0 = 0; % theta0
qP2_1 = -40:5:0; % theta1
qP2_2 = 75:5:100; % theta2
qP2_3 = -150:5:-60; % theta3

% Create Array of All STS States 
% 'if' statements confine states for OpenSim coordinates to physiologic 
% ranges appropriate to task
n = 1;
for i = 1:length(qP2_1)
    for j = 1:length(qP2_2)
        if (qP2_1(i) + qP2_2(j)) < 180
            for k = 1:length(qP2_3)
                % if theta_H (hip flexion angle; theta0 + theta1 + theta2) 
                % is greater than -90, i.e. theta_L (lumbar angle) is 
                % greater than -90 when PTilt = 0;
                if (qP2_0 + qP2_1(i) + qP2_2(j)) >= -90 
                    STS_States(:,n) = [qP2_0; qP2_1(i); qP2_2(j); qP2_3(k)];
                    n = n+1;
                end
            end
        end
    end
end

% Range of OpenSim Model Pelvic Tilt Angles Analyzed
pelvic_tilt_angles = -60:5:20;
%% Compute Induced Potentials for Each Kinematic State
index = 1;
for x = 1:size(STS_States,2)
    % Get IAA Model Angles for current state
    theta0 = STS_States(1,x);
    theta1 = STS_States(2,x);
    theta2 = STS_States(3,x);
    theta3 = STS_States(4,x);
    
    % Each IAA Model State Analyzed For Each Pelvic Tilt Angle
    % Pelvic Tilt Angle Changes Hip Flexion & Lumbar Extension Angles of
    % the OpenSim Model
    for y = 1:length(pelvic_tilt_angles)
        theta_P = pelvic_tilt_angles(y); % Pelvic Tilt Angle
        
        theta_A = -theta1; % Ankle Dorsiflexion Angle
        theta_K = -theta2; % Knee Extension Angle
        theta_H = theta2 + theta1 + theta0 - theta_P; % Hip Flexion Angle
        theta_L = theta_H + theta3; % Lumbar Extension Angle
        
        if theta_L >= -90 && theta_L <= 90 && theta_H <= 120 % run the trial if state constraints are met
            % Name Kinematic States for Results Files
            if theta_P >= 0
                if theta1 < 0
                    kin_state = cellstr(sprintf('STS_Kinematics_%i_neg%i_%i_neg%i_PTilt%i', theta0, abs(theta1), theta2, abs(theta3), theta_P));
                else
                    kin_state = cellstr(sprintf('STS_Kinematics_%i_%i_%i_neg%i_PTilt%i', theta0, theta1, theta2, abs(theta3), theta_P));
                end
                theta_P_name = cellstr(sprintf('PTilt%i', theta_P));
            else
                if theta1 < 0
                    kin_state = cellstr(sprintf('STS_Kinematics_%i_neg%i_%i_neg%i_PTiltneg%i', theta0, abs(theta1), theta2, abs(theta3), abs(theta_P)));
                else
                    kin_state = cellstr(sprintf('STS_Kinematics_%i_%i_%i_neg%i_PTiltneg%i', theta0, theta1, theta2, abs(theta3), abs(theta_P)));
                end
                theta_P_name = cellstr(sprintf('PTiltneg%i', abs(theta_P)));
            end
            
            %% Import Default Kinematics File: Kinematics_default.txt
            cd(kinematics_files_folder)
            default_kinematics = importdata('Kinematics_default.txt');
            nRows = size(default_kinematics.data, 1);
            columnheaders = default_kinematics.colheaders;
            nCoords = length(columnheaders);
            
            % Find columns associated with each sagittal coordinate
            for c = 1:length(sag_coords)
                for n = 1:nCoords
                    if strcmp((sag_coords{c}),(columnheaders{n})) == 1
                        coord_columns.(sag_coords{c}) = n;
                    end
                end
            end
            
            %% Write New Kinematics STO File
            % Create Array of Coordinate Values for OpenSim Model State
            % This example assumes symmetry between limbs
            opensim_states = [theta_P, 0, 0, 0, 0, 0, theta_H, 0, 0, theta_K, theta_A, 0, 0, theta_H, 0, 0, theta_K, theta_A, 0, 0, theta_L, 0, 0];
            
            % Open Kinematics File to Overwrite with Current Kinematic State
            kinematics_file_name = 'STS_Kinematics.sto';
            kinematics_path = [kinematics_files_folder, kinematics_file_name];
            fid = fopen(kinematics_path,'w');
            
            % Print Header
            for i=1:8
                fprintf(fid,'%s\n', (default_kinematics.textdata{i,1}));
            end
            
            % Print Column Headers
            for m = 1:nCoords
                fprintf(fid,'%s\t',columnheaders{m});
            end
            fprintf(fid,'\n');
            
            % Print Data
            for n = 1:nRows
                for m = 1:nCoords
                    if m == 1
                        fprintf(fid,'%i\t', n);
                    else
                        fprintf(fid,'%i\t', opensim_states(1,m-1));
                    end
                end
                fprintf(fid,'\n');
            end
            fclose(fid);
            
            %% Import Model & State
            cd(home);
            % *** Matlab Must Already be Configured to Interact with 
            % OpenSim to Run This Section of the Code *** 
            import org.opensim.modeling.*; 
            
            desiredKinematicsPositions = cellstr(kinematics_path); % file location and name where desired kinematic state file created above is saved
            
            % Import OpenSim Model
            osimModel = Model([home model_file]);
            modelCoordSet = osimModel.getCoordinateSet();
            numModelCoords = modelCoordSet.getSize();
            
            % Store Desired Kinematic States
            kinematics_sto=Storage(desiredKinematicsPositions);
            
            % Prescribe Position of All Coordinates in the Model
            modelCoordSet = osimModel.getCoordinateSet();
            nCoords = modelCoordSet.getSize();
            coordinateSto=Storage(desiredKinematicsPositions);
            coords_to_prescribe = {'pelvis_tilt','pelvis_list','pelvis_rotation','pelvis_tx','pelvis_ty','pelvis_tz','hip_flexion_r','hip_adduction_r','hip_rotation_r',...
                'knee_angle_r','ankle_angle_r','subtalar_angle_r','mtp_angle_r','hip_flexion_l','hip_adduction_l','hip_rotation_l',...
                'knee_angle_l','ankle_angle_l','subtalar_angle_l','mtp_angle_l','lumbar_extension','lumbar_bending','lumbar_rotation'};
            coordset = osimModel.getCoordinateSet();
            nCoords = coordset.getSize();
            
            % Create Empty Matrix 'kinematic_vars' of Appropriate Size
            numVar = kinematics_sto.getSize();
            names = kinematics_sto.getColumnLabels();
            num_labels = names.getSize();
            kinematic_vars = zeros(numVar,num_labels);
            
            time_fit_array = ArrayDouble();
            kinematics_sto.getTimeColumn(time_fit_array);
            
            osimState = osimModel.initSystem();
            
            % Fill 'kinematic_vars' Matrix with Kinematic State Positions
            for j = 1:num_labels-1
                kin_val = ArrayDouble();
                kinematics_sto.getDataColumn(names.getitem(j),kin_val) ;
                for i=0:numVar-1
                    if (j)== 4 || (j)== 5 || (j)== 6 % pelvis x, y, z positions
                        kinematic_vars(i+1,j) = kin_val.getitem(i);
                    else % convert angles from degrees to radians
                        kinematic_vars(i+1,j) = kin_val.getitem(i)*(pi/180);
                    end
                end
            end
            
            % Update the Kinematic State
            for k = 0:numModelCoords-1
                osimState.updY().set(k,kinematic_vars(1,k+1));
            end
            
            osimModel.computeStateVariableDerivatives(osimState);
            osimModel.equilibrateMuscles(osimState);
            
            % Get Muscle Moment Arms from OpenSim Model
            % 'getCoordinateMomentArms_IAATheory' is a separate custom function
            [coordstructure,momentarms] = getCoordinateMomentArms_IAATheory(osimModel,osimState);
            ma.(theta_P_name{1}).(kin_state{1}) = cell2mat(momentarms);
            
            % Get Normalized Fiber Length of Muscles from OpenSim Model
            muscles = osimModel.getMuscles();
            for m = 0:length(all_muscles)-1
                currentmuscle = muscles.get(m);
                normFiberLength.(theta_P_name{1}).(kin_state{1}).(all_muscles{m+1}) = currentmuscle.getNormalizedFiberLength(osimState);
            end
            
            %% Create [R] Matrix - Matrix of Moment Arms
            for i = 1:length(sag_coords_rightonly)
                for n = 1:length(moments)
                    match = strcmp((sag_coords_rightonly{i}), (moments{n}));
                    if isempty(find(match,1)) == 0
                        coord_col.(sag_coords_rightonly{i}) = n;
                    end
                end
            end
            
            for i = 1:length(sag_coords_rightonly)
                for j = 1:length(all_muscles)
                    if i == 1 % theta 0
                        R_matrix.(theta_P_name{1}).(kin_state{1})(i,j) = 0;
                    elseif i == 2 || i == 3 %theta 1 and theta 2, respectively
                        R_matrix.(theta_P_name{1}).(kin_state{1})(i,j) = ma.(theta_P_name{1}).(kin_state{1})(j,coord_col.(sag_coords_rightonly{i-1}));
                    else % theta 3
                        if isempty(cell2mat(strfind(coord_muscles.hip_flexion_r,(all_muscles{j})))) ~= 1
                            R_matrix.(theta_P_name{1}).(kin_state{1})(i,j) = ma.(theta_P_name{1}).(kin_state{1})(j,coord_col.hip_flexion_r);
                        elseif isempty(cell2mat(strfind(coord_muscles.lumbar_extension,(all_muscles{j})))) ~= 1
                            R_matrix.(theta_P_name{1}).(kin_state{1})(i,j) = ma.(theta_P_name{1}).(kin_state{1})(j,coord_col.lumbar_extension);
                        else
                            R_matrix.(theta_P_name{1}).(kin_state{1})(i,j) = 0;
                        end
                    end
                end
            end
            
            %% Setup to Run IAA - Numerically
            % Get Body Segment Masses, Moment of Inertia, Center of Mass 
            % Locations, and Segment Lengths From OpenSim Model for IAA 2D Model
            all_bodies = {'ground' 'pelvis' 'femur_r' 'tibia_r' 'talus_r' 'calcn_r' 'toes_r' 'femur_l' 'tibia_l' 'talus_l' 'calcn_l' 'toes_l' 'torso'};
            numBodies = length(all_bodies);
            [tree, RootName, DOMnode] = xml_readOSIM([home model_file]);
            
            for k = 1:numBodies
                mass.(all_bodies{k}) = tree.Model.BodySet.objects.Body(k).mass;
                Izz.(all_bodies{k}) = tree.Model.BodySet.objects.Body(k).inertia_zz;
            end
            
            mass_center.pelvis = tree.Model.BodySet.objects.Body(2).mass_center;
            
            % Get Marker Locations from OpenSim Model
            for m = 1:14
                marker_name = cellstr(tree.Model.MarkerSet.objects.Marker(m).ATTRIBUTE.name);
                marker_location.(marker_name{1}) = tree.Model.MarkerSet.objects.Marker(m).location;
            end
            
            % Links: 0 = foot, 1 = tibia, 2 = femur, 3 = HAT
            % Link Masses
            m0 = mass.talus_r + mass.calcn_r + mass.toes_r; m1 = mass.tibia_r; m2 = mass.femur_r; m3 = mass.pelvis + mass.torso;
            total_mass = m0 + m1 + m2 + m3;
            
            % Link Lengths
            Origin = [marker_location.AnkleOrigin(1), marker_location.DistalToe(2)]; % In the default position, the origin is located directly inferior of the AnkleOrigin along the vector from the Heel to DistalToe marker
            
            x_L0 = marker_location.AnkleOrigin(1) - Origin(1); y_L0 = marker_location.AnkleOrigin(2) - Origin(2);
            L0 = sqrt(x_L0^2 + y_L0^2);
            
            x_L1 = marker_location.TibiaOrigin(1) - marker_location.AnkleOrigin(1); y_L1 = marker_location.TibiaOrigin(2) - marker_location.AnkleOrigin(2);
            L1 = sqrt(x_L1^2 + y_L1^2);
            
            x_L2 = marker_location.FemurOrigin(1) - marker_location.TibiaOrigin(1); y_L2 = marker_location.FemurOrigin(2) - marker_location.TibiaOrigin(2);
            L2 = sqrt(x_L2^2 + y_L2^2);
            
            % Center of Mass of the Foot Relative to the Origin (based on trial type)
            mass.foot = mass.talus_r + mass.calcn_r + mass.toes_r;
            
            x_foot = ((mass.talus_r*marker_location.TalusCOM(1)) + (mass.calcn_r*marker_location.CalcCOM (1)) + (mass.toes_r*marker_location.ToesCOM(1))) / (mass.foot);
            y_foot = ((mass.talus_r*marker_location.TalusCOM(2)) + (mass.calcn_r*marker_location.CalcCOM (2)) + (mass.toes_r*marker_location.ToesCOM(2))) / (mass.foot);
            mass_center.foot = [x_foot, y_foot];
            
            x_r0 = mass_center.foot(1) - Origin(1);
            y_r0 = mass_center.foot(2) - Origin(2);
            a = x_r0;
            
            % Center of Mass of the Tibia Relative to the Ankle
            x_r1 = marker_location.TibiaCOM(1)-marker_location.AnkleOrigin(1); y_r1 = marker_location.TibiaCOM(2)-marker_location.AnkleOrigin(2);
            
            % Center of Mass of the Femur Relative to the Knee
            x_r2 = marker_location.FemurCOM(1)-marker_location.TibiaOrigin(1); y_r2 = marker_location.FemurCOM(2)-marker_location.TibiaOrigin(2);
            
            % Center of Mass of the HAT (Head, Arms, Torso) Relative to the Hip
            mass.HAT = mass.torso + mass.pelvis;
            x_HAT = ((mass.torso*marker_location.TorsoCOM(1)) + (mass.pelvis*marker_location.PelvisCOM(1))) / mass.HAT;
            y_HAT = ((mass.torso*marker_location.TorsoCOM(2)) + (mass.pelvis*marker_location.PelvisCOM(2))) / mass.HAT;
            mass_center.HAT = [x_HAT, y_HAT];
            
            x_r3 = mass_center.HAT(1) - marker_location.FemurOrigin(1);
            y_r3 = mass_center.HAT(2) - marker_location.FemurOrigin(2);
            
            % r - distance from link origin to link COM
            r0 = sqrt(x_r0^2 + y_r0^2);
            r1 = sqrt(x_r1^2 + y_r1^2);
            r2 = sqrt(x_r2^2 + y_r2^2);
            r3 = sqrt(x_r3^2 + y_r3^2);
            
            beta = asin(abs(y_r0)/r0)*(180/pi); % beta is the internal angle between the base of the foot (CT) and the vector from the Origin to the footCOM 
            gamma = asin(abs(y_L0)/L0)*(180/pi); % gamma is the internal angle between the base of the foot (CT) and the vector from the Origin to the AnkleOrigin (90 degrees for STS)
            phi = acos(abs(y_L0)/L0)*(180/pi); % phi is the internal angle between the vector perpendicular to the base of the foot (AO) and the vector from the Origin to the AnkleOrigin (0 degrees for STS)
            
            % Link Moments of Inertia
            % Foot - by Parallel Axis Theorem
            d_talus = sqrt((mass_center.foot(1) - marker_location.TalusCOM(1))^2 + (mass_center.foot(2) - marker_location.TalusCOM(2))^2);
            d_calcn = sqrt((mass_center.foot(1) - marker_location.CalcCOM(1))^2 + (mass_center.foot(2) - marker_location.CalcCOM(2))^2);
            d_toes = sqrt((mass_center.foot(1) - marker_location.ToesCOM(1))^2 + (mass_center.foot(2) - marker_location.ToesCOM(2))^2);
            
            I_talus = Izz.talus_r + (mass.talus_r * d_talus^2);
            I_calcn = Izz.calcn_r + (mass.calcn_r * d_calcn^2);
            I_toes = Izz.toes_r + (mass.toes_r * d_toes^2);
            
            Izz0 = I_talus + I_calcn + I_toes;
            
            % Tibia - From OpenSim Model
            Izz1 = Izz.tibia_r;
            
            % Femur - From OpenSim Model
            Izz2 = Izz.femur_r;
            
            % HAT - by Parallel Axis Theorem
            d_torso = sqrt((mass_center.HAT(1) - marker_location.TorsoCOM(1))^2 + (mass_center.HAT(2) - marker_location.TorsoCOM(2))^2);
            d_pelvis = sqrt((mass_center.HAT(1) - marker_location.PelvisCOM(1))^2 + (mass_center.HAT(2) - marker_location.PelvisCOM(2))^2);
            
            I_torso = Izz.torso + (mass.torso * d_torso^2);
            I_pelvis = Izz.pelvis + (mass.pelvis * d_pelvis^2);
            
            Izz3 = I_torso + I_pelvis;
            
            %% Run IAA - Numerically
            % M = Mass Matrix 
            % Derived from IAA_Theory_MassMatrix_4Link_Lagrange.m
%             theta1 = theta1 + phi; % Accounts for foot angle if foot is
%             not flat on the ground (foot assumed flat and fixed to ground
%             for STS)
            
            M.(theta_P_name{1}).(kin_state{1}) = [Izz0 + Izz1 + Izz2 + Izz3 + L0^2*m1 + L0^2*m2 + L0^2*m3 + L1^2*m2 + L1^2*m3 + L2^2*m3 + m0*r0^2 + m1*r1^2 + m2*r2^2 + m3*r3^2 + 2*L0*L2*m3*cosd(theta1 + theta2) + 2*L0*m2*r2*cosd(theta1 + theta2) + 2*L1*m3*r3*cosd(theta2 + theta3) + 2*L0*L1*m2*cosd(theta1) + 2*L0*L1*m3*cosd(theta1) + 2*L1*L2*m3*cosd(theta2) + 2*L0*m1*r1*cosd(theta1) + 2*L1*m2*r2*cosd(theta2) + 2*L2*m3*r3*cosd(theta3) + 2*L0*m3*r3*cosd(theta1 + theta2 + theta3), Izz1 + Izz2 + Izz3 + L1^2*m2 + L1^2*m3 + L2^2*m3 + m1*r1^2 + m2*r2^2 + m3*r3^2 + L0*L2*m3*cosd(theta1 + theta2) + L0*m2*r2*cosd(theta1 + theta2) + 2*L1*m3*r3*cosd(theta2 + theta3) + L0*L1*m2*cosd(theta1) + L0*L1*m3*cosd(theta1) + 2*L1*L2*m3*cosd(theta2) + L0*m1*r1*cosd(theta1) + 2*L1*m2*r2*cosd(theta2) + 2*L2*m3*r3*cosd(theta3) + L0*m3*r3*cosd(theta1 + theta2 + theta3), Izz2 + Izz3 + L2^2*m3 + m2*r2^2 + m3*r3^2 + L0*L2*m3*cosd(theta1 + theta2) + L0*m2*r2*cosd(theta1 + theta2) + L1*m3*r3*cosd(theta2 + theta3) + L1*L2*m3*cosd(theta2) + L1*m2*r2*cosd(theta2) + 2*L2*m3*r3*cosd(theta3) + L0*m3*r3*cosd(theta1 + theta2 + theta3), Izz3 + m3*r3^2 + L1*m3*r3*cosd(theta2 + theta3) + L2*m3*r3*cosd(theta3) + L0*m3*r3*cosd(theta1 + theta2 + theta3);
                Izz1 + Izz2 + Izz3 + L1^2*m2 + L1^2*m3 + L2^2*m3 + m1*r1^2 + m2*r2^2 + m3*r3^2 + L0*L2*m3*cosd(theta1 + theta2) + L0*m2*r2*cosd(theta1 + theta2) + 2*L1*m3*r3*cosd(theta2 + theta3) + L0*L1*m2*cosd(theta1) + L0*L1*m3*cosd(theta1) + 2*L1*L2*m3*cosd(theta2) + L0*m1*r1*cosd(theta1) + 2*L1*m2*r2*cosd(theta2) + 2*L2*m3*r3*cosd(theta3) + L0*m3*r3*cosd(theta1 + theta2 + theta3), Izz1 + Izz2 + Izz3 + L1^2*m2 + L1^2*m3 + L2^2*m3 + m1*r1^2 + m2*r2^2 + m3*r3^2 + 2*L1*m3*r3*cosd(theta2 + theta3) + 2*L1*L2*m3*cosd(theta2) + 2*L1*m2*r2*cosd(theta2) + 2*L2*m3*r3*cosd(theta3),  m3*L2^2 + 2*m3*cosd(theta3)*L2*r3 + L1*m3*cosd(theta2)*L2 + m2*r2^2 + L1*m2*cosd(theta2)*r2 + m3*r3^2 + L1*m3*cosd(theta2 + theta3)*r3 + Izz2 + Izz3, Izz3 + m3*r3^2 + L1*m3*r3*cosd(theta2 + theta3) + L2*m3*r3*cosd(theta3);
                Izz2 + Izz3 + L2^2*m3 + m2*r2^2 + m3*r3^2 + L0*L2*m3*cosd(theta1 + theta2) + L0*m2*r2*cosd(theta1 + theta2) + L1*m3*r3*cosd(theta2 + theta3) + L1*L2*m3*cosd(theta2) + L1*m2*r2*cosd(theta2) + 2*L2*m3*r3*cosd(theta3) + L0*m3*r3*cosd(theta1 + theta2 + theta3), m3*L2^2 + 2*m3*cosd(theta3)*L2*r3 + L1*m3*cosd(theta2)*L2 + m2*r2^2 + L1*m2*cosd(theta2)*r2 + m3*r3^2 + L1*m3*cosd(theta2 + theta3)*r3 + Izz2 + Izz3, m3*L2^2 + 2*m3*cosd(theta3)*L2*r3 + m2*r2^2 + m3*r3^2 + Izz2 + Izz3, m3*r3^2 + L2*m3*cosd(theta3)*r3 + Izz3;
                Izz3 + m3*r3^2 + L1*m3*r3*cosd(theta2 + theta3) + L2*m3*r3*cosd(theta3) + L0*m3*r3*cosd(theta1 + theta2 + theta3), Izz3 + m3*r3^2 + L1*m3*r3*cosd(theta2 + theta3) + L2*m3*r3*cosd(theta3), m3*r3^2 + L2*m3*cosd(theta3)*r3 + Izz3, m3*r3^2 + Izz3];
            
            % Potentials = [M]^-1 * [R] - Gives Potential of Muscle to
            % Accelerate the Coordinate
            potentials.(theta_P_name{1}).(kin_state{1}) = inv(M.(theta_P_name{1}).(kin_state{1}))*R_matrix.(theta_P_name{1}).(kin_state{1});
            
            % Convert Coordinate Potentials to X (Progression) & 
            % Y (Support) Potentials using Jacobian
            
            J_v0 = [ -r0*sind(theta0 + beta), 0, 0, 0;...
                  r0*cosd(theta0 + beta), 0, 0, 0];
            J_v1 = [ -(L0*sind(theta0 + gamma)), -(r1*sind(theta0 + gamma + theta1)), 0, 0;...
                L0*cosd(theta0 + gamma), r1*cosd(theta0 + gamma + theta1), 0, 0];
            J_v2 = [ -(L0*sind(theta0 + gamma)), -(L1*sind(theta0 + gamma + theta1)), -r2*sind(theta0 + gamma + theta1 + theta2), 0;...
                L0*cosd(theta0 + gamma), L1*cosd(theta0 + gamma + theta1), r2*cosd(theta0 + gamma + theta1 + theta2), 0];
            J_v3 = [ -(L0*sind(theta0 + gamma)),-(L1*sind(theta0 + gamma + theta1)),-L2*sind(theta0 + gamma + theta1 + theta2), -r3*sind(theta0 + gamma + theta1 + theta2 + theta3);...
                L0*cosd(theta0 + gamma), L1*cosd(theta0 + gamma + theta1), L2*cosd(theta0 + gamma + theta1 + theta2), r3*cosd(theta0 + gamma + theta1 + theta2 + theta3)];
            
            for m = 1:46 % Number of muscles in OpenSim model
%                 P = [potentials.(theta_P_name{1}).(kin_state{1})(1,m); ...
%                     potentials.(theta_P_name{1}).(kin_state{1})(1,m) + potentials.(theta_P_name{1}).(kin_state{1})(2,m); ...
%                     potentials.(theta_P_name{1}).(kin_state{1})(1,m) + potentials.(theta_P_name{1}).(kin_state{1})(2,m) + potentials.(theta_P_name{1}).(kin_state{1})(3,m);...
%                     potentials.(theta_P_name{1}).(kin_state{1})(1,m) + potentials.(theta_P_name{1}).(kin_state{1})(2,m) + potentials.(theta_P_name{1}).(kin_state{1})(3,m) + potentials.(theta_P_name{1}).(kin_state{1})(4,m)];
                
                % Potential of foot (link 0) is set to 0 for STS because it is assumed that the
                % foot is fixed to the ground (cannot be accelerated) 
                P = [0; ...
                    potentials.(theta_P_name{1}).(kin_state{1})(2,m); ...
                    potentials.(theta_P_name{1}).(kin_state{1})(2,m) + potentials.(theta_P_name{1}).(kin_state{1})(3,m);...
                    potentials.(theta_P_name{1}).(kin_state{1})(2,m) + potentials.(theta_P_name{1}).(kin_state{1})(3,m) + potentials.(theta_P_name{1}).(kin_state{1})(4,m)];
                
                % Negative sign required because muscle produces an 
                % internal torque, but we are interested in contributions 
                % to external accelerations (i.e., need to flip sign)
                XY_0 = -J_v0*P; X_0 = XY_0(1); Y_0 = XY_0(2); % X_0 = 0; Y_0 = 0;
                XY_1 = -J_v1*P; X_1 = XY_1(1); Y_1 = XY_1(2);
                XY_2 = -J_v2*P; X_2 = XY_2(1); Y_2 = XY_2(2);
                XY_3 = -J_v3*P; X_3 = XY_3(1); Y_3 = XY_3(2);
                
                X_potentials_COM.(theta_P_name{1}).(kin_state{1}).(all_muscles{m}) = ((X_0*m0/total_mass) + (X_1*m1/total_mass) + (X_2*m2/total_mass) + (X_3*m3/total_mass));
                Y_potentials_COM.(theta_P_name{1}).(kin_state{1}).(all_muscles{m}) = ((Y_0*m0/total_mass) + (Y_1*m1/total_mass) + (Y_2*m2/total_mass) + (Y_3*m3/total_mass));
            end
            
%             theta1 = round(theta1-phi); % Remove phi to save results file
%             with original theta1 (only required for tasks in which phi ~= 0)
            %% Print Results to Spreadsheet
            cd(results_folder)
            results_file_name = sprintf('IAATheoryResults_STS_%i_%i_%i_%i_PTilt%i.txt', theta0, theta1, theta2, theta3, theta_P);
            results_path = [results_folder, results_file_name];
            fid = fopen(results_path,'w+');
            
            % Print Header
            for i=1:8
                fprintf(fid,'%s\n', (default_kinematics.textdata{i,1}));
            end
            
            % Print Column Headers
            fprintf(fid,'Muscle\t');
            fprintf(fid,'Theta_P\t');
            fprintf(fid,'Theta_0\t');
            fprintf(fid,'Theta_1\t');
            fprintf(fid,'Theta_2\t');
            fprintf(fid,'Theta_3\t');
            fprintf(fid,'X_Potentials\t');
            fprintf(fid,'Y_Potentials\t');
            fprintf(fid,'Moment_Arm_Theta_0\t');
            fprintf(fid,'Moment_Arm_Theta_1\t');
            fprintf(fid,'Moment_Arm_Theta_2\t');
            fprintf(fid,'Moment_Arm_Theta_3\t');
            fprintf(fid,'Normalized_FiberLength\t');
            fprintf(fid,'\n');
            
            % Print Data
            for m = 1:length(all_muscles)
                fprintf(fid,'%s\t',all_muscles{m});
                fprintf(fid,'%f\t',theta_P);
                fprintf(fid,'%f\t',theta0);
                fprintf(fid,'%f\t',theta1);
                fprintf(fid,'%f\t',theta2);
                fprintf(fid,'%f\t',theta3);
                fprintf(fid,'%f\t',X_potentials_COM.(theta_P_name{1}).(kin_state{1}).(all_muscles{m}));
                fprintf(fid,'%f\t',Y_potentials_COM.(theta_P_name{1}).(kin_state{1}).(all_muscles{m}));
                fprintf(fid,'%f\t',R_matrix.(theta_P_name{1}).(kin_state{1})(1,m));
                fprintf(fid,'%f\t',R_matrix.(theta_P_name{1}).(kin_state{1})(2,m));
                fprintf(fid,'%f\t',R_matrix.(theta_P_name{1}).(kin_state{1})(3,m));
                fprintf(fid,'%f\t',R_matrix.(theta_P_name{1}).(kin_state{1})(4,m));
                fprintf(fid,'%f\t',normFiberLength.(theta_P_name{1}).(kin_state{1}).(all_muscles{m}));
                fprintf(fid,'\n');
            end
            fclose(fid);
            
            sprintf('Pelvic Tilt %i Trial %i of %i complete', theta_P, index, size(STS_States,2))
        end
    end
    index = index + 1;
end

%% Save Results to .mat File
cd(home)
save('IAA_Theory_STS_Results.mat', 'M', 'potentials', 'R_matrix', 'all_muscles',...
    'X_potentials_COM', 'Y_potentials_COM', 'normFiberLength', 'ma', 'STS_States','STS_kin_state');