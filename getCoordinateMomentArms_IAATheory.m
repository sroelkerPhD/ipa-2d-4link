function [coordstructure,momentarms] = getCoordinateMomentArms_IAATheory(osimModel,osimState)

modelCoordSet = osimModel.getCoordinateSet();
numModelCoords = modelCoordSet.getSize();
muscles = osimModel.getMuscles();
numMuscles = muscles.getSize();
coordstructure = {};

all_muscles = {'glut_med1_r' 'glut_med2_r' 'glut_med3_r' 'semimem_r' 'semiten_r' 'bifemlh_r' 'bifemsh_r' 'glut_max1_r'...
 'glut_max2_r' 'glut_max3_r' 'iliacus_r' 'psoas_r' 'rect_fem_r' 'vas_med_r' 'vas_int_r' 'vas_lat_r' 'med_gas_r' 'lat_gas_r' 'soleus_r' 'tib_ant_r'...
 'glut_med1_l' 'glut_med2_l' 'glut_med3_l' 'semimem_l' 'semiten_l' 'bifemlh_l' 'bifemsh_l' 'glut_max1_l'...
 'glut_max2_l' 'glut_max3_l' 'iliacus_l' 'psoas_l' 'rect_fem_l' 'vas_med_l' 'vas_int_l' 'vas_lat_l' 'med_gas_l' 'lat_gas_l' 'soleus_l' 'tib_ant_l'...
 'ercspn_r' 'ercspn_l' 'intobl_r' 'intobl_l' 'extobl_r' 'extobl_l'};

moments = {'pelvis_tilt', 'pelvis_list', 'pelvis_rotation', 'pelvis_tx', 'pelvis_ty', 'pelvis_tz',...
    'hip_flexion_r','hip_adduction_r','hip_rotation_r','knee_angle_r','ankle_angle_r','subtalar_angle_r','mtp_angle_r',...
    'hip_flexion_l','hip_adduction_l','hip_rotation_l','knee_angle_l','ankle_angle_l','subtalar_angle_l','mtp_angle_l', 'lumbar_extension', 'lumbar_bending', 'lumbar_rotation'};

coord_muscles.hip_flexion_r = {'glut_med1_r' 'iliacus_r' 'psoas_r' 'rect_fem_r' 'bifemlh_r' 'glut_max1_r' 'glut_max2_r' 'glut_max3_r' 'glut_med3_r' 'semimem_r' 'semiten_r'};
coord_muscles.knee_angle_r = {'bifemlh_r' 'bifemsh_r' 'lat_gas_r' 'med_gas_r' 'semimem_r' 'semiten_r' 'rect_fem_r' 'vas_int_r' 'vas_lat_r' 'vas_med_r'};
coord_muscles.ankle_angle_r = {'tib_ant_r' 'lat_gas_r' 'med_gas_r' 'soleus_r'};
coord_muscles.hip_flexion_l = {'glut_med1_l' 'iliacus_l' 'psoas_l' 'rect_fem_l' 'bifemlh_l' 'glut_max1_l' 'glut_max2_l' 'glut_max3_l' 'glut_med3_l' 'semimem_l' 'semiten_l'};
coord_muscles.knee_angle_l = {'bifemlh_l' 'bifemsh_l' 'lat_gas_l' 'med_gas_l' 'semimem_l' 'semiten_l' 'rect_fem_l' 'vas_int_l' 'vas_lat_l' 'vas_med_l'};
coord_muscles.ankle_angle_l = {'tib_ant_l' 'lat_gas_l' 'med_gas_l' 'soleus_l'};
coord_muscles.lumbar_extension = {'ercspn_l' 'ercspn_r' 'extobl_l' 'extobl_r' 'intobl_l' 'intobl_r'};
coord_muscles.pelvis_tilt = {};
coord_muscles.pelvis_list = {};
coord_muscles.pelvis_rotation = {};
coord_muscles.pelvis_tx = {};
coord_muscles.pelvis_ty = {};
coord_muscles.pelvis_tz = {};
coord_muscles.subtalar_angle_r = {};
coord_muscles.subtalar_angle_l = {};
coord_muscles.mtp_angle_r = {};
coord_muscles.mtp_angle_l = {};
coord_muscles.hip_adduction_r = {};
coord_muscles.hip_rotation_r = {};
coord_muscles.hip_adduction_l = {};
coord_muscles.hip_rotation_l = {};
coord_muscles.lumbar_bending = {};
coord_muscles.lumbar_rotation = {};

for j = 0:numModelCoords-1
    currentcoord = modelCoordSet.get(j);
    coord_j = moments(j+1);
    for i = 0:numMuscles-1
        currentmuscle  = muscles.get(i);
        muscle_i = all_muscles(i+1);
        match = strcmp(muscle_i, coord_muscles.(coord_j{1}));
        if isempty(find(match,1)) == 0
            ma = currentmuscle.computeMomentArm(osimState,currentcoord);
            
            coordstructure{(i+1),(j+1)} = currentmuscle.getName();
            momentarms{(i+1),(j+1)} = ma;
        else
            coordstructure{(i+1),(j+1)} = currentmuscle.getName();
            momentarms{(i+1),(j+1)} = 0;
        end
    end    
end

          
          

