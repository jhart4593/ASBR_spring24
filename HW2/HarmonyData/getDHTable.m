function dh_table = getDHTable(ConfigVals,sizes,sideChar)

deg2rad = pi/180;
if strcmpi(sideChar,'L')
    DH_Data = ConfigVals.DH_Parameters.Left; 
elseif strcmpi(sideChar,'R')
    DH_Data = ConfigVals.DH_Parameters.Right; 
end

alpha = DH_Data.Values{strcmp(DH_Data.ParamNames,'alpha')};
alpha = alpha*deg2rad; 

offset = DH_Data.Values{strcmp(DH_Data.ParamNames,'offset')};
offset = offset*deg2rad;

h_se = sizes(1)*.001;
clavAct = sizes(2)*.001;
lh = sizes(3)*.001; 
lf = sizes(4)*.001;
lp = convert_shoulder_breadth_to_clavicle_length_m(clavAct); % clavicle linkage length
w_se = clavAct-lp; 

a = [0, w_se, 0, lp, 0, 0, 0, lh, 0, 0];
d = [0, h_se, 0, 0, 0, 0, 0, 0, 0, lf];

dh_table = [offset' d' a' alpha'];

end