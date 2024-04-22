Gamma_des_x_i = 0
Gamma_des_y_i = 0
alt_des_vel_i = 0
Gamma_des_x_list_num = 11
Gamma_des_y_list_num = 11
alt_des_vel_list_num = 11

ep_i = 0

while True:
    Gamma_des_x_i = Gamma_des_x_i + 1
    ep_i = ep_i + 1
    
    if Gamma_des_x_i >= Gamma_des_x_list_num:
        Gamma_des_x_i = 0
        Gamma_des_y_i = Gamma_des_y_i + 1
    
    if Gamma_des_y_i >= Gamma_des_y_list_num:
        Gamma_des_y_i = 0
        alt_des_vel_i = alt_des_vel_i + 1
    
    print(ep_i, Gamma_des_x_i,Gamma_des_y_i,alt_des_vel_i)
    
    if ep_i >= Gamma_des_x_list_num * Gamma_des_y_list_num * alt_des_vel_list_num:
        break;