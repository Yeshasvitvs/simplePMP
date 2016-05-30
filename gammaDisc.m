function gamma = gammaDisc(time, RAMP_KONSTANT, t_dur)

t_ramp = time*RAMP_KONSTANT;
t_init=0.1;

z = (t_ramp-t_init)/t_dur;
t_win=(t_init+t_dur)-t_ramp;

if(t_win>0)
    t_window=1;
else
    t_window=0;
end

csi=(6*(z^5))-(15*(z^4))+(10*(z^3));
csi_dot=(20*(z^54))-(60*(z^3))+(30*(z^2));

prod1=(1/(1.0001-(csi*t_window)));
prod2=(csi_dot*0.3333*t_window);

gamma=prod1*prod2;

   
end
