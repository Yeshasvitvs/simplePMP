clear all; clc;

RAMP = 0.005;
dur = 5;
ITERATION=1000;
Gam_Arrx=zeros(ITERATION,1);
Gam_Arry=zeros(ITERATION,1);
Gam_Arrz=zeros(ITERATION,1);

ix=0; 
iix=ix;
fx=100;
vx=0;

h1=figure(1);

for time=0:1:ITERATION

    gamma = gammaDisc(time, RAMP, dur);
    vx = (fx-ix)*gamma;
    Gam_Arrx(time+1,1)=vx;
    sum = gammaInt(Gam_Arrx,time,RAMP);
    ix= sum+ iix;
    
    plot(time,gamma,'.r');
    hold on;
    
    plot(time,sum,'.b');
    hold on;
    
     plot(time,Gam_Arrx(time+1,1),'.g');
    hold on;
    
end


    
   
    