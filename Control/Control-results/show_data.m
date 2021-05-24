clear all;
close all;
clc;

%path for the csv data file
D = readtable('Attitude and Altitude Control-202.csv');


Time = (D.Var1 - D.Var1(1));
t=Time;

dTave = 0;
for i=1:1:length(Time)-1
    dTave = dTave + (Time(i+1)-Time(i));
end
dtave = dTave/length(Time);


measured_phi = D.Var2;
measured_theta = D.Var3;
measured_psi = D.Var4;
des_phi = D.Var5;
des_theta = D.Var6;
des_psi = D.Var7;
wb_1 = D.Var8;
wb_2 = D.Var9;
wb_3 = D.Var10;

[d_phi,d_theta,d_psi] = calculateDEuler(wb_1,wb_2,wb_3,measured_phi,measured_theta);

des_d_phi = D.Var11; 
des_d_theta = D.Var12;
des_d_psi = D.Var13;
d_r_e1 = D.Var14;
d_r_e2 = D.Var15;
d_r_e3 = D.Var16;
uz = D.Var17;   %%
servo_angle1 = D.Var18;  %%
servo_angle2 = D.Var19; %%
servo_angle3 = D.Var20; %%
servo_angle4 = D.Var21; %%
vState = D.Var22; %%
e1_pos = D.Var23;
e2_pos = D.Var24;
e3_pos = D.Var25;
vicon_phi = D.Var26; %%
vicon_theta = D.Var27; %%
vicon_psi = D.Var28; %%
des_e1 = D.Var29;
des_e2 = D.Var30;
des_e3 = D.Var31;
pwmApplied = D.Var32; %%
lambda_phi = D.Var33;
lambda_theta = D.Var34;
lambda_psi = D.Var35;
A_phi = D.Var36;
A_theta = D.Var37;
A_psi = D.Var38;
B_phi = D.Var39;
B_theta = D.Var40;
B_psi = D.Var41;
eps_phi = D.Var42;
eps_theta = D.Var43;
eps_psi = D.Var44;
lambda_e1 = D.Var45;
lambda_e2 = D.Var46;
lambda_e3 = D.Var47;
A_e1 = D.Var48;
A_e2 = D.Var49;
A_e3 = D.Var50;
B_e1 = D.Var51;
B_e2 = D.Var52;
B_e3 = D.Var53;
eps_e1 = D.Var54;
eps_e2 = D.Var55;
eps_e3 = D.Var56;

figure(1);
plot(Time,uz/47.93);
grid on;
hold on;
plot(Time,-des_e3)
plot(Time,-e3_pos)

figure(2);
subplot(4,3,1)
plot(Time,lambda_phi);
title("Lambda phi")
grid on;

subplot(4,3,2)
plot(Time,lambda_theta);
title("Lambda theta")
grid on;

subplot(4,3,3)
plot(Time,lambda_psi);
title("Lambda psi")
grid on;

subplot(4,3,4)
plot(Time,A_phi);
title("A phi")
grid on;

subplot(4,3,5)
plot(Time,A_theta);
title("A theta")
grid on;

subplot(4,3,6)
plot(Time,A_psi);
title("A psi");
grid on;

subplot(4,3,7)
plot(Time,B_phi);
title("B phi");
grid on;

subplot(4,3,8)
plot(Time,B_theta);
title("B theta");
grid on;

subplot(4,3,9)
plot(Time,B_psi);
title("B psi");
grid on;

subplot(4,3,10)
plot(Time,eps_phi);
title("eps phi");
grid on;

subplot(4,3,11)
plot(Time,eps_theta);
title("eps theta");
grid on;

subplot(4,3,12)
plot(Time,eps_psi);
title("eps psi");
grid on;

sgtitle("Attitude Controller Gains");




figure(3);
subplot(4,3,1)
plot(Time,lambda_e1);
title("Lambda e1")
grid on;

subplot(4,3,2)
plot(Time,lambda_e2);
title("Lambda e2")
grid on;

subplot(4,3,3)
plot(Time,lambda_e3);
title("Lambda e3")
grid on;

subplot(4,3,4)
plot(Time,A_e1);
title("A e1")
grid on;

subplot(4,3,5)
plot(Time,A_e2);
title("A e2")
grid on;

subplot(4,3,6)
plot(Time,A_e3);
title("A e3");
grid on;

subplot(4,3,7)
plot(Time,B_e1);
title("B e1");
grid on;

subplot(4,3,8)
plot(Time,B_e2);
title("B e2");
grid on;

subplot(4,3,9)
plot(Time,B_e3);
title("B e3");
grid on;

subplot(4,3,10)
plot(Time,eps_e1);
title("eps e1");
grid on;

subplot(4,3,11)
plot(Time,eps_e2);
title("eps e2");
grid on;

subplot(4,3,12)
plot(Time,eps_e3);
title("eps e3");
grid on;

sgtitle("Position Controller Gains");

figure(11)
subplot(4,3,1)
plot(Time,rad2deg(measured_phi));
hold on;
plot(Time,des_phi)
legend('Meas','Ref');
title("Phi");
grid on;

subplot(4,3,2)
plot(Time,rad2deg(measured_theta));
hold on;
plot(Time,des_theta)
legend('Meas','Ref');
title("Theta");
grid on;

subplot(4,3,3)
plot(Time,rad2deg(measured_psi));
hold on;
plot(Time,des_psi)
legend('Meas','Ref');
title("Psi");
grid on;

subplot(4,3,4)
plot(Time,rad2deg(d_phi));
hold on;
plot(Time,des_d_phi)
legend('Meas','Ref');
title("d Phi");
grid on;

subplot(4,3,5)
plot(Time,rad2deg(d_theta));
hold on;
plot(Time,des_d_theta)
legend('Meas','Ref');
title("d Theta");
grid on;

subplot(4,3,6)
plot(Time,rad2deg(d_psi));
hold on;
plot(Time,des_d_psi)
legend('Meas','Ref');
title("d Psi");
grid on;

subplot(4,3,7)
plot(Time,e1_pos);
hold on;
plot(Time,des_e1)
legend('Meas','Ref');
title("e1");
grid on;

subplot(4,3,8)
plot(Time,e2_pos);
hold on;
plot(Time,des_e2)
legend('Meas','Ref');
title("e2");
grid on;

subplot(4,3,9)
plot(Time,-e3_pos);
hold on;
plot(Time,-des_e3)
legend('Meas','Ref');
title("e3");
grid on;

subplot(4,3,10)
plot(Time,d_r_e1);
legend('Meas');
title("d e1");
grid on;

subplot(4,3,11)
plot(Time,d_r_e2);
legend('Meas');
title("d e2");
grid on;

subplot(4,3,12)
plot(Time,-d_r_e3);
legend('Meas');
title("d e3");
sgtitle("Vehicle States");
grid on;

figure(5);
subplot(2,4,1);
plot(Time,pwmApplied);
title("User PWM")
grid on;

subplot(2,4,2);
plot(Time,uz);
title("Controller Thrust");
grid on;

subplot(2,4,3);
plot(Time,uz/47.93);
hold on;
grid on;
plot(Time,-e3_pos);
plot(Time,-des_e3)
title("Controller Thrust and e3");
legend("pwm","e3","des e3");

subplot(2,4,4);
plot(Time,vState);
title("Vehicle State");
grid on;

subplot(2,4,5);
plot(Time,rad2deg(servo_angle1));
title("Deflector 1");
ylabel("deg");
grid on;

subplot(2,4,6);
plot(Time,rad2deg(servo_angle2));
title("Deflector 2");
ylabel("deg");
grid on;

subplot(2,4,7);
plot(Time,rad2deg(servo_angle3));
title("Deflector 3");
ylabel("deg");
grid on;

subplot(2,4,8);
plot(Time,rad2deg(servo_angle4));
title("Deflector 4");
ylabel("deg");
grid on;

sgtitle("Control Inputs");
figure(4)
subplot(2,3,1)
hold on;
plot(Time,rad2deg(measured_phi));
plot(Time,des_phi)
ylim([-12 12])
xlim([0 46])
%%red
sq(40,46,245,133,125)
%%green
sq(0,40,159,247,141)
plot(Time,rad2deg(measured_phi),'color','#0072BD','Linewidth',3);
plot(Time,des_phi,'color','#A2142F','Linewidth',3)
hold off
legend('Meas','Ref');
title("Phi [rad]");
grid on;

subplot(2,3,2)
hold on;
plot(Time,rad2deg(measured_theta));
plot(Time,des_theta)
ylim([-12 12])
xlim([0 46])
%red
sq(40,46,245,133,125)
%green
sq(0,40,159,247,141)
plot(Time,rad2deg(measured_theta),'color','#0072BD','Linewidth',3);
plot(Time,des_theta,'color','#A2142F','Linewidth',3)
hold off
legend('Meas','Ref');
title("Theta [rad]");
grid on;

subplot(2,3,3)
hold on;
plot(Time,rad2deg(measured_psi));
plot(Time,des_psi)
ylim([-10 80])
xlim([0 46])
%red
sq(40,46,245,133,125)
%green
sq(0,40,159,247,141)
plot(Time,rad2deg(measured_psi),'color','#0072BD','Linewidth',3);
plot(Time,des_psi,'color','#A2142F','Linewidth',3)
hold off
legend('Meas','Ref');
title("Psi [rad]");
grid on;

subplot(2,3,4)
hold on;
plot(Time,rad2deg(d_phi));
plot(Time,des_d_phi)
ylim([-20 60])
xlim([0 46])
%red
sq(40,46,245,133,125)
%green
sq(0,40,159,247,141)
plot(Time,rad2deg(d_phi),'color','#0072BD','Linewidth',3);
plot(Time,des_d_phi,'color','#A2142F','Linewidth',3)
hold off
legend('Meas','Ref');
title("d Phi [rad/s]");
grid on;

subplot(2,3,5)
hold on;
plot(Time,rad2deg(d_theta));
plot(Time,des_d_theta)
ylim([-20 60])
xlim([0 46])
%red
sq(40,46,245,133,125)
%green
sq(0,40,159,247,141)
plot(Time,rad2deg(d_theta),'color','#0072BD','Linewidth',3);
plot(Time,des_d_theta,'color','#A2142F','Linewidth',3)
hold off
legend('Meas','Ref');
title("d Theta [rad/s]");
grid on;

subplot(2,3,6)
hold on;
plot(Time,rad2deg(d_psi));
plot(Time,des_d_psi)
ylim([-20 60])
xlim([0 46])
%red
sq(40,46,245,133,125)
%green
sq(0,40,159,247,141)
plot(Time,rad2deg(d_psi),'color','#0072BD','Linewidth',3);
plot(Time,des_d_psi,'color','#A2142F','Linewidth',3)
hold off
legend('Meas','Ref');
title("d Psi [rad/s]");
grid on;





figure(10);

subplot(2,1,1)
hold on
plot(Time,measured_psi);
plot(Time,des_psi)
ylim([-pi pi])
%red
sq(10,12.5,245,133,125)
sq(16,17.4,245,133,125)
%green
sq(12.5,16,159,247,141)
sq(17.4,22,159,247,141)
plot(Time,measured_psi,'color','#0072BD','Linewidth',3);
plot(Time,des_psi,'color','#A2142F','Linewidth',3)
xlim([10 21])
hold off
yticks([-pi -2.5 -2.0 -1.5 -1.0 -0.5 0  0.5 1.0 1.5 2.0 2.5 pi])
yticklabels({'-pi','-2.5','-2.0','-1.5','-1.0','-0.5','0.0','0.5','1.0','1.5','2.0','2.5','pi'})
 a = get(gca,'XTickLabel');  
 set(gca,'XTickLabel',a,'fontsize',12,'FontWeight','bold')
lgd=legend('Meas','Ref');
lgd.FontSize = 14;
title("Psi [rad]");
grid on;


subplot(2,1,2)
hold on;
plot(Time,d_psi);
plot(Time,des_d_psi)
ylim([-pi pi])
patch([12.5 10 10 12.5], [max(ylim) max(ylim) -max(ylim) -max(ylim)],[245/255,133/255,125/255])
patch([17.4 16 16 17.4], [max(ylim) max(ylim) -max(ylim) -max(ylim)],[245/255,133/255,125/255])
patch([16 12.5 12.5 16], [max(ylim) max(ylim) -max(ylim) -max(ylim)],[159/255,247/255,141/255])
patch([22 17.4 17.4 22], [max(ylim) max(ylim) -max(ylim) -max(ylim)],[159/255,247/255,141/255])
plot(Time,d_psi,'color','#0072BD','Linewidth',3);
plot(Time,des_d_psi,'color','#A2142F','Linewidth',3)
xlim([10 21])
hold off;
yticks([-pi -2.5 -2.0 -1.5 -1.0 -0.5 0  0.5 1.0 1.5 2.0 2.5 pi])
yticklabels({'-pi','-2.5','-2.0','-1.5','-1.0','-0.5','0.0','0.5','1.0','1.5','2.0','2.5','pi'})
 a = get(gca,'XTickLabel');  
 set(gca,'XTickLabel',a,'fontsize',12,'FontWeight','bold')
lgd=legend('Meas','Ref');
lgd.FontSize = 14;
title("d Psi [rad/s]");
grid on;


function sq(x,y,r,g,b)
patch([y x x y], [max(ylim) max(ylim) -max(ylim) -max(ylim)],[r/255,g/255,b/255])
end


function [d_phi,d_theta,d_psi] = calculateDEuler(w1,w2,w3,phi,theta)
    d_phi = w1;
    d_theta = w2;
    d_psi = w3;
    
    for i=1:1:length(w1)
        d_phi(i) = w1(i) + w2(i) * sin(phi(i)) * tan(theta(i)) + w3(i) * cos(phi(i)) * tan(theta(i));
        d_theta(i) = w2(i) * cos(phi(i)) - w3(i) * sin(phi(i));
        d_psi(i) = (w2(i) * sin(phi(i))) / cos(theta(i)) + (w3(i) * cos(phi(i))) / cos(theta(i));

    end
end









