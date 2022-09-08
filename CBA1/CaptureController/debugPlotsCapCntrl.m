% To be used with plant block.slx and the relevant workspace materials

figure(1);
x1 = data{4}.Values;
x2 = data{1}.Values;
x3  = data{3}.Values;
x4 = data{2}.Values;

capCntrl = data{5}.Values;

figure(1)
plot(x1.Time, x1.Data);
grid on
title("Flywheel Angle <rad>")
xlabel("Time <s>")
ylabel("Angle <RPM>")

figure(2)
plot(x2.Time, x2.Data);
grid on
title("x2 (Gimbal) <rad>")
xlabel("Time <s>")
ylabel("Angle <RPM>")

figure(3)
plot(x3.Time, x3.Data);
grid on
title("x3 (Table) <rad>")
xlabel("Time <s>")
ylabel("Angle <rad>")

figure(4)
plot(x4.Time, x4.Data);
grid on
title("x4 (Pendulum) <rad>")
xlabel("Time <s>")
ylabel("Angle <rad>")

figure(5)
plot(capCntrl.Time, capCntrl.Data);
grid on
title("Capture Controller Output")
xlabel("Time <s>")
ylabel("Voltage <V>")
