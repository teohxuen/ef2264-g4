clc; clear;
m=readmatrix('measurement.txt','NumHeaderLines',1)
figure(1);
plot(m(:,1),m(:,2))
hold on;title("Temperature");xlabel('Time [m]');ylabel('Temperature [ºC]'); hold off;

n=NaN;
for i=1:length(m)
    if m(i,2)>1
        if ~isnan(n)
            n=[n;m(i,1),m(i,2)];
        else
            n=[m(i,1),m(i,2)];
        end
    end
end
figure(2);
plot(n(:,1),n(:,2));
hold on;title("Temperature Fixed");xlabel('Time [s]');ylabel('Temperature [ºC]'); hold off;
figure(3);
plot(m(:,1),m(:,3))
hold on;title("Luminosity");xlabel('Time [s]');ylabel('Luminosity [Lux]'); hold off;
