a = 1:1:48;
b = T.A(a);
c = T.B(a);
%truth = T.tr(a);

n=48;

figure
xlim([0 48])
ylim([0 15])
hold on

for i = 1:n
    plot(a(1:i), b(1:i),'--g',a(1:i),c(1:i),'--r','LineWidth',4)
    %plot(a(1:i), b(1:i),'b','LineWidth',4)
    if i == 1 
        pause(2)
    end
    title('Depth Estimation')
    ylabel('Depth')
    xlabel('Time')
    set(gca,'FontSize',30)
    legend('UKF Estimated Depth', 'Calculated Depth')
    pause(0.5)
    
end